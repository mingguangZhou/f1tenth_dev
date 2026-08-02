#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

namespace
{
constexpr double kPi = 3.14159265358979323846;

double degreesToRadians(const double degrees)
{
  return degrees * kPi / 180.0;
}

double clampValue(const double value, const double minimum, const double maximum)
{
  return std::max(minimum, std::min(value, maximum));
}
}  // namespace

class LowerSafetyController : public rclcpp::Node
{
public:
  LowerSafetyController()
  : Node("lower_safety_controller")
  {
    declareParameters();
    loadParameters();

    command_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      selected_command_topic_, 10,
      std::bind(&LowerSafetyController::commandCallback, this, _1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LowerSafetyController::scanCallback, this, _1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&LowerSafetyController::odomCallback, this, _1));
    upper_status_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      upper_status_topic_, 10,
      std::bind(&LowerSafetyController::upperStatusCallback, this, _1));

    safe_command_pub_ =
      create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(safe_command_topic_, 10);
    status_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(lower_status_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / control_frequency_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LowerSafetyController::controlCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "reactive_control_v2 v0.2.5 lower_safety_controller ready: "
      "selected=%s, scan=%s, safe=%s, "
      "upper_failure_status_fallback=%s, "
      "command_limits=|speed|<=%.1f m/s and |steering|<=%.1f deg, ftg_debug=%s",
      selected_command_topic_.c_str(), scan_topic_.c_str(), safe_command_topic_.c_str(),
      enable_fallback_on_upper_failure_status_ ? "true" : "false",
      absolute_speed_limit_mps_, absolute_steering_limit_deg_,
      fallback_terminal_debug_ ? "true" : "false");
  }

private:
  enum class Mode
  {
    NOMINAL,
    FALLBACK_FTG,
    EMERGENCY_STOP
  };

  struct ScanData
  {
    bool valid{false};
    std::string invalid_reason;
    std::vector<double> ranges;
    std::vector<double> angles;
    std::vector<uint8_t> observed;
    std::vector<uint8_t> hit;
    double valid_ratio{0.0};
    double front_min_distance_m{std::numeric_limits<double>::infinity()};
  };

  struct FallbackResult
  {
    bool valid{false};
    std::string reason;
    double speed{0.0};
    double steering{0.0};
    double target_angle{0.0};
    double target_range{0.0};
    int gap_start{-1};
    int gap_end{-1};
    double gap_start_angle_deg{0.0};
    double gap_end_angle_deg{0.0};
    double gap_width_deg{0.0};
    double gap_mean_depth_m{0.0};
    double gap_score{0.0};
    double closest_obstacle_distance_m{std::numeric_limits<double>::infinity()};
    double closest_obstacle_angle{0.0};
    double bubble_half_angle{0.0};
  };

  // ROS interfaces
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr upper_status_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr safe_command_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Shared callback data. The current launch uses a single-threaded executor,
  // but the mutex keeps the ownership clear and remains correct if that changes.
  std::mutex mutex_;
  ackermann_msgs::msg::AckermannDriveStamped::SharedPtr latest_command_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_upper_status_time_{0, 0, RCL_ROS_TIME};
  bool command_received_{false};
  bool scan_received_{false};
  bool odom_received_{false};
  bool upper_status_received_{false};
  double current_speed_mps_{0.0};
  std::string upper_state_;

  // Topic and timing parameters
  std::string selected_command_topic_;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string upper_status_topic_;
  std::string safe_command_topic_;
  std::string lower_status_topic_;
  double control_frequency_hz_{20.0};
  double command_timeout_sec_{0.30};
  double scan_timeout_sec_{0.30};
  double odom_timeout_sec_{0.50};
  double upper_status_timeout_sec_{0.50};
  bool enable_fallback_on_upper_failure_status_{true};

  // Final command bounds. Speed remains a loose sanity bound; steering is
  // clamped to the approximately +/-25 degree physical steering limit.
  double absolute_speed_limit_mps_{20.0};
  double absolute_steering_limit_deg_{25.0};
  double absolute_steering_limit_rad_{degreesToRadians(25.0)};

  // Simple independent emergency brake. It looks only at a narrow forward
  // sector; it does not predict or validate the commanded Ackermann trajectory.
  double emergency_sector_half_angle_deg_{15.0};
  double emergency_distance_m_{0.25};
  double emergency_ttc_sec_{0.25};
  double emergency_ttc_clearance_m_{0.10};

  // Conservative Follow-the-Gap fallback parameters.
  double fallback_sector_half_angle_deg_{75.0};
  double fallback_range_cap_m_{6.0};
  double fallback_min_valid_ratio_{0.35};
  double fallback_bubble_radius_m_{0.30};
  double fallback_min_clearance_m_{0.70};
  double fallback_min_gap_width_deg_{8.0};
  double fallback_steering_gain_{1.0};
  double fallback_steering_limit_deg_{25.0};
  double fallback_steering_limit_rad_{degreesToRadians(25.0)};
  double fallback_speed_max_mps_{0.40};
  double fallback_speed_min_mps_{0.25};
  double fallback_deepest_region_ratio_{0.90};
  bool fallback_terminal_debug_{false};
  double fallback_terminal_debug_period_sec_{0.50};
  rclcpp::Time last_fallback_debug_time_{0, 0, RCL_ROS_TIME};

  // State retained for concise transition logs and future reverse/resume work.
  Mode last_mode_{Mode::EMERGENCY_STOP};
  std::string last_reason_;
  rclcpp::Time stopped_since_{0, 0, RCL_ROS_TIME};
  bool stopped_timer_active_{false};

  void declareParameters()
  {
    declare_parameter<std::string>(
      "selected_command_topic", "/reactive_control_v2/selected_cmd");
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
    declare_parameter<std::string>("upper_status_topic", "/reactive_control_v2/status");
    declare_parameter<std::string>("safe_command_topic", "/reactive_control_v2/safe_cmd");
    declare_parameter<std::string>(
      "lower_status_topic", "/reactive_control_v2/lower_safety_status");
    declare_parameter<double>("control_frequency_hz", 20.0);
    declare_parameter<double>("command_timeout_sec", 0.30);
    declare_parameter<double>("scan_timeout_sec", 0.30);
    declare_parameter<double>("odom_timeout_sec", 0.50);
    declare_parameter<double>("upper_status_timeout_sec", 0.50);
    declare_parameter<bool>("enable_fallback_on_upper_failure_status", true);

    declare_parameter<double>("absolute_speed_limit_mps", 20.0);
    declare_parameter<double>("absolute_steering_limit_deg", 25.0);

    declare_parameter<double>("emergency_sector_half_angle_deg", 15.0);
    declare_parameter<double>("emergency_distance_m", 0.25);
    declare_parameter<double>("emergency_ttc_sec", 0.25);
    declare_parameter<double>("emergency_ttc_clearance_m", 0.10);

    declare_parameter<double>("fallback_sector_half_angle_deg", 75.0);
    declare_parameter<double>("fallback_range_cap_m", 6.0);
    declare_parameter<double>("fallback_min_valid_ratio", 0.35);
    declare_parameter<double>("fallback_bubble_radius_m", 0.30);
    declare_parameter<double>("fallback_min_clearance_m", 0.70);
    declare_parameter<double>("fallback_min_gap_width_deg", 8.0);
    declare_parameter<double>("fallback_steering_gain", 1.0);
    declare_parameter<double>("fallback_steering_limit_deg", 25.0);
    declare_parameter<double>("fallback_speed_max_mps", 0.40);
    declare_parameter<double>("fallback_speed_min_mps", 0.25);
    declare_parameter<double>("fallback_deepest_region_ratio", 0.90);
    declare_parameter<bool>("fallback_terminal_debug", false);
    declare_parameter<double>("fallback_terminal_debug_period_sec", 0.50);
  }

  void loadParameters()
  {
    selected_command_topic_ = get_parameter("selected_command_topic").as_string();
    scan_topic_ = get_parameter("scan_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    upper_status_topic_ = get_parameter("upper_status_topic").as_string();
    safe_command_topic_ = get_parameter("safe_command_topic").as_string();
    lower_status_topic_ = get_parameter("lower_status_topic").as_string();
    control_frequency_hz_ = std::max(1.0, get_parameter("control_frequency_hz").as_double());
    command_timeout_sec_ = std::max(0.01, get_parameter("command_timeout_sec").as_double());
    scan_timeout_sec_ = std::max(0.01, get_parameter("scan_timeout_sec").as_double());
    odom_timeout_sec_ = std::max(0.01, get_parameter("odom_timeout_sec").as_double());
    upper_status_timeout_sec_ =
      std::max(0.01, get_parameter("upper_status_timeout_sec").as_double());
    enable_fallback_on_upper_failure_status_ =
      get_parameter("enable_fallback_on_upper_failure_status").as_bool();

    absolute_speed_limit_mps_ =
      std::max(0.1, get_parameter("absolute_speed_limit_mps").as_double());
    absolute_steering_limit_deg_ =
      std::max(1.0, get_parameter("absolute_steering_limit_deg").as_double());
    absolute_steering_limit_rad_ = degreesToRadians(absolute_steering_limit_deg_);

    emergency_sector_half_angle_deg_ = clampValue(
      get_parameter("emergency_sector_half_angle_deg").as_double(), 1.0, 90.0);
    emergency_distance_m_ =
      std::max(0.0, get_parameter("emergency_distance_m").as_double());
    emergency_ttc_sec_ = std::max(0.0, get_parameter("emergency_ttc_sec").as_double());
    emergency_ttc_clearance_m_ =
      std::max(0.0, get_parameter("emergency_ttc_clearance_m").as_double());

    fallback_sector_half_angle_deg_ = clampValue(
      get_parameter("fallback_sector_half_angle_deg").as_double(), 5.0, 170.0);
    fallback_range_cap_m_ =
      std::max(0.1, get_parameter("fallback_range_cap_m").as_double());
    fallback_min_valid_ratio_ = clampValue(
      get_parameter("fallback_min_valid_ratio").as_double(), 0.0, 1.0);
    fallback_bubble_radius_m_ =
      std::max(0.0, get_parameter("fallback_bubble_radius_m").as_double());
    fallback_min_clearance_m_ =
      std::max(0.0, get_parameter("fallback_min_clearance_m").as_double());
    fallback_min_gap_width_deg_ =
      std::max(0.0, get_parameter("fallback_min_gap_width_deg").as_double());
    fallback_steering_gain_ =
      std::max(0.0, get_parameter("fallback_steering_gain").as_double());
    fallback_steering_limit_deg_ =
      std::max(1.0, get_parameter("fallback_steering_limit_deg").as_double());
    fallback_steering_limit_rad_ = degreesToRadians(fallback_steering_limit_deg_);
    fallback_speed_max_mps_ =
      std::max(0.0, get_parameter("fallback_speed_max_mps").as_double());
    fallback_speed_min_mps_ = clampValue(
      get_parameter("fallback_speed_min_mps").as_double(), 0.0, fallback_speed_max_mps_);
    fallback_deepest_region_ratio_ = clampValue(
      get_parameter("fallback_deepest_region_ratio").as_double(), 0.50, 1.0);
    fallback_terminal_debug_ = get_parameter("fallback_terminal_debug").as_bool();
    fallback_terminal_debug_period_sec_ = std::max(
      0.05, get_parameter("fallback_terminal_debug_period_sec").as_double());
  }

  void commandCallback(
    const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr command)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_command_ = command;
    last_command_time_ = now();
    command_received_ = true;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_scan_ = scan;
    last_scan_time_ = now();
    scan_received_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_speed_mps_ = odom->twist.twist.linear.x;
    last_odom_time_ = now();
    odom_received_ = true;
  }

  void upperStatusCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr array)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & status : array->status) {
      if (status.name != "reactive_control_v2/upper_corridor_follower") {
        continue;
      }
      std::string state;
      for (const auto & value : status.values) {
        if (value.key == "state") {
          state = value.value;
        }
      }
      // Fall back to the text message only for diagnostics produced by an
      // older compatible upper controller without key/value state fields.
      if (state.empty()) {
        const auto separator = status.message.find(':');
        state = status.message.substr(0, separator);
      }
      upper_state_ = state;
      last_upper_status_time_ = now();
      upper_status_received_ = true;
      break;
    }
  }

  ScanData preprocessScan(const sensor_msgs::msg::LaserScan & scan) const
  {
    ScanData output;
    if (scan.ranges.empty() || !std::isfinite(scan.angle_increment) ||
      scan.angle_increment <= 0.0)
    {
      output.invalid_reason = "empty or malformed LaserScan";
      return output;
    }

    const double sector_limit = degreesToRadians(fallback_sector_half_angle_deg_);
    const double emergency_limit = degreesToRadians(emergency_sector_half_angle_deg_);
    const bool range_max_valid = std::isfinite(scan.range_max) && scan.range_max > 0.0;
    const double usable_max = range_max_valid ?
      std::min(fallback_range_cap_m_, static_cast<double>(scan.range_max)) :
      fallback_range_cap_m_;
    const double range_min =
      std::isfinite(scan.range_min) ? std::max(0.0, static_cast<double>(scan.range_min)) : 0.0;

    size_t sector_beams = 0;
    size_t valid_beams = 0;
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      if (std::abs(angle) > sector_limit) {
        continue;
      }
      ++sector_beams;
      const double raw = scan.ranges[i];
      const bool finite_hit = std::isfinite(raw) && raw >= range_min &&
        (!range_max_valid || raw <= scan.range_max);
      const bool clear_to_max = std::isinf(raw) ||
        (std::isfinite(raw) && range_max_valid && raw > scan.range_max);

      output.angles.push_back(angle);
      if (finite_hit) {
        output.ranges.push_back(std::min(raw, usable_max));
        output.observed.push_back(1);
        output.hit.push_back(raw < usable_max ? 1 : 0);
        ++valid_beams;
        if (std::abs(angle) <= emergency_limit) {
          output.front_min_distance_m = std::min(output.front_min_distance_m, raw);
        }
      } else if (clear_to_max) {
        output.ranges.push_back(usable_max);
        output.observed.push_back(1);
        output.hit.push_back(0);
        ++valid_beams;
      } else {
        output.ranges.push_back(0.0);
        output.observed.push_back(0);
        output.hit.push_back(0);
      }
    }

    if (sector_beams == 0) {
      output.invalid_reason = "no LaserScan beams in fallback sector";
      return output;
    }
    output.valid_ratio = static_cast<double>(valid_beams) / static_cast<double>(sector_beams);
    if (output.valid_ratio < fallback_min_valid_ratio_) {
      output.invalid_reason = "too few valid LaserScan beams";
      return output;
    }
    output.valid = true;
    return output;
  }

  FallbackResult makeFallback(const ScanData & scan) const
  {
    FallbackResult result;
    if (!scan.valid || scan.ranges.empty()) {
      result.reason = scan.invalid_reason.empty() ? "invalid scan" : scan.invalid_reason;
      return result;
    }

    std::vector<double> ranges = scan.ranges;

    // Apply the safety bubble only around the nearest actual return. Infinite
    // or cap-limited clear beams are deliberately not treated as obstacles.
    size_t closest_index = ranges.size();
    double closest_distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < ranges.size(); ++i) {
      if (scan.observed[i] && scan.hit[i] && ranges[i] < closest_distance) {
        closest_distance = ranges[i];
        closest_index = i;
      }
    }
    if (closest_index < ranges.size()) {
      const double bubble_half_angle = std::atan2(
        fallback_bubble_radius_m_, std::max(0.01, closest_distance));
      const double obstacle_angle = scan.angles[closest_index];
      result.closest_obstacle_distance_m = closest_distance;
      result.closest_obstacle_angle = obstacle_angle;
      result.bubble_half_angle = bubble_half_angle;
      for (size_t i = 0; i < ranges.size(); ++i) {
        if (std::abs(scan.angles[i] - obstacle_angle) <= bubble_half_angle) {
          ranges[i] = 0.0;
        }
      }
    }

    // Find every contiguous set of observed beams with enough clearance, then
    // score complete gaps using angular width and mean/max depth. Width has the
    // largest influence; depth makes a genuinely open side gap beat a broad but
    // shallow forward region. The small centre penalty only settles otherwise
    // similar choices and must not prevent a decisive turn into free space.
    int best_start = -1;
    int best_end = -1;
    int current_start = -1;
    double best_width = 0.0;
    double best_mean_depth = 0.0;
    double best_score = -std::numeric_limits<double>::infinity();
    const double sector_span = std::max(
      1e-6, scan.angles.back() - scan.angles.front());
    const double sector_half_angle = degreesToRadians(fallback_sector_half_angle_deg_);
    for (size_t i = 0; i <= ranges.size(); ++i) {
      const bool free = i < ranges.size() && scan.observed[i] &&
        ranges[i] >= fallback_min_clearance_m_;
      if (free && current_start < 0) {
        current_start = static_cast<int>(i);
      }
      if ((!free || i == ranges.size()) && current_start >= 0) {
        const int end = static_cast<int>(i) - 1;
        const double width = scan.angles[static_cast<size_t>(end)] -
          scan.angles[static_cast<size_t>(current_start)];
        double depth_sum = 0.0;
        double max_depth = 0.0;
        for (int j = current_start; j <= end; ++j) {
          const double depth = ranges[static_cast<size_t>(j)];
          depth_sum += depth;
          max_depth = std::max(max_depth, depth);
        }
        const double beam_count = static_cast<double>(end - current_start + 1);
        const double mean_depth = depth_sum / std::max(1.0, beam_count);
        const double center = 0.5 * (
          scan.angles[static_cast<size_t>(current_start)] +
          scan.angles[static_cast<size_t>(end)]);
        const double width_score = clampValue(width / sector_span, 0.0, 1.0);
        const double mean_depth_score = clampValue(
          mean_depth / fallback_range_cap_m_, 0.0, 1.0);
        const double max_depth_score = clampValue(
          max_depth / fallback_range_cap_m_, 0.0, 1.0);
        const double centre_cost = clampValue(
          std::abs(center) / std::max(1e-6, sector_half_angle), 0.0, 1.0);
        const double score = 0.55 * width_score + 0.30 * mean_depth_score +
          0.15 * max_depth_score - 0.05 * centre_cost;
        if (width >= degreesToRadians(fallback_min_gap_width_deg_) &&
          score > best_score + 1e-6)
        {
          best_score = score;
          best_width = width;
          best_mean_depth = mean_depth;
          best_start = current_start;
          best_end = end;
        }
        current_start = -1;
      }
    }

    if (best_start < 0 || best_end < best_start ||
      best_width < degreesToRadians(fallback_min_gap_width_deg_))
    {
      result.reason = "no sufficiently wide fallback gap";
      return result;
    }

    // Aim at the centre of the deepest region inside the chosen gap. This is
    // intentionally different from choosing the deepest beam nearest zero:
    // that old tie-break kept pointing almost straight when many open beams
    // were range-capped. Averaging the deepest plateau produces a target well
    // inside the selected opening and therefore a clear turn into the gap.
    double target_range = 0.0;
    for (int i = best_start; i <= best_end; ++i) {
      target_range = std::max(target_range, ranges[static_cast<size_t>(i)]);
    }
    const double deepest_threshold = fallback_deepest_region_ratio_ * target_range;
    double weighted_index_sum = 0.0;
    double weight_sum = 0.0;
    for (int i = best_start; i <= best_end; ++i) {
      const double candidate = ranges[static_cast<size_t>(i)];
      if (candidate + 1e-6 < deepest_threshold) {
        continue;
      }
      const double weight = std::max(1e-6, candidate);
      weighted_index_sum += static_cast<double>(i) * weight;
      weight_sum += weight;
    }
    int target_index = weight_sum > 0.0 ?
      static_cast<int>(std::lround(weighted_index_sum / weight_sum)) :
      (best_start + best_end) / 2;
    // Keep the target off the immediate gap edge when enough beams exist.
    if (best_end - best_start >= 4) {
      target_index = std::max(best_start + 2, std::min(target_index, best_end - 2));
    }

    result.valid = true;
    result.reason = "nominal unavailable; safe fallback gap selected";
    result.target_angle = scan.angles[static_cast<size_t>(target_index)];
    result.target_range = ranges[static_cast<size_t>(target_index)];
    result.gap_start = best_start;
    result.gap_end = best_end;
    result.gap_start_angle_deg = scan.angles[static_cast<size_t>(best_start)] * 180.0 / kPi;
    result.gap_end_angle_deg = scan.angles[static_cast<size_t>(best_end)] * 180.0 / kPi;
    result.gap_width_deg = best_width * 180.0 / kPi;
    result.gap_mean_depth_m = best_mean_depth;
    result.gap_score = best_score;
    result.steering = clampValue(
      fallback_steering_gain_ * result.target_angle,
      -fallback_steering_limit_rad_, fallback_steering_limit_rad_);
    const double steering_ratio = clampValue(
      std::abs(result.steering) / fallback_steering_limit_rad_, 0.0, 1.0);
    result.speed = fallback_speed_max_mps_ - steering_ratio *
      (fallback_speed_max_mps_ - fallback_speed_min_mps_);
    return result;
  }

  void logFallbackDebug(
    const Mode mode, const ScanData & scan, const FallbackResult & fallback,
    const ackermann_msgs::msg::AckermannDriveStamped & output)
  {
    if (!fallback_terminal_debug_) {
      return;
    }
    const rclcpp::Time current_time = now();
    if (last_fallback_debug_time_.nanoseconds() != 0 &&
      (current_time - last_fallback_debug_time_).seconds() <
      fallback_terminal_debug_period_sec_)
    {
      return;
    }
    last_fallback_debug_time_ = current_time;

    const double target_deg = fallback.target_angle * 180.0 / kPi;
    const double steering_deg = output.drive.steering_angle * 180.0 / kPi;
    if (fallback.valid) {
      RCLCPP_INFO(
        get_logger(),
        "FTG debug | mode=%s valid_ratio=%.2f front_min=%.2f m | "
        "nearest=(%.2f m,%+.1f deg) bubble_half=%.1f deg | "
        "gap=[%+.1f,%+.1f deg] width=%.1f deg mean_depth=%.2f m score=%.3f | "
        "target=(%+.1f deg,%.2f m) cmd=(%.2f m/s,%+.1f deg)",
        modeName(mode), scan.valid_ratio, scan.front_min_distance_m,
        fallback.closest_obstacle_distance_m,
        fallback.closest_obstacle_angle * 180.0 / kPi,
        fallback.bubble_half_angle * 180.0 / kPi,
        fallback.gap_start_angle_deg, fallback.gap_end_angle_deg, fallback.gap_width_deg,
        fallback.gap_mean_depth_m, fallback.gap_score,
        target_deg, fallback.target_range, output.drive.speed, steering_deg);
    } else {
      RCLCPP_WARN(
        get_logger(), "FTG debug | mode=%s unavailable | scan_valid=%s "
        "valid_ratio=%.2f front_min=%.2f m reason=%s",
        modeName(mode), scan.valid ? "true" : "false", scan.valid_ratio,
        scan.front_min_distance_m, fallback.reason.c_str());
    }
  }

  bool upperRequestsFallback(
    const bool status_received, const double status_age,
    const std::string & state) const
  {
    if (!enable_fallback_on_upper_failure_status_ || !status_received ||
      status_age > upper_status_timeout_sec_)
    {
      return false;
    }
    // Only planner failures which may still leave a simple LiDAR gap request
    // FTG. DISABLED, stale/invalid scan, TF and odometry failures remain STOP.
    return state == "PATH_INVALID" || state == "BLOCKED";
  }

  bool commandNumericallyValid(
    const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr & command) const
  {
    return command && std::isfinite(command->drive.speed) &&
           std::isfinite(command->drive.steering_angle);
  }

  ackermann_msgs::msg::AckermannDriveStamped boundedNominal(
    const ackermann_msgs::msg::AckermannDriveStamped & input) const
  {
    // Preserve every upper-controller field and change only the two values for
    // which this final gateway owns loose absolute sanity limits.
    auto output = input;
    output.header.stamp = now();
    output.drive.speed = clampValue(
      output.drive.speed, -absolute_speed_limit_mps_, absolute_speed_limit_mps_);
    output.drive.steering_angle = clampValue(
      output.drive.steering_angle,
      -absolute_steering_limit_rad_, absolute_steering_limit_rad_);
    return output;
  }

  ackermann_msgs::msg::AckermannDriveStamped fallbackCommand(
    const FallbackResult & fallback) const
  {
    ackermann_msgs::msg::AckermannDriveStamped output;
    output.header.stamp = now();
    output.header.frame_id = "base_link";
    output.drive.speed = fallback.speed;
    output.drive.steering_angle = fallback.steering;
    return output;
  }

  ackermann_msgs::msg::AckermannDriveStamped stopCommand() const
  {
    ackermann_msgs::msg::AckermannDriveStamped output;
    output.header.stamp = now();
    output.header.frame_id = "base_link";
    output.drive.speed = 0.0;
    output.drive.steering_angle = 0.0;
    return output;
  }

  const char * modeName(const Mode mode) const
  {
    switch (mode) {
      case Mode::NOMINAL: return "NOMINAL";
      case Mode::FALLBACK_FTG: return "FALLBACK_FTG";
      case Mode::EMERGENCY_STOP: return "EMERGENCY_STOP";
    }
    return "UNKNOWN";
  }

  void publishStatus(
    const Mode mode, const std::string & reason, const ScanData & scan,
    const FallbackResult & fallback, const double command_age,
    const double scan_age, const double current_speed, const std::string & upper_state)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "reactive_control_v2/lower_safety_controller";
    status.hardware_id = "final_command_gateway";
    status.level = mode == Mode::EMERGENCY_STOP ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = std::string(modeName(mode)) + ": " + reason;

    auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue pair;
        pair.key = key;
        pair.value = value;
        status.values.push_back(pair);
      };
    add("mode", modeName(mode));
    add("reason", reason);
    add(
      "upper_failure_status_fallback_enabled",
      enable_fallback_on_upper_failure_status_ ? "true" : "false");
    add("upper_state", upper_state.empty() ? "unavailable" : upper_state);
    add("command_age_sec", std::to_string(command_age));
    add("scan_age_sec", std::to_string(scan_age));
    add("scan_valid_ratio", std::to_string(scan.valid_ratio));
    add(
      "front_min_distance_m",
      std::isfinite(scan.front_min_distance_m) ?
      std::to_string(scan.front_min_distance_m) : "inf");
    add("current_speed_mps", std::to_string(current_speed));
    add("fallback_available", fallback.valid ? "true" : "false");
    add("fallback_target_angle_rad", std::to_string(fallback.target_angle));
    add("fallback_target_range_m", std::to_string(fallback.target_range));
    add("fallback_gap_width_deg", std::to_string(fallback.gap_width_deg));
    add("fallback_gap_start_angle_deg", std::to_string(fallback.gap_start_angle_deg));
    add("fallback_gap_end_angle_deg", std::to_string(fallback.gap_end_angle_deg));
    add("fallback_gap_mean_depth_m", std::to_string(fallback.gap_mean_depth_m));
    add("fallback_gap_score", std::to_string(fallback.gap_score));
    add("fallback_terminal_debug_enabled", fallback_terminal_debug_ ? "true" : "false");
    const double stop_duration = stopped_timer_active_ ?
      std::max(0.0, (now() - stopped_since_).seconds()) : 0.0;
    add("stop_duration_sec", std::to_string(stop_duration));
    // stop_duration, current speed, front distance and fallback availability
    // are intentionally exposed for the later dead-end/reverse state machine.
    array.status.push_back(status);
    status_pub_->publish(array);
  }

  void logTransition(const Mode mode, const std::string & reason)
  {
    if (mode == last_mode_ && reason == last_reason_) {
      return;
    }
    if (mode == Mode::EMERGENCY_STOP) {
      RCLCPP_WARN(get_logger(), "mode=%s -> STOP | reason=%s", modeName(mode), reason.c_str());
    } else if (mode == Mode::FALLBACK_FTG) {
      RCLCPP_WARN(get_logger(), "mode=%s | reason=%s", modeName(mode), reason.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "mode=%s | reason=%s", modeName(mode), reason.c_str());
    }
    last_mode_ = mode;
    last_reason_ = reason;
  }

  void controlCallback()
  {
    ackermann_msgs::msg::AckermannDriveStamped::SharedPtr command;
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    rclcpp::Time command_time(0, 0, RCL_ROS_TIME);
    rclcpp::Time scan_time(0, 0, RCL_ROS_TIME);
    rclcpp::Time status_time(0, 0, RCL_ROS_TIME);
    rclcpp::Time odom_time(0, 0, RCL_ROS_TIME);
    bool command_received = false;
    bool scan_received = false;
    bool status_received = false;
    bool odom_received = false;
    double current_speed = 0.0;
    std::string upper_state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command = latest_command_;
      scan = latest_scan_;
      command_time = last_command_time_;
      scan_time = last_scan_time_;
      status_time = last_upper_status_time_;
      odom_time = last_odom_time_;
      command_received = command_received_;
      scan_received = scan_received_;
      status_received = upper_status_received_;
      odom_received = odom_received_;
      current_speed = current_speed_mps_;
      upper_state = upper_state_;
    }

    const rclcpp::Time current_time = now();
    const double command_age = command_received ?
      (current_time - command_time).seconds() : std::numeric_limits<double>::infinity();
    const double scan_age = scan_received ?
      (current_time - scan_time).seconds() : std::numeric_limits<double>::infinity();
    const double status_age = status_received ?
      (current_time - status_time).seconds() : std::numeric_limits<double>::infinity();
    const double odom_age = odom_received ?
      (current_time - odom_time).seconds() : std::numeric_limits<double>::infinity();
    if (!odom_received || odom_age > odom_timeout_sec_ || !std::isfinite(current_speed)) {
      // Odometry improves TTC estimation but is never required for nominal or
      // fallback operation. A stale value must not create a false emergency.
      current_speed = 0.0;
    }

    ScanData scan_data;
    if (scan_received && scan_age <= scan_timeout_sec_ && scan) {
      scan_data = preprocessScan(*scan);
    } else {
      scan_data.invalid_reason = scan_received ? "LaserScan timeout" : "no LaserScan received";
    }
    const FallbackResult fallback = makeFallback(scan_data);

    Mode mode = Mode::EMERGENCY_STOP;
    std::string reason;
    ackermann_msgs::msg::AckermannDriveStamped output;

    if (!scan_data.valid) {
      reason = scan_data.invalid_reason;
      output = stopCommand();
    } else {
      const double nominal_forward_speed = commandNumericallyValid(command) ?
        std::max(0.0, static_cast<double>(command->drive.speed)) : 0.0;
      const double forward_speed = std::max(std::max(0.0, current_speed), nominal_forward_speed);
      const double available_distance = std::isfinite(scan_data.front_min_distance_m) ?
        std::max(0.0, scan_data.front_min_distance_m - emergency_ttc_clearance_m_) :
        std::numeric_limits<double>::infinity();
      const double ttc = forward_speed > 0.05 ?
        available_distance / forward_speed : std::numeric_limits<double>::infinity();

      if (scan_data.front_min_distance_m < emergency_distance_m_) {
        reason = "obstacle inside hard forward emergency distance";
        output = stopCommand();
      } else if (ttc < emergency_ttc_sec_) {
        reason = "forward time-to-collision below emergency threshold";
        output = stopCommand();
      } else {
        const bool command_fresh = command_received && command_age <= command_timeout_sec_;
        const bool command_valid = commandNumericallyValid(command);
        const bool explicit_fallback = upperRequestsFallback(
          status_received, status_age, upper_state);
        if (command_fresh && command_valid && !explicit_fallback) {
          mode = Mode::NOMINAL;
          reason = "fresh finite selected command passed through";
          output = boundedNominal(*command);
        } else if (fallback.valid) {
          mode = Mode::FALLBACK_FTG;
          if (explicit_fallback) {
            reason = "upper state " + upper_state + "; using conservative FTG";
          } else if (!command_received || !command_fresh) {
            reason = "selected command unavailable or stale; using conservative FTG";
          } else {
            reason = "selected command contains non-finite value; using conservative FTG";
          }
          output = fallbackCommand(fallback);
        } else {
          reason = fallback.reason.empty() ? "no valid nominal or fallback command" : fallback.reason;
          output = stopCommand();
        }
      }
    }

    if (mode == Mode::EMERGENCY_STOP) {
      if (!stopped_timer_active_) {
        stopped_since_ = current_time;
        stopped_timer_active_ = true;
      }
    } else {
      stopped_timer_active_ = false;
    }

    safe_command_pub_->publish(output);
    logFallbackDebug(mode, scan_data, fallback, output);
    publishStatus(
      mode, reason, scan_data, fallback, command_age, scan_age, current_speed, upper_state);
    logTransition(mode, reason);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LowerSafetyController>());
  rclcpp::shutdown();
  return 0;
}
