#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr char kPackageVersion[] = "0.1.3";

double degToRad(const double degrees)
{
  return degrees * kPi / 180.0;
}

double clampValue(const double value, const double low, const double high)
{
  return std::max(low, std::min(value, high));
}

std::string numberString(const double value)
{
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.3f", value);
  return std::string(buffer);
}

struct Point2
{
  double x{0.0};
  double y{0.0};
};

struct Interval
{
  double x{0.0};
  double low{0.0};
  double high{0.0};

  double center() const {return 0.5 * (low + high);}
  double width() const {return high - low;}
};

struct Branch
{
  std::vector<Interval> intervals;
  double min_width{std::numeric_limits<double>::infinity()};
  double lateral_motion{0.0};

  double reach() const
  {
    return intervals.empty() ? 0.0 : intervals.back().x;
  }
};

struct BeamData
{
  std::vector<double> ranges;
  std::vector<uint8_t> observed;
  std::vector<uint8_t> hit;
  std::vector<Point2> obstacle_points;
  double valid_ratio{0.0};
};

struct ValidationFailure
{
  bool failed{false};
  std::string path_source{"none"};
  std::string check_code{"NONE"};
  size_t segment_index{0};
  size_t segment_count{0};
  int sample_index{0};
  int samples_in_segment{0};
  Point2 segment_start;
  Point2 segment_end;
  Point2 point;
  double segment_length{0.0};
  double segment_heading_deg{0.0};
  int beam_index{-1};
  double beam_angle_deg{0.0};
  double point_range_m{0.0};
  double observed_range_m{0.0};
  double radial_clearance_m{std::numeric_limits<double>::infinity()};
  double nearest_obstacle_distance_m{std::numeric_limits<double>::infinity()};
  double required_clearance_m{0.0};
};

struct PlanResult
{
  bool valid{false};
  std::string state{"BLOCKED"};
  std::string reason{"no connected corridor"};
  Branch branch;
  std::vector<Point2> path;
  Point2 lookahead;
  double scan_valid_ratio{0.0};
  double furthest_detected_reach{0.0};
  size_t detected_branches{0};
  size_t usable_branches{0};
  double steering{0.0};
  double speed{0.0};
  bool used_raw_fallback{false};
  ValidationFailure smoothed_failure;
  ValidationFailure raw_failure;
};

}  // namespace

class CorridorPlannerNode : public rclcpp::Node
{
public:
  CorridorPlannerNode()
  : Node("corridor_planner")
  {
    declareParameters();
    readParameters();
    validateParameters();

    command_pub_ =
      create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(command_topic_, 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
    marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    status_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(status_topic_, 10);

    using std::placeholders::_1;
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CorridorPlannerNode::scanCallback, this, _1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&CorridorPlannerNode::odomCallback, this, _1));
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_, 10, std::bind(&CorridorPlannerNode::enableCallback, this, _1));

    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&CorridorPlannerNode::watchdogCallback, this));

    enabled_ = !require_enable_message_;
    last_scan_receive_time_ = now();
    last_odom_receive_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "reactive_control_v2 v%s ready: scan=%s, command=%s, "
      "envelope=%.2f m/side, terminal_status=%.1f s",
      kPackageVersion, scan_topic_.c_str(), command_topic_.c_str(),
      envelope_radius_, terminal_status_period_sec_);
    RCLCPP_INFO(
      get_logger(),
      "Terminal diagnostics enabled: the first planning result prints immediately.");
  }

private:
  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Topic and input parameters
  std::string scan_topic_;
  std::string odom_topic_;
  std::string enable_topic_;
  std::string command_topic_;
  std::string path_topic_;
  std::string marker_topic_;
  std::string status_topic_;
  bool require_enable_message_{false};
  double scan_timeout_sec_{0.3};
  double odom_timeout_sec_{0.5};
  bool require_fresh_odom_{false};
  double planning_angle_min_rad_{-degToRad(100.0)};
  double planning_angle_max_rad_{degToRad(100.0)};
  double scan_range_cap_m_{6.0};
  int median_filter_window_{3};
  double min_valid_beam_ratio_{0.35};

  // Geometry and corridor parameters
  double vehicle_width_m_{0.32};
  double lateral_safety_margin_m_{0.10};
  double obstacle_endpoint_margin_m_{0.03};
  double envelope_radius_{0.26};
  double forward_start_m_{0.25};
  double forward_max_m_{5.0};
  double forward_slice_step_m_{0.20};
  double lateral_limit_m_{2.0};
  double lateral_sample_step_m_{0.05};
  double min_interval_width_m_{0.10};
  double max_interval_shift_per_slice_m_{0.25};
  double min_path_reach_m_{0.80};

  // Selection parameters
  double reach_tie_tolerance_m_{0.30};
  double width_tie_tolerance_m_{0.08};
  double switch_reach_advantage_m_{0.50};
  int switch_confirmation_cycles_{3};
  double side_deadband_m_{0.12};

  // Smoothing and control parameters
  int spatial_smoothing_passes_{3};
  double spatial_smoothing_weight_{0.45};
  double temporal_smoothing_alpha_{0.40};
  int path_start_anchor_points_{2};
  double lookahead_distance_m_{0.50};
  double wheelbase_m_{0.33};
  double steering_max_rad_{degToRad(20.6)};
  double steering_filter_alpha_{0.55};
  double velocity_max_mps_{1.50};
  double velocity_min_mps_{1.00};
  double slow_reach_distance_m_{1.50};
  double stop_reach_distance_m_{0.60};

  // Debug parameters
  bool publish_visualization_{true};
  double marker_lifetime_sec_{0.25};
  double terminal_status_period_sec_{2.0};

  // Runtime state
  std::mutex mutex_;
  bool enabled_{true};
  bool scan_received_{false};
  bool odom_received_{false};
  rclcpp::Time last_scan_receive_time_;
  rclcpp::Time last_odom_receive_time_;
  double current_speed_mps_{0.0};
  std::vector<Point2> previous_path_;
  int current_side_{0};
  int pending_side_{0};
  int pending_side_cycles_{0};
  double previous_steering_{0.0};
  std::string last_frame_id_;
  std::string last_terminal_state_;
  std::string last_terminal_reason_;
  bool first_scan_logged_{false};
  std::chrono::steady_clock::time_point last_terminal_log_time_{
    std::chrono::steady_clock::now()};

  void declareParameters()
  {
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
    declare_parameter<std::string>("enable_topic", "/reactive_control_v2/enable");
    declare_parameter<std::string>(
      "command_topic", "/reactive_control_v2/nominal_cmd");
    declare_parameter<std::string>("path_topic", "/reactive_control_v2/local_path");
    declare_parameter<std::string>("marker_topic", "/reactive_control_v2/markers");
    declare_parameter<std::string>("status_topic", "/reactive_control_v2/status");
    declare_parameter<bool>("require_enable_message", false);

    declare_parameter<double>("scan_timeout_sec", 0.30);
    declare_parameter<double>("odom_timeout_sec", 0.50);
    declare_parameter<bool>("require_fresh_odom", false);
    declare_parameter<double>("planning_angle_min_deg", -100.0);
    declare_parameter<double>("planning_angle_max_deg", 100.0);
    declare_parameter<double>("scan_range_cap_m", 6.0);
    declare_parameter<int>("median_filter_window", 3);
    declare_parameter<double>("min_valid_beam_ratio", 0.35);

    declare_parameter<double>("vehicle_width_m", 0.32);
    declare_parameter<double>("lateral_safety_margin_m", 0.10);
    declare_parameter<double>("obstacle_endpoint_margin_m", 0.03);
    declare_parameter<double>("forward_start_m", 0.25);
    declare_parameter<double>("forward_max_m", 5.0);
    declare_parameter<double>("forward_slice_step_m", 0.20);
    declare_parameter<double>("lateral_limit_m", 2.0);
    declare_parameter<double>("lateral_sample_step_m", 0.05);
    declare_parameter<double>("min_interval_width_m", 0.10);
    declare_parameter<double>("max_interval_shift_per_slice_m", 0.25);
    declare_parameter<double>("min_path_reach_m", 0.80);

    declare_parameter<double>("reach_tie_tolerance_m", 0.30);
    declare_parameter<double>("width_tie_tolerance_m", 0.08);
    declare_parameter<double>("switch_reach_advantage_m", 0.50);
    declare_parameter<int>("switch_confirmation_cycles", 3);
    declare_parameter<double>("side_deadband_m", 0.12);

    declare_parameter<int>("spatial_smoothing_passes", 3);
    declare_parameter<double>("spatial_smoothing_weight", 0.45);
    declare_parameter<double>("temporal_smoothing_alpha", 0.40);
    declare_parameter<int>("path_start_anchor_points", 2);
    declare_parameter<double>("lookahead_distance_m", 0.50);
    declare_parameter<double>("wheelbase_m", 0.33);
    declare_parameter<double>("steering_max_deg", 20.6);
    declare_parameter<double>("steering_filter_alpha", 0.55);
    declare_parameter<double>("velocity_max_mps", 1.50);
    declare_parameter<double>("velocity_min_mps", 1.00);
    declare_parameter<double>("slow_reach_distance_m", 1.50);
    declare_parameter<double>("stop_reach_distance_m", 0.60);

    declare_parameter<bool>("publish_visualization", true);
    declare_parameter<double>("marker_lifetime_sec", 0.25);
    declare_parameter<double>("terminal_status_period_sec", 2.0);
  }

  void readParameters()
  {
    scan_topic_ = get_parameter("scan_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    enable_topic_ = get_parameter("enable_topic").as_string();
    command_topic_ = get_parameter("command_topic").as_string();
    path_topic_ = get_parameter("path_topic").as_string();
    marker_topic_ = get_parameter("marker_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    require_enable_message_ = get_parameter("require_enable_message").as_bool();

    scan_timeout_sec_ = get_parameter("scan_timeout_sec").as_double();
    odom_timeout_sec_ = get_parameter("odom_timeout_sec").as_double();
    require_fresh_odom_ = get_parameter("require_fresh_odom").as_bool();
    planning_angle_min_rad_ = degToRad(get_parameter("planning_angle_min_deg").as_double());
    planning_angle_max_rad_ = degToRad(get_parameter("planning_angle_max_deg").as_double());
    scan_range_cap_m_ = get_parameter("scan_range_cap_m").as_double();
    median_filter_window_ = get_parameter("median_filter_window").as_int();
    min_valid_beam_ratio_ = get_parameter("min_valid_beam_ratio").as_double();

    vehicle_width_m_ = get_parameter("vehicle_width_m").as_double();
    lateral_safety_margin_m_ = get_parameter("lateral_safety_margin_m").as_double();
    obstacle_endpoint_margin_m_ =
      get_parameter("obstacle_endpoint_margin_m").as_double();
    envelope_radius_ = 0.5 * vehicle_width_m_ + lateral_safety_margin_m_;
    forward_start_m_ = get_parameter("forward_start_m").as_double();
    forward_max_m_ = get_parameter("forward_max_m").as_double();
    forward_slice_step_m_ = get_parameter("forward_slice_step_m").as_double();
    lateral_limit_m_ = get_parameter("lateral_limit_m").as_double();
    lateral_sample_step_m_ = get_parameter("lateral_sample_step_m").as_double();
    min_interval_width_m_ = get_parameter("min_interval_width_m").as_double();
    max_interval_shift_per_slice_m_ =
      get_parameter("max_interval_shift_per_slice_m").as_double();
    min_path_reach_m_ = get_parameter("min_path_reach_m").as_double();

    reach_tie_tolerance_m_ = get_parameter("reach_tie_tolerance_m").as_double();
    width_tie_tolerance_m_ = get_parameter("width_tie_tolerance_m").as_double();
    switch_reach_advantage_m_ =
      get_parameter("switch_reach_advantage_m").as_double();
    switch_confirmation_cycles_ = get_parameter("switch_confirmation_cycles").as_int();
    side_deadband_m_ = get_parameter("side_deadband_m").as_double();

    spatial_smoothing_passes_ = get_parameter("spatial_smoothing_passes").as_int();
    spatial_smoothing_weight_ = get_parameter("spatial_smoothing_weight").as_double();
    temporal_smoothing_alpha_ = get_parameter("temporal_smoothing_alpha").as_double();
    path_start_anchor_points_ = get_parameter("path_start_anchor_points").as_int();
    lookahead_distance_m_ = get_parameter("lookahead_distance_m").as_double();
    wheelbase_m_ = get_parameter("wheelbase_m").as_double();
    steering_max_rad_ = degToRad(get_parameter("steering_max_deg").as_double());
    steering_filter_alpha_ = get_parameter("steering_filter_alpha").as_double();
    velocity_max_mps_ = get_parameter("velocity_max_mps").as_double();
    velocity_min_mps_ = get_parameter("velocity_min_mps").as_double();
    slow_reach_distance_m_ = get_parameter("slow_reach_distance_m").as_double();
    stop_reach_distance_m_ = get_parameter("stop_reach_distance_m").as_double();

    publish_visualization_ = get_parameter("publish_visualization").as_bool();
    marker_lifetime_sec_ = get_parameter("marker_lifetime_sec").as_double();
    terminal_status_period_sec_ =
      get_parameter("terminal_status_period_sec").as_double();
  }

  void validateParameters()
  {
    forward_slice_step_m_ = std::max(0.05, forward_slice_step_m_);
    lateral_sample_step_m_ = std::max(0.02, lateral_sample_step_m_);
    forward_start_m_ = std::max(forward_slice_step_m_, forward_start_m_);
    forward_max_m_ = std::max(forward_start_m_ + forward_slice_step_m_, forward_max_m_);
    lateral_limit_m_ = std::max(envelope_radius_ + 0.1, lateral_limit_m_);
    median_filter_window_ = std::max(1, median_filter_window_);
    if (median_filter_window_ % 2 == 0) {
      ++median_filter_window_;
    }
    min_valid_beam_ratio_ = clampValue(min_valid_beam_ratio_, 0.0, 1.0);
    spatial_smoothing_weight_ = clampValue(spatial_smoothing_weight_, 0.0, 1.0);
    temporal_smoothing_alpha_ = clampValue(temporal_smoothing_alpha_, 0.0, 1.0);
    steering_filter_alpha_ = clampValue(steering_filter_alpha_, 0.0, 1.0);
    switch_confirmation_cycles_ = std::max(1, switch_confirmation_cycles_);
    velocity_min_mps_ = std::max(0.0, velocity_min_mps_);
    velocity_max_mps_ = std::max(velocity_min_mps_, velocity_max_mps_);
    slow_reach_distance_m_ =
      std::max(stop_reach_distance_m_ + 0.05, slow_reach_distance_m_);
    terminal_status_period_sec_ = std::max(0.2, terminal_status_period_sec_);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_speed_mps_ = msg->twist.twist.linear.x;
    last_odom_receive_time_ = now();
    odom_received_ = true;
  }

  void enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = msg->data;
    if (!enabled_) {
      resetPlannerHistory();
      publishStop("DISABLED", "enable input is false", last_frame_id_);
    }
  }

  BeamData preprocessScan(const sensor_msgs::msg::LaserScan & scan) const
  {
    BeamData output;
    const size_t count = scan.ranges.size();
    output.ranges.assign(count, scan_range_cap_m_);
    output.observed.assign(count, 0);
    output.hit.assign(count, 0);
    if (count == 0 || scan.angle_increment <= 0.0) {
      return output;
    }

    const double usable_max = std::min(
      scan_range_cap_m_,
      std::isfinite(scan.range_max) && scan.range_max > 0.0 ?
      static_cast<double>(scan.range_max) : scan_range_cap_m_);
    size_t planning_beams = 0;
    size_t valid_beams = 0;

    for (size_t i = 0; i < count; ++i) {
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      if (angle < planning_angle_min_rad_ || angle > planning_angle_max_rad_) {
        continue;
      }
      ++planning_beams;
      const double raw = scan.ranges[i];
      const bool finite_hit =
        std::isfinite(raw) && raw >= scan.range_min && raw <= scan.range_max;
      const bool clear_to_max =
        std::isinf(raw) || (std::isfinite(raw) && raw > scan.range_max);

      if (finite_hit) {
        output.ranges[i] = std::min(raw, usable_max);
        output.observed[i] = 1;
        output.hit[i] = raw < usable_max ? 1 : 0;
        ++valid_beams;
      } else if (clear_to_max) {
        output.ranges[i] = usable_max;
        output.observed[i] = 1;
        ++valid_beams;
      }
    }

    output.valid_ratio = planning_beams > 0 ?
      static_cast<double>(valid_beams) / static_cast<double>(planning_beams) : 0.0;

    if (median_filter_window_ > 1) {
      const std::vector<double> original = output.ranges;
      const int radius = median_filter_window_ / 2;
      for (size_t i = 0; i < count; ++i) {
        if (!output.observed[i]) {
          continue;
        }
        std::vector<double> window;
        for (int offset = -radius; offset <= radius; ++offset) {
          const int index = static_cast<int>(i) + offset;
          if (index >= 0 && index < static_cast<int>(count) &&
            output.observed[static_cast<size_t>(index)])
          {
            window.push_back(original[static_cast<size_t>(index)]);
          }
        }
        if (!window.empty()) {
          std::sort(window.begin(), window.end());
          // Smooth isolated long-range spikes, but never erase a closer return.
          output.ranges[i] = std::min(
            original[i], window[window.size() / 2]);
        }
      }
    }

    output.obstacle_points.reserve(count);
    for (size_t i = 0; i < count; ++i) {
      if (!output.observed[i] || !output.hit[i]) {
        continue;
      }
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      if (angle < planning_angle_min_rad_ || angle > planning_angle_max_rad_) {
        continue;
      }
      const double range = std::max(
        0.0, output.ranges[i] - obstacle_endpoint_margin_m_);
      output.obstacle_points.push_back(
        Point2{range * std::cos(angle), range * std::sin(angle)});
    }
    return output;
  }

  bool observedFree(
    const double x, const double y, const sensor_msgs::msg::LaserScan & scan,
    const BeamData & beam_data) const
  {
    const double angle = std::atan2(y, x);
    if (angle < planning_angle_min_rad_ || angle > planning_angle_max_rad_) {
      return false;
    }
    const double index_f = (angle - scan.angle_min) / scan.angle_increment;
    const int index = static_cast<int>(std::lround(index_f));
    if (index < 0 || index >= static_cast<int>(beam_data.ranges.size()) ||
      !beam_data.observed[static_cast<size_t>(index)])
    {
      return false;
    }

    const double radial_distance = std::hypot(x, y);
    if (radial_distance + obstacle_endpoint_margin_m_ >
      beam_data.ranges[static_cast<size_t>(index)])
    {
      return false;
    }

    const double envelope_squared = envelope_radius_ * envelope_radius_;
    for (const auto & obstacle : beam_data.obstacle_points) {
      if (std::abs(obstacle.x - x) > envelope_radius_ ||
        std::abs(obstacle.y - y) > envelope_radius_)
      {
        continue;
      }
      const double dx = obstacle.x - x;
      const double dy = obstacle.y - y;
      if (dx * dx + dy * dy < envelope_squared) {
        return false;
      }
    }
    return true;
  }

  std::vector<Interval> buildSliceIntervals(
    const double x, const sensor_msgs::msg::LaserScan & scan,
    const BeamData & beam_data) const
  {
    std::vector<Interval> intervals;
    bool inside = false;
    double start_y = 0.0;
    double previous_y = -lateral_limit_m_;

    const int samples = static_cast<int>(
      std::floor(2.0 * lateral_limit_m_ / lateral_sample_step_m_)) + 1;
    for (int sample = 0; sample < samples; ++sample) {
      const double y = -lateral_limit_m_ +
        static_cast<double>(sample) * lateral_sample_step_m_;
      const bool usable = observedFree(x, y, scan, beam_data);
      if (usable && !inside) {
        inside = true;
        start_y = y;
      } else if (!usable && inside) {
        const double end_y = previous_y;
        if (end_y - start_y >= min_interval_width_m_) {
          intervals.push_back(Interval{x, start_y, end_y});
        }
        inside = false;
      }
      previous_y = y;
    }
    if (inside && previous_y - start_y >= min_interval_width_m_) {
      intervals.push_back(Interval{x, start_y, previous_y});
    }
    return intervals;
  }

  bool intervalsConnect(const Interval & previous, const Interval & current) const
  {
    return current.low <= previous.high + max_interval_shift_per_slice_m_ &&
           current.high >= previous.low - max_interval_shift_per_slice_m_;
  }

  static bool betterPredecessor(const Branch & lhs, const Branch & rhs)
  {
    if (std::abs(lhs.min_width - rhs.min_width) > 1e-6) {
      return lhs.min_width > rhs.min_width;
    }
    return lhs.lateral_motion < rhs.lateral_motion;
  }

  std::vector<Branch> constructBranches(
    const sensor_msgs::msg::LaserScan & scan, const BeamData & beam_data,
    double & furthest_detected_reach, size_t & raw_branch_count) const
  {
    std::vector<Branch> active;
    std::vector<Branch> completed;

    bool first_slice = true;
    for (double x = forward_start_m_; x <= forward_max_m_ + 1e-6;
      x += forward_slice_step_m_)
    {
      const auto intervals = buildSliceIntervals(x, scan, beam_data);
      if (first_slice) {
        for (const auto & interval : intervals) {
          const double distance_to_center =
            0.0 < interval.low ? interval.low :
            (0.0 > interval.high ? -interval.high : 0.0);
          if (distance_to_center <= max_interval_shift_per_slice_m_) {
            Branch branch;
            branch.intervals.push_back(interval);
            branch.min_width = interval.width();
            branch.lateral_motion = std::abs(interval.center());
            active.push_back(branch);
          }
        }
        first_slice = false;
        if (active.empty()) {
          break;
        }
        continue;
      }

      std::vector<Branch> next;
      std::vector<uint8_t> predecessor_used(active.size(), 0);
      for (const auto & interval : intervals) {
        bool found = false;
        Branch best;
        size_t best_index = 0;
        for (size_t branch_index = 0; branch_index < active.size(); ++branch_index) {
          if (!intervalsConnect(active[branch_index].intervals.back(), interval)) {
            continue;
          }
          Branch candidate = active[branch_index];
          candidate.lateral_motion +=
            std::abs(interval.center() - candidate.intervals.back().center());
          candidate.intervals.push_back(interval);
          candidate.min_width = std::min(candidate.min_width, interval.width());
          if (!found || betterPredecessor(candidate, best)) {
            found = true;
            best = std::move(candidate);
            best_index = branch_index;
          }
        }
        if (found) {
          predecessor_used[best_index] = 1;
          next.push_back(std::move(best));
        }
      }

      for (size_t i = 0; i < active.size(); ++i) {
        if (!predecessor_used[i]) {
          completed.push_back(std::move(active[i]));
        }
      }
      active = std::move(next);
      if (active.empty()) {
        break;
      }
    }

    for (auto & branch : active) {
      completed.push_back(std::move(branch));
    }

    furthest_detected_reach = 0.0;
    raw_branch_count = completed.size();
    for (const auto & branch : completed) {
      furthest_detected_reach =
        std::max(furthest_detected_reach, branch.reach());
    }

    completed.erase(
      std::remove_if(
        completed.begin(), completed.end(),
        [this](const Branch & branch) {
          return branch.reach() < min_path_reach_m_ || branch.intervals.size() < 3;
        }),
      completed.end());
    return completed;
  }

  int branchSide(const Branch & branch) const
  {
    if (branch.intervals.empty()) {
      return 0;
    }
    double sum = 0.0;
    size_t count = 0;
    for (const auto & interval : branch.intervals) {
      if (interval.x >= std::min(1.0, branch.reach())) {
        sum += interval.center();
        ++count;
      }
    }
    const double mean = count > 0 ? sum / static_cast<double>(count) :
      branch.intervals.back().center();
    if (mean > side_deadband_m_) {
      return 1;
    }
    if (mean < -side_deadband_m_) {
      return -1;
    }
    return 0;
  }

  bool branchBetter(const Branch & candidate, const Branch & reference) const
  {
    if (candidate.reach() > reference.reach() + reach_tie_tolerance_m_) {
      return true;
    }
    if (reference.reach() > candidate.reach() + reach_tie_tolerance_m_) {
      return false;
    }
    if (candidate.min_width > reference.min_width + width_tie_tolerance_m_) {
      return true;
    }
    if (reference.min_width > candidate.min_width + width_tie_tolerance_m_) {
      return false;
    }
    return candidate.lateral_motion < reference.lateral_motion;
  }

  size_t chooseBranch(const std::vector<Branch> & branches)
  {
    size_t best_index = 0;
    for (size_t i = 1; i < branches.size(); ++i) {
      if (branchBetter(branches[i], branches[best_index])) {
        best_index = i;
      }
    }

    if (current_side_ == 0) {
      current_side_ = branchSide(branches[best_index]);
      pending_side_ = current_side_;
      pending_side_cycles_ = 0;
      return best_index;
    }

    size_t persistent_index = branches.size();
    for (size_t i = 0; i < branches.size(); ++i) {
      if (branchSide(branches[i]) != current_side_) {
        continue;
      }
      if (persistent_index == branches.size() ||
        branchBetter(branches[i], branches[persistent_index]))
      {
        persistent_index = i;
      }
    }

    if (persistent_index == branches.size()) {
      current_side_ = branchSide(branches[best_index]);
      pending_side_ = current_side_;
      pending_side_cycles_ = 0;
      return best_index;
    }

    const int best_side = branchSide(branches[best_index]);
    const bool clearly_better =
      branches[best_index].reach() >=
      branches[persistent_index].reach() + switch_reach_advantage_m_;
    if (best_side == current_side_ || !clearly_better) {
      pending_side_ = current_side_;
      pending_side_cycles_ = 0;
      return persistent_index;
    }

    if (pending_side_ == best_side) {
      ++pending_side_cycles_;
    } else {
      pending_side_ = best_side;
      pending_side_cycles_ = 1;
    }
    if (pending_side_cycles_ >= switch_confirmation_cycles_) {
      current_side_ = best_side;
      pending_side_cycles_ = 0;
      return best_index;
    }
    return persistent_index;
  }

  double previousPathYAt(const double x) const
  {
    if (previous_path_.empty()) {
      return 0.0;
    }
    auto iterator = std::lower_bound(
      previous_path_.begin(), previous_path_.end(), x,
      [](const Point2 & point, const double query_x) {return point.x < query_x;});
    if (iterator == previous_path_.begin()) {
      return iterator->y;
    }
    if (iterator == previous_path_.end()) {
      return previous_path_.back().y;
    }
    const auto & right = *iterator;
    const auto & left = *(iterator - 1);
    const double denominator = right.x - left.x;
    const double ratio = denominator > 1e-9 ? (x - left.x) / denominator : 0.0;
    return left.y + ratio * (right.y - left.y);
  }

  std::vector<Point2> generatePath(
    const Branch & branch, std::vector<Point2> & raw_path)
  {
    raw_path.clear();
    raw_path.reserve(branch.intervals.size() + 1);
    raw_path.push_back(Point2{0.0, 0.0});
    for (const auto & interval : branch.intervals) {
      raw_path.push_back(Point2{interval.x, interval.center()});
    }
    std::vector<Point2> path = raw_path;

    if (!previous_path_.empty()) {
      for (size_t i = 1; i < path.size(); ++i) {
        const auto & interval = branch.intervals[i - 1];
        const double blended =
          temporal_smoothing_alpha_ * path[i].y +
          (1.0 - temporal_smoothing_alpha_) * previousPathYAt(path[i].x);
        path[i].y = clampValue(blended, interval.low, interval.high);
      }
    }

    const size_t anchor_index = std::min(
      static_cast<size_t>(std::max(0, path_start_anchor_points_)),
      path.size() - 1);
    for (int pass = 0; pass < spatial_smoothing_passes_; ++pass) {
      std::vector<Point2> smoothed = path;
      for (size_t i = 1; i + 1 < path.size(); ++i) {
        if (i <= anchor_index && anchor_index > 0) {
          const double anchor_ratio =
            path[i].x / std::max(1e-6, path[anchor_index].x);
          const double anchored = anchor_ratio * path[anchor_index].y;
          const auto & interval = branch.intervals[i - 1];
          smoothed[i].y = clampValue(anchored, interval.low, interval.high);
          continue;
        }
        const double neighbor_mean = 0.5 * (path[i - 1].y + path[i + 1].y);
        const double candidate =
          (1.0 - spatial_smoothing_weight_) * path[i].y +
          spatial_smoothing_weight_ * neighbor_mean;
        const auto & interval = branch.intervals[i - 1];
        smoothed[i].y = clampValue(candidate, interval.low, interval.high);
      }
      path = std::move(smoothed);
    }
    path.front() = Point2{0.0, 0.0};
    return path;
  }

  bool pathIsValid(
    const std::vector<Point2> & path,
    const sensor_msgs::msg::LaserScan & scan,
    const BeamData & beam_data,
    const std::string & path_source,
    ValidationFailure & failure) const
  {
    failure = ValidationFailure{};
    failure.path_source = path_source;
    if (path.size() < 2) {
      failure.failed = true;
      failure.check_code = "PATH_TOO_SHORT";
      return false;
    }
    const double validation_step =
      std::max(0.02, 0.5 * std::min(forward_slice_step_m_, lateral_sample_step_m_));
    for (size_t i = 0; i + 1 < path.size(); ++i) {
      const double dx = path[i + 1].x - path[i].x;
      const double dy = path[i + 1].y - path[i].y;
      const double length = std::hypot(dx, dy);
      const int samples = std::max(1, static_cast<int>(std::ceil(length / validation_step)));
      for (int sample = 1; sample <= samples; ++sample) {
        const double ratio =
          static_cast<double>(sample) / static_cast<double>(samples);
        const double x = path[i].x + ratio * dx;
        const double y = path[i].y + ratio * dy;
        if (!observedFree(x, y, scan, beam_data)) {
          failure.failed = true;
          failure.segment_index = i;
          failure.segment_count = path.size() - 1;
          failure.sample_index = sample;
          failure.samples_in_segment = samples;
          failure.segment_start = path[i];
          failure.segment_end = path[i + 1];
          failure.point = Point2{x, y};
          failure.segment_length = length;
          failure.segment_heading_deg = std::atan2(dy, dx) * 180.0 / kPi;
          diagnoseValidationFailure(x, y, scan, beam_data, failure);
          return false;
        }
      }
    }
    return true;
  }

  void diagnoseValidationFailure(
    const double x, const double y,
    const sensor_msgs::msg::LaserScan & scan,
    const BeamData & beam_data,
    ValidationFailure & failure) const
  {
    const double angle = std::atan2(y, x);
    failure.beam_angle_deg = angle * 180.0 / kPi;
    failure.point_range_m = std::hypot(x, y);
    failure.required_clearance_m = envelope_radius_;

    if (angle < planning_angle_min_rad_ || angle > planning_angle_max_rad_) {
      failure.check_code = "OUTSIDE_PLANNING_ANGLE";
      return;
    }

    const double index_f = (angle - scan.angle_min) / scan.angle_increment;
    const int index = static_cast<int>(std::lround(index_f));
    failure.beam_index = index;
    if (index < 0 || index >= static_cast<int>(beam_data.ranges.size())) {
      failure.check_code = "BEAM_INDEX_OUT_OF_RANGE";
      return;
    }
    if (!beam_data.observed[static_cast<size_t>(index)]) {
      failure.check_code = "BEAM_UNOBSERVED";
      return;
    }

    failure.observed_range_m = beam_data.ranges[static_cast<size_t>(index)];
    failure.radial_clearance_m =
      failure.observed_range_m - failure.point_range_m -
      obstacle_endpoint_margin_m_;
    if (failure.point_range_m + obstacle_endpoint_margin_m_ >
      failure.observed_range_m)
    {
      failure.check_code = "BEYOND_OBSERVED_RANGE";
      return;
    }

    double nearest_distance = std::numeric_limits<double>::infinity();
    for (const auto & obstacle : beam_data.obstacle_points) {
      nearest_distance = std::min(
        nearest_distance, std::hypot(obstacle.x - x, obstacle.y - y));
    }
    failure.nearest_obstacle_distance_m = nearest_distance;
    if (nearest_distance < envelope_radius_) {
      failure.check_code = "OBSTACLE_ENVELOPE_COLLISION";
      return;
    }

    // This should be unreachable because the caller invokes this helper only
    // after observedFree() rejects the same point. Keep an explicit code so a
    // future mismatch between the two checks is visible.
    failure.check_code = "UNKNOWN_FREE_SPACE_REJECTION";
  }

  bool findLookahead(const std::vector<Point2> & path, Point2 & target) const
  {
    for (const auto & point : path) {
      if (std::hypot(point.x, point.y) >= lookahead_distance_m_) {
        target = point;
        return true;
      }
    }
    return false;
  }

  PlanResult makePlan(
    const sensor_msgs::msg::LaserScan & scan, const BeamData & beam_data)
  {
    PlanResult result;
    result.scan_valid_ratio = beam_data.valid_ratio;
    if (beam_data.valid_ratio < min_valid_beam_ratio_) {
      result.state = "INPUT_INVALID";
      result.reason = "insufficient valid forward laser beams";
      return result;
    }

    size_t raw_branch_count = 0;
    const auto branches = constructBranches(
      scan, beam_data, result.furthest_detected_reach, raw_branch_count);
    result.detected_branches = raw_branch_count;
    result.usable_branches = branches.size();
    if (branches.empty()) {
      result.state = "BLOCKED";
      result.reason = raw_branch_count == 0 ?
        "no connected free corridor starts in front of the vehicle" :
        "detected corridors are shorter than min_path_reach_m";
      return result;
    }

    result.branch = branches[chooseBranch(branches)];
    std::vector<Point2> raw_path;
    result.path = generatePath(result.branch, raw_path);
    ValidationFailure smoothed_failure;
    if (!pathIsValid(
        result.path, scan, beam_data, "smoothed", smoothed_failure))
    {
      result.smoothed_failure = smoothed_failure;
      ValidationFailure raw_failure;
      if (pathIsValid(raw_path, scan, beam_data, "raw", raw_failure)) {
        result.path = std::move(raw_path);
        result.used_raw_fallback = true;
      } else {
        result.raw_failure = raw_failure;
        result.state = "PATH_INVALID";
        result.reason =
          "both smoothed and raw corridor midlines fail swept-path validation";
        return result;
      }
    }
    previous_path_ = result.path;
    if (!findLookahead(result.path, result.lookahead)) {
      result.state = "BLOCKED";
      result.reason = "selected corridor is shorter than pure-pursuit lookahead";
      return result;
    }

    const double target_distance =
      std::hypot(result.lookahead.x, result.lookahead.y);
    const double curvature =
      2.0 * result.lookahead.y / (target_distance * target_distance);
    const double raw_steering = clampValue(
      std::atan(wheelbase_m_ * curvature), -steering_max_rad_, steering_max_rad_);
    result.steering =
      steering_filter_alpha_ * raw_steering +
      (1.0 - steering_filter_alpha_) * previous_steering_;
    previous_steering_ = result.steering;

    const double steering_ratio = clampValue(
      std::abs(result.steering) / std::max(1e-6, steering_max_rad_), 0.0, 1.0);
    const double steering_speed =
      velocity_max_mps_ -
      steering_ratio * (velocity_max_mps_ - velocity_min_mps_);
    const double reach_factor = clampValue(
      (result.branch.reach() - stop_reach_distance_m_) /
      (slow_reach_distance_m_ - stop_reach_distance_m_), 0.0, 1.0);
    result.speed = steering_speed * reach_factor;
    if (result.branch.reach() <= stop_reach_distance_m_) {
      result.speed = 0.0;
    }

    result.valid = result.speed > 0.0;
    result.state = result.valid ? "DRIVING" : "BLOCKED";
    result.reason = result.valid ?
      (result.used_raw_fallback ?
      "raw midline fallback after smoothed path validation failed" :
      "valid connected corridor") :
      "insufficient corridor reach";
    return result;
  }

  std::string validationFailureText(const ValidationFailure & failure) const
  {
    if (!failure.failed) {
      return "none";
    }
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2)
           << failure.path_source
           << "{check=" << failure.check_code
           << ", segment=" << (failure.segment_index + 1)
           << "/" << failure.segment_count
           << ", sample=" << failure.sample_index
           << "/" << failure.samples_in_segment
           << ", from=(" << failure.segment_start.x
           << "," << failure.segment_start.y << ")"
           << ", to=(" << failure.segment_end.x
           << "," << failure.segment_end.y << ")"
           << ", fail=(" << failure.point.x
           << "," << failure.point.y << ")"
           << ", length=" << failure.segment_length << " m"
           << ", heading=" << failure.segment_heading_deg << " deg";

    if (failure.check_code == "OBSTACLE_ENVELOPE_COLLISION") {
      stream << ", clearance=" << failure.nearest_obstacle_distance_m
             << "<" << failure.required_clearance_m << " m";
    } else if (failure.check_code == "BEYOND_OBSERVED_RANGE") {
      stream << ", point_range=" << failure.point_range_m
             << " m, observed_range=" << failure.observed_range_m
             << " m, endpoint_margin=" << obstacle_endpoint_margin_m_ << " m";
    } else if (
      failure.check_code == "BEAM_UNOBSERVED" ||
      failure.check_code == "BEAM_INDEX_OUT_OF_RANGE")
    {
      stream << ", beam=" << failure.beam_index
             << ", angle=" << failure.beam_angle_deg << " deg";
    } else if (failure.check_code == "OUTSIDE_PLANNING_ANGLE") {
      stream << ", angle=" << failure.beam_angle_deg
             << " deg, allowed=[" << planning_angle_min_rad_ * 180.0 / kPi
             << "," << planning_angle_max_rad_ * 180.0 / kPi << "] deg";
    }
    stream << "}";
    return stream.str();
  }

  std::string sideName() const
  {
    if (current_side_ > 0) {
      return "LEFT";
    }
    if (current_side_ < 0) {
      return "RIGHT";
    }
    return "CENTER";
  }

  void logTerminalStatus(const PlanResult & result)
  {
    const auto wall_now = std::chrono::steady_clock::now();
    const double elapsed_sec =
      std::chrono::duration<double>(wall_now - last_terminal_log_time_).count();
    const bool state_changed =
      result.state != last_terminal_state_ || result.reason != last_terminal_reason_;
    if (!state_changed && elapsed_sec < terminal_status_period_sec_) {
      return;
    }

    const double reported_reach = result.branch.intervals.empty() ?
      result.furthest_detected_reach : result.branch.reach();
    const double reported_width = std::isfinite(result.branch.min_width) ?
      result.branch.min_width : 0.0;
    const double steering_deg = result.steering * 180.0 / kPi;
    const char * motion_note =
      odom_received_ && result.speed > 0.20 && std::abs(current_speed_mps_) < 0.05 ?
      " | NOTE: forward command but odom nearly stationary" : "";

    if (result.valid && result.used_raw_fallback) {
      RCLCPP_WARN(
        get_logger(),
        "state=%s | path_source=RAW_FALLBACK; smoothed path rejected: %s | "
        "reach=%.2f m, side=%s | cmd=%.2f m/s, steer=%+.1f deg, odom=%.2f m/s",
        result.state.c_str(),
        validationFailureText(result.smoothed_failure).c_str(),
        reported_reach, sideName().c_str(), result.speed, steering_deg,
        current_speed_mps_);
    } else if (result.valid) {
      RCLCPP_INFO(
        get_logger(),
        "state=%s | path_source=SMOOTHED, reach=%.2f m, min_width=%.2f m, side=%s, "
        "scan_valid=%.2f | cmd=%.2f m/s, steer=%+.1f deg, odom=%.2f m/s%s",
        result.state.c_str(), reported_reach, reported_width, sideName().c_str(),
        result.scan_valid_ratio, result.speed, steering_deg, current_speed_mps_,
        motion_note);
    } else if (result.state == "PATH_INVALID") {
      RCLCPP_WARN(
        get_logger(),
        "state=PATH_INVALID -> STOP | both candidate paths failed | %s | %s | "
        "corridor_reach=%.2f m, min_width=%.2f m, "
        "usable/detected_branches=%zu/%zu, "
        "scan_valid=%.2f, envelope=%.2f m/side, odom=%.2f m/s",
        validationFailureText(result.smoothed_failure).c_str(),
        validationFailureText(result.raw_failure).c_str(),
        reported_reach, reported_width, result.usable_branches,
        result.detected_branches, result.scan_valid_ratio, envelope_radius_,
        current_speed_mps_);
    } else if (result.state == "BLOCKED") {
      RCLCPP_WARN(
        get_logger(),
        "state=%s -> STOP | reason=%s | detected_reach=%.2f m "
        "(required %.2f m), detected_branches=%zu, usable_branches=%zu, "
        "scan_valid=%.2f, envelope=%.2f m/side, odom=%.2f m/s",
        result.state.c_str(), result.reason.c_str(), reported_reach,
        min_path_reach_m_, result.detected_branches, result.usable_branches,
        result.scan_valid_ratio, envelope_radius_, current_speed_mps_);
    } else if (result.state == "INPUT_INVALID") {
      RCLCPP_WARN(
        get_logger(),
        "state=%s -> STOP | reason=%s | scan_valid=%.2f (required %.2f)",
        result.state.c_str(), result.reason.c_str(), result.scan_valid_ratio,
        min_valid_beam_ratio_);
    } else {
      const double scan_age = scan_received_ ?
        (now() - last_scan_receive_time_).seconds() : -1.0;
      const double odom_age = odom_received_ ?
        (now() - last_odom_receive_time_).seconds() : -1.0;
      RCLCPP_WARN(
        get_logger(),
        "state=%s -> STOP | reason=%s | scan_age=%.2f s, odom_age=%.2f s, "
        "odom=%.2f m/s",
        result.state.c_str(), result.reason.c_str(), scan_age, odom_age,
        current_speed_mps_);
    }

    last_terminal_state_ = result.state;
    last_terminal_reason_ = result.reason;
    last_terminal_log_time_ = wall_now;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    scan_received_ = true;
    last_scan_receive_time_ = now();
    last_frame_id_ = scan->header.frame_id;

    if (!first_scan_logged_) {
      RCLCPP_INFO(
        get_logger(),
        "First LaserScan received: frame=%s, beams=%zu, angle_increment=%.6f rad",
        scan->header.frame_id.c_str(), scan->ranges.size(), scan->angle_increment);
      first_scan_logged_ = true;
    }

    if (!enabled_) {
      publishStop("DISABLED", "enable input is false", scan->header.frame_id);
      return;
    }
    if (require_fresh_odom_ &&
      (!odom_received_ || (now() - last_odom_receive_time_).seconds() > odom_timeout_sec_))
    {
      publishStop("ODOM_STALE", "fresh odometry is required", scan->header.frame_id);
      return;
    }
    if (scan->ranges.empty() || scan->angle_increment <= 0.0) {
      publishStop("INPUT_INVALID", "empty or malformed LaserScan", scan->header.frame_id);
      return;
    }

    const BeamData beam_data = preprocessScan(*scan);
    PlanResult result = makePlan(*scan, beam_data);
    publishResult(result, scan->header);
  }

  void watchdogCallback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double scan_age = (now() - last_scan_receive_time_).seconds();
    if (!scan_received_ || scan_age > scan_timeout_sec_) {
      resetPlannerHistory();
      publishStop(
        "WAITING_FOR_SCAN",
        scan_received_ ? "LaserScan timeout" : "no LaserScan received",
        last_frame_id_);
    }
  }

  void resetPlannerHistory()
  {
    previous_path_.clear();
    current_side_ = 0;
    pending_side_ = 0;
    pending_side_cycles_ = 0;
    previous_steering_ = 0.0;
  }

  void publishResult(
    const PlanResult & result, const std_msgs::msg::Header & source_header)
  {
    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header = source_header;
    command.drive.speed = result.valid ? result.speed : 0.0;
    command.drive.steering_angle = result.valid ? result.steering : 0.0;
    command_pub_->publish(command);

    publishPath(result.valid ? result.path : std::vector<Point2>{}, source_header);
    publishStatus(result, source_header.stamp);
    if (publish_visualization_) {
      publishMarkers(result, source_header);
    }

    if (!result.valid) {
      previous_steering_ = 0.0;
    }
    logTerminalStatus(result);
  }

  void publishStop(
    const std::string & state, const std::string & reason,
    const std::string & frame_id)
  {
    PlanResult stopped;
    stopped.state = state;
    stopped.reason = reason;
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = frame_id;
    publishResult(stopped, header);
  }

  void publishPath(
    const std::vector<Point2> & path, const std_msgs::msg::Header & header)
  {
    nav_msgs::msg::Path message;
    message.header = header;
    message.poses.reserve(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = header;
      pose.pose.position.x = path[i].x;
      pose.pose.position.y = path[i].y;
      pose.pose.position.z = 0.02;
      double yaw = 0.0;
      if (i + 1 < path.size()) {
        yaw = std::atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
      } else if (i > 0) {
        yaw = std::atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x);
      }
      pose.pose.orientation.z = std::sin(0.5 * yaw);
      pose.pose.orientation.w = std::cos(0.5 * yaw);
      message.poses.push_back(pose);
    }
    path_pub_->publish(message);
  }

  void publishStatus(
    const PlanResult & result, const builtin_interfaces::msg::Time & stamp)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "reactive_control_v2/corridor_planner";
    status.hardware_id = "lidar_corridor";
    status.message = result.state + ": " + result.reason;
    status.level = result.valid ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;

    auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue pair;
        pair.key = key;
        pair.value = value;
        status.values.push_back(pair);
      };
    add("state", result.state);
    add("reason", result.reason);
    add("corridor_reach_m", numberString(result.branch.reach()));
    add("furthest_detected_reach_m", numberString(result.furthest_detected_reach));
    add("detected_branches", std::to_string(result.detected_branches));
    add("usable_branches", std::to_string(result.usable_branches));
    add("scan_valid_ratio", numberString(result.scan_valid_ratio));
    add(
      "corridor_min_center_width_m",
      std::isfinite(result.branch.min_width) ? numberString(result.branch.min_width) : "0.000");
    add("steering_rad", numberString(result.steering));
    add("speed_mps", numberString(result.speed));
    add("current_odom_speed_mps", numberString(current_speed_mps_));
    add("selected_side", std::to_string(current_side_));
    add(
      "path_source",
      result.state == "PATH_INVALID" ? "invalid" :
      (result.used_raw_fallback ? "raw_fallback" : "smoothed"));
    add(
      "smoothed_validation_failed",
      result.smoothed_failure.failed ? "true" : "false");
    add("smoothed_failure_code", result.smoothed_failure.check_code);
    add(
      "smoothed_failure_segment",
      result.smoothed_failure.failed ?
      std::to_string(result.smoothed_failure.segment_index + 1) : "0");
    add("smoothed_failure_x_m", numberString(result.smoothed_failure.point.x));
    add("smoothed_failure_y_m", numberString(result.smoothed_failure.point.y));
    add(
      "smoothed_failure_clearance_m",
      std::isfinite(result.smoothed_failure.nearest_obstacle_distance_m) ?
      numberString(result.smoothed_failure.nearest_obstacle_distance_m) : "nan");
    add(
      "smoothed_required_clearance_m",
      numberString(result.smoothed_failure.required_clearance_m));
    add(
      "raw_validation_failed",
      result.raw_failure.failed ? "true" : "false");
    add("raw_failure_code", result.raw_failure.check_code);
    add(
      "raw_failure_segment",
      result.raw_failure.failed ?
      std::to_string(result.raw_failure.segment_index + 1) : "0");
    add("raw_failure_x_m", numberString(result.raw_failure.point.x));
    add("raw_failure_y_m", numberString(result.raw_failure.point.y));
    add(
      "raw_failure_clearance_m",
      std::isfinite(result.raw_failure.nearest_obstacle_distance_m) ?
      numberString(result.raw_failure.nearest_obstacle_distance_m) : "nan");
    add(
      "raw_required_clearance_m",
      numberString(result.raw_failure.required_clearance_m));
    array.status.push_back(status);
    status_pub_->publish(array);
  }

  geometry_msgs::msg::Point markerPoint(
    const double x, const double y, const double z = 0.0) const
  {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  }

  visualization_msgs::msg::Marker baseMarker(
    const std_msgs::msg::Header & header, const std::string & ns,
    const int id, const int type) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
    return marker;
  }

  void publishMarkers(
    const PlanResult & result, const std_msgs::msg::Header & header)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    if (!result.branch.intervals.empty()) {
      auto fill = baseMarker(
        header, "corridor_fill", 0, visualization_msgs::msg::Marker::TRIANGLE_LIST);
      // RViz validates scale for every marker type. TRIANGLE_LIST uses the
      // points directly, so an identity scale preserves the geometry and
      // avoids the misleading "Scale of 0" warning.
      fill.scale.x = 1.0;
      fill.scale.y = 1.0;
      fill.scale.z = 1.0;
      fill.color.r = 0.85F;
      fill.color.g = 0.30F;
      fill.color.b = 0.03F;
      fill.color.a = 0.16F;

      auto left = baseMarker(
        header, "corridor_left", 1, visualization_msgs::msg::Marker::LINE_STRIP);
      auto right = baseMarker(
        header, "corridor_right", 2, visualization_msgs::msg::Marker::LINE_STRIP);
      left.scale.x = 0.035;
      right.scale.x = 0.035;
      left.color.r = right.color.r = 0.85F;
      left.color.g = right.color.g = 0.30F;
      left.color.b = right.color.b = 0.03F;
      left.color.a = right.color.a = 1.0F;

      for (const auto & interval : result.branch.intervals) {
        left.points.push_back(markerPoint(interval.x, interval.high, 0.01));
        right.points.push_back(markerPoint(interval.x, interval.low, 0.01));
      }
      for (size_t i = 0; i + 1 < result.branch.intervals.size(); ++i) {
        const auto & a = result.branch.intervals[i];
        const auto & b = result.branch.intervals[i + 1];
        fill.points.push_back(markerPoint(a.x, a.low));
        fill.points.push_back(markerPoint(a.x, a.high));
        fill.points.push_back(markerPoint(b.x, b.high));
        fill.points.push_back(markerPoint(a.x, a.low));
        fill.points.push_back(markerPoint(b.x, b.high));
        fill.points.push_back(markerPoint(b.x, b.low));
      }
      array.markers.push_back(fill);
      array.markers.push_back(left);
      array.markers.push_back(right);
    }

    if (result.valid) {
      auto lookahead = baseMarker(
        header, "lookahead", 3, visualization_msgs::msg::Marker::SPHERE);
      lookahead.pose.position.x = result.lookahead.x;
      lookahead.pose.position.y = result.lookahead.y;
      lookahead.pose.position.z = 0.08;
      lookahead.scale.x = lookahead.scale.y = lookahead.scale.z = 0.14;
      lookahead.color.r = 1.0F;
      lookahead.color.g = 0.2F;
      lookahead.color.b = 0.1F;
      lookahead.color.a = 1.0F;

      auto steering = baseMarker(
        header, "steering", 4, visualization_msgs::msg::Marker::ARROW);
      steering.points.push_back(markerPoint(0.0, 0.0, 0.05));
      steering.points.push_back(
        markerPoint(std::cos(result.steering), std::sin(result.steering), 0.05));
      steering.scale.x = 0.05;
      steering.scale.y = 0.10;
      steering.scale.z = 0.10;
      steering.color.r = 0.2F;
      steering.color.g = 0.4F;
      steering.color.b = 1.0F;
      steering.color.a = 1.0F;
      array.markers.push_back(lookahead);
      array.markers.push_back(steering);
    }

    auto addFailureMarker =
      [this, &array, &header](
      const ValidationFailure & failure, const int id,
      const std::string & marker_namespace,
      const float red, const float green, const float blue)
      {
        if (!failure.failed) {
          return;
        }
        auto marker = baseMarker(
          header, marker_namespace, id,
          visualization_msgs::msg::Marker::SPHERE);
        marker.pose.position.x = failure.point.x;
        marker.pose.position.y = failure.point.y;
        marker.pose.position.z = 0.10;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.16;
        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = blue;
        marker.color.a = 1.0F;
        array.markers.push_back(marker);
      };
    addFailureMarker(
      result.smoothed_failure, 5, "validation_failure_smoothed",
      1.0F, 0.0F, 0.0F);
    addFailureMarker(
      result.raw_failure, 6, "validation_failure_raw",
      1.0F, 0.0F, 1.0F);
    marker_pub_->publish(array);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CorridorPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
