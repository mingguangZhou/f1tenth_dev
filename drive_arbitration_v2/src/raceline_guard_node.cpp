#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;

namespace
{
struct Point2
{
  double x{0.0};
  double y{0.0};
};

double pointToSegmentDistance(const Point2 & point, const Point2 & a, const Point2 & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length2 = dx * dx + dy * dy;
  if (length2 <= 1e-12) {
    return std::hypot(point.x - a.x, point.y - a.y);
  }
  const double projection = std::clamp(
    ((point.x - a.x) * dx + (point.y - a.y) * dy) / length2, 0.0, 1.0);
  const double closest_x = a.x + projection * dx;
  const double closest_y = a.y + projection * dy;
  return std::hypot(point.x - closest_x, point.y - closest_y);
}
}  // namespace

class RacelineGuardNode : public rclcpp::Node
{
public:
  RacelineGuardNode()
  : Node("raceline_guard"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("local_path_topic", "/path_following_v2/local_path");
    declare_parameter<std::string>(
      "status_topic", "/drive_arbitration_v2/raceline_guard_status");
    declare_parameter<double>("control_rate_hz", 20.0);
    declare_parameter<double>("scan_timeout_sec", 0.30);
    declare_parameter<double>("path_timeout_sec", 0.30);
    declare_parameter<double>("transform_timeout_sec", 0.05);
    declare_parameter<double>("scan_range_cap_m", 6.0);
    declare_parameter<double>("guard_distance_m", 4.0);
    declare_parameter<double>("guard_path_step_m", 0.10);
    declare_parameter<double>("vehicle_width_m", 0.28);
    declare_parameter<double>("lateral_safety_margin_m", 0.10);
    declare_parameter<double>("min_valid_beam_ratio", 0.35);
    declare_parameter<int>("blocked_min_points", 3);
    declare_parameter<int>("blocked_confirmation_scans", 2);
    declare_parameter<int>("clear_confirmation_scans", 3);
    declare_parameter<double>("critical_block_distance_m", 0.80);

    scan_topic_ = get_parameter("scan_topic").as_string();
    local_path_topic_ = get_parameter("local_path_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    control_rate_hz_ = std::max(1.0, get_parameter("control_rate_hz").as_double());
    scan_timeout_sec_ = std::max(0.01, get_parameter("scan_timeout_sec").as_double());
    path_timeout_sec_ = std::max(0.01, get_parameter("path_timeout_sec").as_double());
    transform_timeout_sec_ = std::max(
      0.0, get_parameter("transform_timeout_sec").as_double());
    scan_range_cap_m_ = std::max(0.10, get_parameter("scan_range_cap_m").as_double());
    guard_distance_m_ = std::max(0.10, get_parameter("guard_distance_m").as_double());
    guard_path_step_m_ = std::max(0.01, get_parameter("guard_path_step_m").as_double());
    vehicle_width_m_ = std::max(0.01, get_parameter("vehicle_width_m").as_double());
    lateral_safety_margin_m_ = std::max(
      0.0, get_parameter("lateral_safety_margin_m").as_double());
    min_valid_beam_ratio_ = std::clamp(
      get_parameter("min_valid_beam_ratio").as_double(), 0.0, 1.0);
    blocked_min_points_ = std::max(
      1, static_cast<int>(get_parameter("blocked_min_points").as_int()));
    blocked_confirmation_scans_ = std::max(
      1, static_cast<int>(get_parameter("blocked_confirmation_scans").as_int()));
    clear_confirmation_scans_ = std::max(
      1, static_cast<int>(get_parameter("clear_confirmation_scans").as_int()));
    critical_block_distance_m_ = std::max(
      0.0, get_parameter("critical_block_distance_m").as_double());
    guard_half_width_m_ = 0.5 * vehicle_width_m_ + lateral_safety_margin_m_;

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      local_path_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&RacelineGuardNode::pathCallback, this, _1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RacelineGuardNode::scanCallback, this, _1));
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(status_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RacelineGuardNode::controlLoop, this));

    RCLCPP_INFO(
      get_logger(),
      "raceline_guard ready: path=%s scan=%s distance=%.2f m half_width=%.2f m",
      local_path_topic_.c_str(), scan_topic_.c_str(), guard_distance_m_, guard_half_width_m_);
  }

private:
  enum class State {UNKNOWN, CLEAR, BLOCKED};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  std::mutex mutex_;
  nav_msgs::msg::Path::SharedPtr latest_path_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  rclcpp::Time last_path_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_scan_time_{0, 0, RCL_STEADY_TIME};
  bool path_received_{false};
  bool scan_received_{false};

  std::string scan_topic_;
  std::string local_path_topic_;
  std::string status_topic_;
  double control_rate_hz_{20.0};
  double scan_timeout_sec_{0.30};
  double path_timeout_sec_{0.30};
  double transform_timeout_sec_{0.05};
  double scan_range_cap_m_{6.0};
  double guard_distance_m_{4.0};
  double guard_path_step_m_{0.10};
  double vehicle_width_m_{0.28};
  double lateral_safety_margin_m_{0.10};
  double guard_half_width_m_{0.24};
  double min_valid_beam_ratio_{0.35};
  int blocked_min_points_{3};
  int blocked_confirmation_scans_{2};
  int clear_confirmation_scans_{3};
  double critical_block_distance_m_{0.80};

  State state_{State::UNKNOWN};
  bool blocked_latched_{false};
  int blocked_cycles_{0};
  int clear_cycles_{0};
  rclcpp::Time last_evaluated_scan_time_{0, 0, RCL_STEADY_TIME};
  int last_interfering_points_{0};
  double last_min_interference_range_m_{std::numeric_limits<double>::infinity()};
  double last_valid_beam_ratio_{0.0};
  double last_checked_reach_m_{0.0};
  std::string last_reason_{"waiting for path and scan"};

  void pathCallback(const nav_msgs::msg::Path::SharedPtr path)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_path_ = path;
    last_path_time_ = steady_clock_.now();
    path_received_ = true;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_scan_ = scan;
    last_scan_time_ = steady_clock_.now();
    scan_received_ = true;
  }

  const char * stateName(const State state) const
  {
    switch (state) {
      case State::UNKNOWN: return "UNKNOWN";
      case State::CLEAR: return "CLEAR";
      case State::BLOCKED: return "BLOCKED";
    }
    return "UNKNOWN";
  }

  bool transformPath(
    const nav_msgs::msg::Path & path, const std::string & target_frame,
    std::vector<Point2> & output, double & checked_reach)
  {
    output.clear();
    checked_reach = 0.0;
    if (path.poses.size() < 2 || path.header.frame_id.empty() || target_frame.empty()) {
      return false;
    }

    tf2::Transform target_from_path;
    try {
      const auto transform = tf_buffer_.lookupTransform(
        target_frame, path.header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_sec_));
      tf2::fromMsg(transform.transform, target_from_path);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 2000, "primary trajectory guard TF failed: %s", exception.what());
      return false;
    }

    Point2 previous_raw;
    bool have_previous_raw = false;
    double accumulated = 0.0;
    for (const auto & pose : path.poses) {
      const tf2::Vector3 transformed = target_from_path * tf2::Vector3(
        pose.pose.position.x, pose.pose.position.y, 0.0);
      Point2 point{transformed.x(), transformed.y()};
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        return false;
      }
      if (have_previous_raw) {
        accumulated += std::hypot(point.x - previous_raw.x, point.y - previous_raw.y);
      }
      previous_raw = point;
      have_previous_raw = true;

      if (output.empty() ||
        std::hypot(point.x - output.back().x, point.y - output.back().y) >= guard_path_step_m_ ||
        accumulated >= guard_distance_m_)
      {
        output.push_back(point);
      }
      if (accumulated >= guard_distance_m_) {
        break;
      }
    }
    if (output.size() == 1 && path.poses.size() >= 2) {
      const auto & pose = path.poses.back();
      const tf2::Vector3 transformed = target_from_path * tf2::Vector3(
        pose.pose.position.x, pose.pose.position.y, 0.0);
      output.push_back(Point2{transformed.x(), transformed.y()});
    }
    checked_reach = std::min(accumulated, guard_distance_m_);
    return output.size() >= 2;
  }

  void updateDebounce(const bool raw_blocked, const bool critical)
  {
    const State previous_state = state_;
    if (raw_blocked) {
      blocked_cycles_ = std::min(blocked_cycles_ + 1, blocked_confirmation_scans_);
      clear_cycles_ = 0;
      if (critical || blocked_cycles_ >= blocked_confirmation_scans_) {
        blocked_latched_ = true;
      }
    } else {
      blocked_cycles_ = 0;
      clear_cycles_ = std::min(clear_cycles_ + 1, clear_confirmation_scans_);
      if (clear_cycles_ >= clear_confirmation_scans_) {
        blocked_latched_ = false;
      }
    }

    if (blocked_latched_) {
      state_ = State::BLOCKED;
    } else if (!raw_blocked &&
      (previous_state == State::CLEAR || clear_cycles_ >= clear_confirmation_scans_))
    {
      // Keep confirmed CLEAR through one unconfirmed blocked scan.
      state_ = State::CLEAR;
    } else if (raw_blocked && previous_state == State::CLEAR) {
      // Preserve a previously confirmed CLEAR state while an ordinary single
      // blocked scan waits for its configured confirmation count.
      state_ = State::CLEAR;
    } else {
      state_ = State::UNKNOWN;
    }
  }

  void evaluateNewScan(
    const nav_msgs::msg::Path & path, const sensor_msgs::msg::LaserScan & scan)
  {
    if (scan.ranges.empty() || !std::isfinite(scan.angle_increment) ||
      scan.angle_increment <= 0.0 || scan.header.frame_id.empty())
    {
      state_ = State::UNKNOWN;
      last_reason_ = "empty or malformed LaserScan";
      return;
    }

    std::vector<Point2> guard_path;
    if (!transformPath(path, scan.header.frame_id, guard_path, last_checked_reach_m_)) {
      state_ = State::UNKNOWN;
      last_reason_ = "local path invalid or transform unavailable";
      return;
    }

    int valid_beams = 0;
    int interfering_points = 0;
    double minimum_interference_range = std::numeric_limits<double>::infinity();
    const double configured_min = std::isfinite(scan.range_min) ? scan.range_min : 0.0;
    const double sensor_max = std::isfinite(scan.range_max) && scan.range_max > 0.0 ?
      scan.range_max : scan_range_cap_m_;
    const double usable_max = std::min(scan_range_cap_m_, sensor_max);

    for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
      const double range = scan.ranges[index];
      if (!std::isnan(range) && range >= configured_min) {
        ++valid_beams;
      }
      if (!std::isfinite(range) || range < configured_min || range > usable_max) {
        continue;
      }
      const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
      const Point2 scan_point{range * std::cos(angle), range * std::sin(angle)};
      bool interferes = false;
      for (std::size_t segment = 0; segment + 1 < guard_path.size(); ++segment) {
        if (pointToSegmentDistance(
            scan_point, guard_path[segment], guard_path[segment + 1]) <= guard_half_width_m_)
        {
          interferes = true;
          break;
        }
      }
      if (interferes) {
        ++interfering_points;
        minimum_interference_range = std::min(minimum_interference_range, range);
      }
    }

    last_valid_beam_ratio_ = scan.ranges.empty() ? 0.0 :
      static_cast<double>(valid_beams) / static_cast<double>(scan.ranges.size());
    last_interfering_points_ = interfering_points;
    last_min_interference_range_m_ = minimum_interference_range;
    if (last_valid_beam_ratio_ < min_valid_beam_ratio_) {
      state_ = State::UNKNOWN;
      last_reason_ = "insufficient valid LaserScan beams";
      return;
    }

    const bool raw_blocked = interfering_points >= blocked_min_points_;
    const bool critical = raw_blocked &&
      minimum_interference_range <= critical_block_distance_m_;
    updateDebounce(raw_blocked, critical);
    if (state_ == State::BLOCKED) {
      last_reason_ = critical ? "critical close interference inside selected trajectory" :
        "interference inside selected trajectory confirmed";
    } else if (raw_blocked) {
      last_reason_ = "possible interference awaiting confirmation";
    } else if (state_ == State::CLEAR) {
      last_reason_ = "expanded selected primary trajectory is clear";
    } else {
      last_reason_ = "clear scan awaiting confirmation";
    }
  }

  void publishStatus(const double path_age, const double scan_age)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "drive_arbitration_v2/raceline_guard";
    status.hardware_id = "selected_primary_trajectory_scan_guard";
    status.level = state_ == State::CLEAR ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = std::string(stateName(state_)) + ": " + last_reason_;

    auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue pair;
        pair.key = key;
        pair.value = value;
        status.values.push_back(pair);
      };
    add("state", stateName(state_));
    add("reason", last_reason_);
    add("path_age_sec", std::to_string(path_age));
    add("scan_age_sec", std::to_string(scan_age));
    add("guard_distance_m", std::to_string(guard_distance_m_));
    add("checked_path_reach_m", std::to_string(last_checked_reach_m_));
    add("guard_half_width_m", std::to_string(guard_half_width_m_));
    add("interfering_points", std::to_string(last_interfering_points_));
    add(
      "min_interference_range_m",
      std::isfinite(last_min_interference_range_m_) ?
      std::to_string(last_min_interference_range_m_) : "inf");
    add("valid_beam_ratio", std::to_string(last_valid_beam_ratio_));
    add("blocked_confirmation_count", std::to_string(blocked_cycles_));
    add("clear_confirmation_count", std::to_string(clear_cycles_));
    array.status.push_back(status);
    status_pub_->publish(array);
  }

  void controlLoop()
  {
    nav_msgs::msg::Path::SharedPtr path;
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    rclcpp::Time path_time(0, 0, RCL_STEADY_TIME);
    rclcpp::Time scan_time(0, 0, RCL_STEADY_TIME);
    bool path_received = false;
    bool scan_received = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      path = latest_path_;
      scan = latest_scan_;
      path_time = last_path_time_;
      scan_time = last_scan_time_;
      path_received = path_received_;
      scan_received = scan_received_;
    }

    const auto current_time = steady_clock_.now();
    const double path_age = path_received ? (current_time - path_time).seconds() :
      std::numeric_limits<double>::infinity();
    const double scan_age = scan_received ? (current_time - scan_time).seconds() :
      std::numeric_limits<double>::infinity();
    if (!path_received || !path || path_age > path_timeout_sec_) {
      state_ = State::UNKNOWN;
      last_reason_ = path_received ? "local path heartbeat stale" : "no local path received";
      publishStatus(path_age, scan_age);
      return;
    }
    if (!scan_received || !scan || scan_age > scan_timeout_sec_) {
      state_ = State::UNKNOWN;
      last_reason_ = scan_received ? "LaserScan heartbeat stale" : "no LaserScan received";
      publishStatus(path_age, scan_age);
      return;
    }

    if (scan_time.nanoseconds() != last_evaluated_scan_time_.nanoseconds()) {
      last_evaluated_scan_time_ = scan_time;
      evaluateNewScan(*path, *scan);
    }
    publishStatus(path_age, scan_age);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RacelineGuardNode>());
  rclcpp::shutdown();
  return 0;
}
