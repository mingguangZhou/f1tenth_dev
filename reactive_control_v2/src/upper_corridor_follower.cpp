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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

// Small mathematical and formatting helpers are kept outside the ROS node because
// they do not depend on node state.
constexpr double kPi = 3.14159265358979323846;
constexpr char kPackageVersion[] = "0.2.9";

double degToRad(const double degrees)
{
  return degrees * kPi / 180.0;
}

double clampValue(const double value, const double low, const double high)
{
  return std::max(low, std::min(value, high));
}

std::string normalizeFrameId(std::string frame_id)
{
  while (!frame_id.empty() && frame_id.front() == '/') {
    frame_id.erase(frame_id.begin());
  }
  return frame_id;
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

// Planar rigid transform from the incoming LaserScan frame to base_frame.
// x/y describe the laser origin in base_frame; cos_yaw/sin_yaw describe its
// orientation. Keeping both directions here avoids repeating frame mathematics.
struct LaserToBaseTransform
{
  double x{0.0};
  double y{0.0};
  double cos_yaw{1.0};
  double sin_yaw{0.0};

  Point2 laserToBase(const Point2 & point) const
  {
    return Point2{
      x + cos_yaw * point.x - sin_yaw * point.y,
      y + sin_yaw * point.x + cos_yaw * point.y};
  }

  Point2 baseToLaser(const Point2 & point) const
  {
    const double dx = point.x - x;
    const double dy = point.y - y;
    return Point2{
      cos_yaw * dx + sin_yaw * dy,
      -sin_yaw * dx + cos_yaw * dy};
  }
};

struct Interval
{
  // One continuous lateral band of usable vehicle-centre positions at a fixed x.
  // Vehicle width and safety margin have already been applied when this is built.
  double x{0.0};
  double low{0.0};
  double high{0.0};

  double center() const {return 0.5 * (low + high);}
  double width() const {return high - low;}
};

struct Branch
{
  // A corridor is a sequence of connected free intervals from near to far.
  // min_width and lateral_motion are cached for later branch ranking.
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
  // Preprocessed scan indexed exactly like the original LaserScan.
  // observed distinguishes valid knowledge from missing data; hit distinguishes
  // a real obstacle return from a ray known to be clear up to its usable maximum.
  std::vector<double> ranges;
  std::vector<uint8_t> observed;
  std::vector<uint8_t> hit;
  std::vector<Point2> obstacle_points;
  double valid_ratio{0.0};
  LaserToBaseTransform laser_to_base;
};

struct ValidationFailure
{
  // Detailed evidence for the first rejected sample along one candidate path.
  // These fields feed terminal diagnostics, diagnostic_msgs and RViz markers.
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
  Point2 nearest_obstacle;
  bool nearest_obstacle_available{false};
  double required_clearance_m{0.0};
};

struct PlanResult
{
  // Complete output of one scan-to-command planning cycle.
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

class UpperCorridorFollower : public rclcpp::Node
{
public:
  UpperCorridorFollower()
  : Node("upper_corridor_follower")
  {
    // Parameters are declared first so launch/YAML overrides are available,
    // then copied into typed members and constrained to safe numeric ranges.
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

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // LaserScan drives planning. Odometry is used for freshness/status checking,
    // while the enable input allows an external supervisor to disable this layer.
    using std::placeholders::_1;
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&UpperCorridorFollower::scanCallback, this, _1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&UpperCorridorFollower::odomCallback, this, _1));
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_, 10, std::bind(&UpperCorridorFollower::enableCallback, this, _1));

    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&UpperCorridorFollower::watchdogCallback, this));

    // Without a mandatory enable message, the upper controller starts enabled.
    enabled_ = !require_enable_message_;
    last_scan_receive_time_ = now();
    last_odom_receive_time_ = now();
    last_frame_id_ = base_frame_;

    RCLCPP_INFO(
      get_logger(),
      "reactive_control_v2 v%s upper_corridor_follower ready: "
      "scan=%s, base_frame=%s, command=%s, "
      "envelope=%.2f m/side, swept_path_validation=%s (%d fail/%d recover), "
      "full_terminal_debug=%s",
      kPackageVersion, scan_topic_.c_str(), base_frame_.c_str(), command_topic_.c_str(),
      envelope_radius_, enable_swept_path_validation_ ? "true" : "false",
      swept_path_failure_confirmation_cycles_, swept_path_recovery_confirmation_cycles_,
      full_terminal_debug_ ? "true" : "false");
    if (full_terminal_debug_) {
      RCLCPP_INFO(
        get_logger(),
        "Full terminal diagnostics enabled: first result immediately, then every %.1f s.",
        terminal_status_period_sec_);
    }
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
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Topic and input parameters
  std::string scan_topic_;
  std::string odom_topic_;
  std::string enable_topic_;
  std::string command_topic_;
  std::string path_topic_;
  std::string marker_topic_;
  std::string status_topic_;
  std::string base_frame_;
  double transform_timeout_sec_{0.05};
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

  // Swept-path validation can be disabled for controlled debugging. Its
  // hysteresis applies only to the transition into/out of PATH_INVALID; other
  // planner failures and the lower controller's own safety checks are immediate.
  bool enable_swept_path_validation_{true};
  int swept_path_failure_confirmation_cycles_{2};
  int swept_path_recovery_confirmation_cycles_{3};

  // Debug parameters
  bool publish_visualization_{true};
  double marker_lifetime_sec_{0.25};
  double visualization_period_sec_{0.10};
  bool full_terminal_debug_{false};
  double terminal_status_period_sec_{2.0};

  // Runtime state
  // The callbacks and watchdog share these values, so every callback takes mutex_.
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
  int swept_path_failure_cycles_{0};
  int swept_path_recovery_cycles_{0};
  bool swept_path_failure_latched_{false};
  double previous_steering_{0.0};
  std::string last_frame_id_;
  std::string last_terminal_state_;
  std::string last_terminal_reason_;
  bool first_scan_logged_{false};
  bool visualization_published_{false};
  std::string last_visualization_state_;
  std::string last_visualization_reason_;
  std::chrono::steady_clock::time_point last_terminal_log_time_{
    std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_visualization_publish_time_{
    std::chrono::steady_clock::now()};

  void declareParameters()
  {
    // This block defines defaults only. Values supplied by YAML replace them
    // before readParameters() copies them into the member variables.
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
    declare_parameter<std::string>("enable_topic", "/reactive_control_v2/enable");
    declare_parameter<std::string>(
      "command_topic", "/reactive_control_v2/nominal_cmd");
    declare_parameter<std::string>("path_topic", "/reactive_control_v2/local_path");
    declare_parameter<std::string>("marker_topic", "/reactive_control_v2/markers");
    declare_parameter<std::string>("status_topic", "/reactive_control_v2/status");
    declare_parameter<std::string>("base_frame", "ego_racecar/base_link");
    declare_parameter<double>("transform_timeout_sec", 0.05);
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

    declare_parameter<bool>("enable_swept_path_validation", true);
    declare_parameter<int>("swept_path_failure_confirmation_cycles", 2);
    declare_parameter<int>("swept_path_recovery_confirmation_cycles", 3);

    declare_parameter<bool>("publish_visualization", true);
    declare_parameter<double>("marker_lifetime_sec", 0.25);
    declare_parameter<double>("visualization_period_sec", 0.10);
    declare_parameter<bool>("full_terminal_debug", false);
    declare_parameter<double>("terminal_status_period_sec", 2.0);
  }

  void readParameters()
  {
    // Normalize the configured frame name so "/base_link" and "base_link"
    // behave identically in TF lookups.
    scan_topic_ = get_parameter("scan_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    enable_topic_ = get_parameter("enable_topic").as_string();
    command_topic_ = get_parameter("command_topic").as_string();
    path_topic_ = get_parameter("path_topic").as_string();
    marker_topic_ = get_parameter("marker_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    base_frame_ = normalizeFrameId(get_parameter("base_frame").as_string());
    transform_timeout_sec_ = get_parameter("transform_timeout_sec").as_double();
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

    enable_swept_path_validation_ =
      get_parameter("enable_swept_path_validation").as_bool();
    swept_path_failure_confirmation_cycles_ =
      get_parameter("swept_path_failure_confirmation_cycles").as_int();
    swept_path_recovery_confirmation_cycles_ =
      get_parameter("swept_path_recovery_confirmation_cycles").as_int();

    publish_visualization_ = get_parameter("publish_visualization").as_bool();
    marker_lifetime_sec_ = get_parameter("marker_lifetime_sec").as_double();
    visualization_period_sec_ =
      get_parameter("visualization_period_sec").as_double();
    full_terminal_debug_ = get_parameter("full_terminal_debug").as_bool();
    terminal_status_period_sec_ =
      get_parameter("terminal_status_period_sec").as_double();
  }

  void validateParameters()
  {
    // Clamp settings that would otherwise cause invalid geometry, division by
    // zero, an even median-filter window, or values outside blending ranges.
    if (base_frame_.empty()) {
      base_frame_ = "base_link";
      RCLCPP_WARN(get_logger(), "Empty base_frame parameter; using base_link.");
    }
    transform_timeout_sec_ = std::max(0.0, transform_timeout_sec_);
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
    swept_path_failure_confirmation_cycles_ =
      std::max(1, swept_path_failure_confirmation_cycles_);
    swept_path_recovery_confirmation_cycles_ =
      std::max(1, swept_path_recovery_confirmation_cycles_);
    velocity_min_mps_ = std::max(0.0, velocity_min_mps_);
    velocity_max_mps_ = std::max(velocity_min_mps_, velocity_max_mps_);
    slow_reach_distance_m_ =
      std::max(stop_reach_distance_m_ + 0.05, slow_reach_distance_m_);
    visualization_period_sec_ = std::max(0.0, visualization_period_sec_);
    terminal_status_period_sec_ = std::max(0.2, terminal_status_period_sec_);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // The current implementation does not use odometry to build the path.
    // It stores forward speed for freshness checks and diagnostic reporting.
    current_speed_mps_ = msg->twist.twist.linear.x;
    last_odom_receive_time_ = now();
    odom_received_ = true;
  }

  void enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = msg->data;
    if (!enabled_) {
      // Do not carry steering, path smoothing or branch-persistence history
      // across a period in which the upper controller is disabled.
      resetPlannerHistory();
      publishStop("DISABLED", "enable input is false", last_frame_id_);
    }
  }

  bool lookupLaserToBaseTransform(
    const sensor_msgs::msg::LaserScan & scan,
    LaserToBaseTransform & output,
    std::string & error) const
  {
    // All planning and control geometry uses base_frame. The LaserScan may come
    // from a differently named and physically offset sensor frame.
    const std::string scan_frame = normalizeFrameId(scan.header.frame_id);
    if (scan_frame.empty()) {
      error = "LaserScan frame_id is empty";
      return false;
    }
    if (scan_frame == base_frame_) {
      // Identity transform: the scan is already expressed in the control frame.
      output = LaserToBaseTransform{};
      return true;
    }

    try {
      // Query TF at the scan timestamp so moving-frame data is not mixed across
      // different instants. The existing static laser transform also works here.
      const auto transform = tf_buffer_->lookupTransform(
        base_frame_, scan_frame, rclcpp::Time(scan.header.stamp),
        rclcpp::Duration::from_seconds(transform_timeout_sec_));
      const auto & translation = transform.transform.translation;
      const auto & rotation = transform.transform.rotation;
      const double sin_yaw =
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y);
      const double cos_yaw =
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z);
      const double yaw = std::atan2(sin_yaw, cos_yaw);
      output.x = translation.x;
      output.y = translation.y;
      output.cos_yaw = std::cos(yaw);
      output.sin_yaw = std::sin(yaw);
      return true;
    } catch (const tf2::TransformException & exception) {
      error = exception.what();
      return false;
    }
  }

  bool pointIsInStartRegion(
    const Point2 & point,
    const LaserToBaseTransform & laser_to_base) const
  {
    // A forward-mounted laser cannot directly observe the short region between
    // base_link and its own origin. Model that region as a capsule around the
    // connecting segment so valid start-path samples are not rejected merely
    // because no laser ray points backward into it.
    const double segment_length_squared =
      laser_to_base.x * laser_to_base.x + laser_to_base.y * laser_to_base.y;
    if (segment_length_squared < 1e-9) {
      return std::hypot(point.x, point.y) <= envelope_radius_;
    }
    const double projection = clampValue(
      (point.x * laser_to_base.x + point.y * laser_to_base.y) /
      segment_length_squared, 0.0, 1.0);
    const double nearest_x = projection * laser_to_base.x;
    const double nearest_y = projection * laser_to_base.y;
    // This function only grants observed-space coverage. obstacleClear() is
    // evaluated separately, so the start-region exception does not ignore hits.
    return std::hypot(point.x - nearest_x, point.y - nearest_y) <= envelope_radius_;
  }

  bool obstacleClear(
    const Point2 & point, const BeamData & beam_data) const
  {
    // Treat the planned point as the vehicle centre and reject it if any obstacle
    // endpoint lies inside the circular half-width-plus-margin envelope.
    const double envelope_squared = envelope_radius_ * envelope_radius_;
    for (const auto & obstacle : beam_data.obstacle_points) {
      // Cheap axis-aligned rejection avoids computing a squared distance for
      // obstacle points that cannot possibly be inside the circle.
      if (std::abs(obstacle.x - point.x) > envelope_radius_ ||
        std::abs(obstacle.y - point.y) > envelope_radius_)
      {
        continue;
      }
      const double dx = obstacle.x - point.x;
      const double dy = obstacle.y - point.y;
      if (dx * dx + dy * dy < envelope_squared) {
        return false;
      }
    }
    return true;
  }

  BeamData preprocessScan(
    const sensor_msgs::msg::LaserScan & scan,
    const LaserToBaseTransform & laser_to_base) const
  {
    // Stage 1: classify raw rays, apply conservative filtering, then convert
    // actual obstacle endpoints from the laser frame into base_frame.
    BeamData output;
    output.laser_to_base = laser_to_base;
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
      // A beam is included in scan-validity statistics only when its direction,
      // after rotation into base_frame, lies inside the configured planning sector.
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double direction_x =
        laser_to_base.cos_yaw * std::cos(angle) -
        laser_to_base.sin_yaw * std::sin(angle);
      const double direction_y =
        laser_to_base.sin_yaw * std::cos(angle) +
        laser_to_base.cos_yaw * std::sin(angle);
      const double base_direction_angle = std::atan2(direction_y, direction_x);
      const bool in_planning_sector =
        base_direction_angle >= planning_angle_min_rad_ &&
        base_direction_angle <= planning_angle_max_rad_;
      if (in_planning_sector) {
        ++planning_beams;
      }
      const double raw = scan.ranges[i];
      const bool finite_hit =
        std::isfinite(raw) && raw >= scan.range_min && raw <= scan.range_max;
      const bool clear_to_max =
        std::isinf(raw) || (std::isfinite(raw) && raw > scan.range_max);

      if (finite_hit) {
        // A finite measurement at usable_max is observed space but is not stored
        // as an obstacle endpoint, because it represents the planning-range cap.
        output.ranges[i] = std::min(raw, usable_max);
        output.observed[i] = 1;
        output.hit[i] = raw < usable_max ? 1 : 0;
        if (in_planning_sector) {
          ++valid_beams;
        }
      } else if (clear_to_max) {
        // Positive infinity is the normal LaserScan representation for a ray
        // with no return; it still proves free space up to usable_max.
        output.ranges[i] = usable_max;
        output.observed[i] = 1;
        if (in_planning_sector) {
          ++valid_beams;
        }
      }
    }

    output.valid_ratio = planning_beams > 0 ?
      static_cast<double>(valid_beams) / static_cast<double>(planning_beams) : 0.0;

    if (median_filter_window_ > 1) {
      // The one-sided median filter may shorten a suspiciously long ray but
      // deliberately cannot lengthen a close return and erase an obstacle.
      const std::vector<double> original = output.ranges;
      const int radius = median_filter_window_ / 2;
      std::vector<double> window;
      window.reserve(static_cast<size_t>(median_filter_window_));
      for (size_t i = 0; i < count; ++i) {
        if (!output.observed[i]) {
          continue;
        }
        window.clear();
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
      // Only finite hits become obstacle points used by the envelope check.
      if (!output.observed[i] || !output.hit[i]) {
        continue;
      }
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double direction_x =
        laser_to_base.cos_yaw * std::cos(angle) -
        laser_to_base.sin_yaw * std::sin(angle);
      const double direction_y =
        laser_to_base.sin_yaw * std::cos(angle) +
        laser_to_base.cos_yaw * std::sin(angle);
      const double base_direction_angle = std::atan2(direction_y, direction_x);
      if (base_direction_angle < planning_angle_min_rad_ ||
        base_direction_angle > planning_angle_max_rad_)
      {
        continue;
      }
      const double range = std::max(
        0.0, output.ranges[i] - obstacle_endpoint_margin_m_);
      // Pull the endpoint slightly toward the sensor before transforming it.
      // This intentionally makes the collision model more conservative.
      output.obstacle_points.push_back(laser_to_base.laserToBase(
          Point2{range * std::cos(angle), range * std::sin(angle)}));
    }
    return output;
  }

  bool observedFree(
    const double x, const double y, const sensor_msgs::msg::LaserScan & scan,
    const BeamData & beam_data) const
  {
    // A candidate vehicle-centre position is usable only if it is:
    //   1. inside the planning sector,
    //   2. outside every inflated obstacle envelope, and
    //   3. supported by an observed laser ray (or the protected start region).
    const Point2 point{x, y};
    const double base_angle = std::atan2(y, x);
    if (base_angle < planning_angle_min_rad_ || base_angle > planning_angle_max_rad_) {
      return false;
    }
    if (!obstacleClear(point, beam_data)) {
      return false;
    }

    const Point2 laser_point = beam_data.laser_to_base.baseToLaser(point);
    // Convert the candidate back to laser coordinates because LaserScan ranges
    // are indexed by angles measured in the laser frame.
    const double laser_angle = std::atan2(laser_point.y, laser_point.x);
    const double index_f = (laser_angle - scan.angle_min) / scan.angle_increment;
    const int index = static_cast<int>(std::lround(index_f));
    if (index < 0 || index >= static_cast<int>(beam_data.ranges.size()) ||
      !beam_data.observed[static_cast<size_t>(index)])
    {
      // Missing ray data is never assumed free, except for the geometrically
      // protected base_link-to-laser start region described above.
      return pointIsInStartRegion(point, beam_data.laser_to_base);
    }

    const double radial_distance = std::hypot(laser_point.x, laser_point.y);
    // Reject points at or beyond the measured endpoint after reserving the
    // configured endpoint margin. This prevents planning behind an obstacle.
    if (radial_distance + obstacle_endpoint_margin_m_ >
      beam_data.ranges[static_cast<size_t>(index)])
    {
      return false;
    }
    return true;
  }

  std::vector<Interval> buildSliceIntervals(
    const double x, const sensor_msgs::msg::LaserScan & scan,
    const BeamData & beam_data) const
  {
    // Stage 2: at one forward x slice, scan y from right to left and group
    // consecutive observedFree() samples into continuous lateral intervals.
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
        // Entering a new run of usable lateral samples.
        inside = true;
        start_y = y;
      } else if (!usable && inside) {
        // Leaving a run: retain it only if its sampled width is meaningful.
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
    // Expanded-overlap test between consecutive x slices. This permits a free
    // band to move sideways by max_interval_shift_per_slice_m_ without breaking
    // the corridor. It does not directly limit midpoint-to-midpoint movement.
    return current.low <= previous.high + max_interval_shift_per_slice_m_ &&
           current.high >= previous.low - max_interval_shift_per_slice_m_;
  }

  static bool betterPredecessor(const Branch & lhs, const Branch & rhs)
  {
    // When several older branches merge into the same new interval, preserve
    // the history with the better bottleneck width, then the straighter history.
    if (std::abs(lhs.min_width - rhs.min_width) > 1e-6) {
      return lhs.min_width > rhs.min_width;
    }
    return lhs.lateral_motion < rhs.lateral_motion;
  }

  std::vector<Branch> constructBranches(
    const sensor_msgs::msg::LaserScan & scan, const BeamData & beam_data,
    double & furthest_detected_reach, size_t & raw_branch_count) const
  {
    // Stage 3: connect free intervals across increasing x slices. Active
    // branches reach the current slice; completed branches ended earlier.
    std::vector<Branch> active;
    std::vector<Branch> completed;

    bool first_slice = true;
    for (double x = forward_start_m_; x <= forward_max_m_ + 1e-6;
      x += forward_slice_step_m_)
    {
      const auto intervals = buildSliceIntervals(x, scan, beam_data);
      if (first_slice) {
        // Root a corridor only in an interval containing y=0 or sufficiently
        // close to it. This prevents choosing disconnected remote free space.
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
        // One interval can continue from multiple earlier branches after paths
        // merge. Retain only the best predecessor for this new interval.
        bool found = false;
        Branch best;
        size_t best_index = 0;
        for (size_t branch_index = 0; branch_index < active.size(); ++branch_index) {
          if (!intervalsConnect(active[branch_index].intervals.back(), interval)) {
            continue;
          }
          Branch candidate = active[branch_index];
          // Accumulate absolute midpoint movement as a simple path-wandering cost.
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
        // An unused predecessor did not reach this slice, so its corridor ends.
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
      // Keep raw_branch_count for diagnostics, but return only corridors long
      // enough to control and containing enough intervals to define a path.
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
    // Classify branch tendency using its centres from approximately 1 m onward,
    // where a left/right decision is more meaningful than near the vehicle.
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
    // Lexicographic preference with tolerances:
    // longer reach, then larger minimum width, then less lateral movement.
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
    // Stage 4: find the best instantaneous branch, then apply side persistence
    // so small scan-to-scan differences do not repeatedly flip left/right.
    size_t best_index = 0;
    for (size_t i = 1; i < branches.size(); ++i) {
      if (branchBetter(branches[i], branches[best_index])) {
        best_index = i;
      }
    }

    if (current_side_ == 0) {
      // No established left/right history yet: accept the current best branch.
      current_side_ = branchSide(branches[best_index]);
      pending_side_ = current_side_;
      pending_side_cycles_ = 0;
      return best_index;
    }

    size_t persistent_index = branches.size();
    // Find the best branch that preserves the previously selected side.
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
      // The old side no longer exists, so switching immediately is necessary.
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
      // Keep the persistent branch unless the alternative has enough reach
      // advantage to justify starting the confirmation counter.
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
      // Switch only after the same advantageous side wins for enough scans.
      current_side_ = best_side;
      pending_side_cycles_ = 0;
      return best_index;
    }
    return persistent_index;
  }

  double previousPathYAt(const double x) const
  {
    // Interpolate the previous accepted path at an arbitrary current slice x.
    // This lets temporal smoothing work even if path lengths differ by one scan.
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
    // Stage 5: the raw candidate follows every selected interval midpoint,
    // starting at the base_link/control origin.
    raw_path.clear();
    raw_path.reserve(branch.intervals.size() + 1);
    raw_path.push_back(Point2{0.0, 0.0});
    for (const auto & interval : branch.intervals) {
      raw_path.push_back(Point2{interval.x, interval.center()});
    }
    std::vector<Point2> path = raw_path;

    if (!previous_path_.empty()) {
      // Temporal smoothing blends the new lateral request with the last accepted
      // path. Clamp back into the current interval to preserve free-space support.
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
      // Each spatial pass reduces local zig-zags. A separate vector prevents
      // early points in this pass from immediately influencing later points.
      std::vector<Point2> smoothed = path;
      for (size_t i = 1; i + 1 < path.size(); ++i) {
        if (i <= anchor_index && anchor_index > 0) {
          // Shape the beginning as a linear transition from (0,0) to the anchor.
          // Anchoring is intentionally performed inside every smoothing pass.
          const double anchor_ratio =
            path[i].x / std::max(1e-6, path[anchor_index].x);
          const double anchored = anchor_ratio * path[anchor_index].y;
          const auto & interval = branch.intervals[i - 1];
          smoothed[i].y = clampValue(anchored, interval.low, interval.high);
          continue;
        }
        const double neighbor_mean = 0.5 * (path[i - 1].y + path[i + 1].y);
        // Convex blend between the point itself and its two-neighbour mean.
        const double candidate =
          (1.0 - spatial_smoothing_weight_) * path[i].y +
          spatial_smoothing_weight_ * neighbor_mean;
        const auto & interval = branch.intervals[i - 1];
        smoothed[i].y = clampValue(candidate, interval.low, interval.high);
      }
      path = std::move(smoothed);
    }
    // Never allow smoothing to move the control origin.
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
    // Stage 6: midpoint samples may each be free while a connecting or smoothed
    // segment cuts a corner. Densely resample every segment and run exactly the
    // same observedFree() test used during corridor construction.
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
        // Start at sample 1 because the previous segment already covered its
        // start point; include samples to guarantee the endpoint is checked.
        const double ratio =
          static_cast<double>(sample) / static_cast<double>(samples);
        const double x = path[i].x + ratio * dx;
        const double y = path[i].y + ratio * dy;
        if (!observedFree(x, y, scan, beam_data)) {
          // Stop at the first failed sample so diagnostics identify the earliest
          // place where this candidate ceases to be safe/observed.
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
    // Re-evaluate the rejected point in the same order as observedFree(), but
    // record quantitative evidence instead of returning only true/false.
    const Point2 point{x, y};
    failure.required_clearance_m = envelope_radius_;

    double nearest_distance = std::numeric_limits<double>::infinity();
    Point2 nearest_obstacle;
    for (const auto & obstacle : beam_data.obstacle_points) {
      const double distance = std::hypot(obstacle.x - x, obstacle.y - y);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_obstacle = obstacle;
      }
    }
    failure.nearest_obstacle_distance_m = nearest_distance;
    if (std::isfinite(nearest_distance)) {
      failure.nearest_obstacle = nearest_obstacle;
      failure.nearest_obstacle_available = true;
    }
    if (nearest_distance < envelope_radius_) {
      // The path sample lies inside the inflated obstacle envelope.
      failure.check_code = "OBSTACLE_ENVELOPE_COLLISION";
      return;
    }

    const double base_angle = std::atan2(y, x);
    if (base_angle < planning_angle_min_rad_ || base_angle > planning_angle_max_rad_) {
      failure.beam_angle_deg = base_angle * 180.0 / kPi;
      failure.check_code = "OUTSIDE_PLANNING_ANGLE";
      return;
    }

    const Point2 laser_point = beam_data.laser_to_base.baseToLaser(point);
    // Ray-coverage diagnostics must use the original LaserScan coordinate frame.
    const double laser_angle = std::atan2(laser_point.y, laser_point.x);
    failure.beam_angle_deg = laser_angle * 180.0 / kPi;
    failure.point_range_m = std::hypot(laser_point.x, laser_point.y);
    const double index_f = (laser_angle - scan.angle_min) / scan.angle_increment;
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

    // This should be unreachable because the caller invokes this helper only
    // after observedFree() rejects the same point. Keep an explicit code so a
    // future mismatch between the two checks is visible.
    failure.check_code = "UNKNOWN_FREE_SPACE_REJECTION";
  }

  bool findLookahead(const std::vector<Point2> & path, Point2 & target) const
  {
    // Use the first discrete path point at least lookahead_distance_m_ from the
    // base_link origin. No interpolation between path points is performed here.
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
    // One complete planning pipeline:
    // scan quality -> branches -> persistent selection -> candidate paths ->
    // swept validation -> pure pursuit -> steering/reach speed modulation.
    PlanResult result;
    result.scan_valid_ratio = beam_data.valid_ratio;
    if (beam_data.valid_ratio < min_valid_beam_ratio_) {
      result.state = "INPUT_INVALID";
      result.reason = "insufficient valid forward laser beams";
      return result;
    }

    size_t raw_branch_count = 0;
    // raw_branch_count includes short completed branches for diagnosis, whereas
    // branches contains only candidates that pass the minimum reach/size filter.
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
    if (enable_swept_path_validation_) {
      // Prefer the smoother candidate. If smoothing created a collision or moved
      // outside observed space, retry the unsmoothed midpoint path for this same
      // selected branch; these are not alternative left/right branches.
      if (!pathIsValid(
          result.path, scan, beam_data, "smoothed", smoothed_failure))
      {
        result.smoothed_failure = smoothed_failure;
        ValidationFailure raw_failure;
        if (pathIsValid(raw_path, scan, beam_data, "raw", raw_failure)) {
          result.path = std::move(raw_path);
          result.used_raw_fallback = true;
        } else {
          // Both geometric candidates are invalid. Hysteresis is applied after
          // makePlan(), before this result is published or consumed downstream.
          result.raw_failure = raw_failure;
          result.state = "PATH_INVALID";
          result.reason =
            "both smoothed and raw corridor midlines fail swept-path validation";
          return result;
        }
      }
    }
    // Only a path that passed complete validation may influence the next scan's
    // temporal smoothing.
    previous_path_ = result.path;
    if (!findLookahead(result.path, result.lookahead)) {
      result.state = "BLOCKED";
      result.reason = "selected corridor is shorter than pure-pursuit lookahead";
      return result;
    }

    const double target_distance =
      std::hypot(result.lookahead.x, result.lookahead.y);
    // Pure-pursuit curvature for a target expressed in the vehicle/base frame.
    const double curvature =
      2.0 * result.lookahead.y / (target_distance * target_distance);
    const double raw_steering = clampValue(
      std::atan(wheelbase_m_ * curvature), -steering_max_rad_, steering_max_rad_);
    result.steering =
      // Low-pass filtering reduces scan-to-scan steering command jumps.
      steering_filter_alpha_ * raw_steering +
      (1.0 - steering_filter_alpha_) * previous_steering_;
    previous_steering_ = result.steering;

    const double steering_ratio = clampValue(
      std::abs(result.steering) / std::max(1e-6, steering_max_rad_), 0.0, 1.0);
    const double steering_speed =
      // Interpolate from maximum straight speed toward minimum speed as the
      // filtered steering magnitude approaches steering_max_rad_.
      velocity_max_mps_ -
      steering_ratio * (velocity_max_mps_ - velocity_min_mps_);
    const double reach_factor = clampValue(
      // A corridor at/below stop reach gets factor 0; at/above slow reach it
      // gets factor 1; the interval between those thresholds is linear.
      (result.branch.reach() - stop_reach_distance_m_) /
      (slow_reach_distance_m_ - stop_reach_distance_m_), 0.0, 1.0);
    result.speed = steering_speed * reach_factor;
    if (result.branch.reach() <= stop_reach_distance_m_) {
      // Explicit guard documents and enforces the exact stop threshold.
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

  void applySweptPathValidationHysteresis(PlanResult & result)
  {
    if (!enable_swept_path_validation_) {
      swept_path_failure_cycles_ = 0;
      swept_path_recovery_cycles_ = 0;
      swept_path_failure_latched_ = false;
      return;
    }

    if (result.state == "PATH_INVALID") {
      swept_path_recovery_cycles_ = 0;
      if (!swept_path_failure_latched_) {
        swept_path_failure_cycles_ = std::min(
          swept_path_failure_cycles_ + 1,
          swept_path_failure_confirmation_cycles_);
        if (swept_path_failure_cycles_ >= swept_path_failure_confirmation_cycles_) {
          swept_path_failure_latched_ = true;
        } else {
          // Stop immediately, but do not request lower-layer FTG until the
          // geometric failure has persisted for the configured number of scans.
          result.state = "PATH_VALIDATION_PENDING";
          result.reason =
            "swept-path failure awaiting confirmation (" +
            std::to_string(swept_path_failure_cycles_) + "/" +
            std::to_string(swept_path_failure_confirmation_cycles_) + ")";
        }
      }
      return;
    }

    if (!swept_path_failure_latched_) {
      // Only consecutive PATH_INVALID results count toward entry.
      swept_path_failure_cycles_ = 0;
      swept_path_recovery_cycles_ = 0;
      return;
    }

    if (result.valid) {
      swept_path_recovery_cycles_ = std::min(
        swept_path_recovery_cycles_ + 1,
        swept_path_recovery_confirmation_cycles_);
      if (swept_path_recovery_cycles_ >= swept_path_recovery_confirmation_cycles_) {
        swept_path_failure_latched_ = false;
        swept_path_failure_cycles_ = 0;
        swept_path_recovery_cycles_ = 0;
      } else {
        // Keep PATH_INVALID latched so the lower controller remains in FTG (if
        // enabled) until free-path recovery is stable for consecutive scans.
        result.valid = false;
        result.state = "PATH_INVALID";
        result.reason =
          "swept-path recovery awaiting confirmation (" +
          std::to_string(swept_path_recovery_cycles_) + "/" +
          std::to_string(swept_path_recovery_confirmation_cycles_) + ")";
      }
    } else {
      // Unrelated planner failures keep their own immediate state and do not
      // count as proof that swept-path validation has recovered.
      swept_path_recovery_cycles_ = 0;
    }
  }

  std::string validationFailureText(const ValidationFailure & failure) const
  {
    // Render one compact but complete description of the first failed sample.
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
      if (failure.nearest_obstacle_available) {
        stream << ", nearest_obstacle=(" << failure.nearest_obstacle.x
               << "," << failure.nearest_obstacle.y << ")";
      }
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
    // Default mode reports state transitions only. Full mode also reports
    // periodic geometry/control evidence useful during tuning and debugging.
    const auto wall_now = std::chrono::steady_clock::now();
    const bool state_changed =
      result.state != last_terminal_state_ || result.reason != last_terminal_reason_;

    if (!full_terminal_debug_) {
      if (!result.valid && state_changed) {
        if (result.state == "PATH_INVALID") {
          RCLCPP_WARN(
            get_logger(),
            "state=PATH_INVALID -> STOP | reason=%s | smoothed=%s, raw=%s",
            result.reason.c_str(), result.smoothed_failure.check_code.c_str(),
            result.raw_failure.check_code.c_str());
        } else {
          RCLCPP_WARN(
            get_logger(), "state=%s -> STOP | reason=%s",
            result.state.c_str(), result.reason.c_str());
        }
      }
      if (state_changed) {
        last_terminal_state_ = result.state;
        last_terminal_reason_ = result.reason;
        last_terminal_log_time_ = wall_now;
      }
      return;
    }

    const double elapsed_sec =
      std::chrono::duration<double>(wall_now - last_terminal_log_time_).count();
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
    // LaserScan is the main execution clock: one valid scan produces at most one
    // new plan and nominal Ackermann command.
    std::lock_guard<std::mutex> lock(mutex_);
    scan_received_ = true;
    last_scan_receive_time_ = now();
    last_frame_id_ = base_frame_;

    if (!enabled_) {
      // All failure gates publish an explicit zero command rather than silently
      // returning and leaving an earlier command active.
      publishStop("DISABLED", "enable input is false", base_frame_);
      return;
    }
    if (require_fresh_odom_ &&
      (!odom_received_ || (now() - last_odom_receive_time_).seconds() > odom_timeout_sec_))
    {
      publishStop("ODOM_STALE", "fresh odometry is required", base_frame_);
      return;
    }
    if (scan->ranges.empty() || scan->angle_increment <= 0.0) {
      publishStop("INPUT_INVALID", "empty or malformed LaserScan", base_frame_);
      return;
    }

    LaserToBaseTransform laser_to_base;
    std::string transform_error;
    if (!lookupLaserToBaseTransform(*scan, laser_to_base, transform_error)) {
      // Never fall back to treating the laser origin as base_link; doing so
      // would shift both the path origin and collision envelope.
      publishStop(
        "TF_UNAVAILABLE",
        "cannot transform LaserScan into configured base_frame", base_frame_);
      if (full_terminal_debug_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TF lookup %s <- %s failed: %s",
          base_frame_.c_str(), scan->header.frame_id.c_str(), transform_error.c_str());
      }
      return;
    }

    if (full_terminal_debug_ && !first_scan_logged_) {
      RCLCPP_INFO(
        get_logger(),
        "First LaserScan: frame=%s -> %s, beams=%zu, "
        "laser_origin_in_base=(%.3f, %.3f), angle_increment=%.6f rad",
        scan->header.frame_id.c_str(), base_frame_.c_str(), scan->ranges.size(),
        laser_to_base.x, laser_to_base.y, scan->angle_increment);
      first_scan_logged_ = true;
    }

    const BeamData beam_data = preprocessScan(*scan, laser_to_base);
    PlanResult result = makePlan(*scan, beam_data);
    applySweptPathValidationHysteresis(result);
    // Every published geometry item is now expressed in base_frame.
    std_msgs::msg::Header output_header = scan->header;
    output_header.frame_id = base_frame_;
    publishResult(result, output_header);
  }

  void watchdogCallback()
  {
    // Independent of LaserScan callbacks, repeatedly stop if scan input ceases.
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
    // Clear only state that influences future planning/control decisions.
    previous_path_.clear();
    current_side_ = 0;
    pending_side_ = 0;
    pending_side_cycles_ = 0;
    swept_path_failure_cycles_ = 0;
    swept_path_recovery_cycles_ = 0;
    swept_path_failure_latched_ = false;
    previous_steering_ = 0.0;
  }

  void publishResult(
    const PlanResult & result, const std_msgs::msg::Header & source_header)
  {
    // A PlanResult is the single source for command, path, status, markers and
    // terminal logs, keeping all outputs consistent for the same scan cycle.
    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header = source_header;
    command.drive.speed = result.valid ? result.speed : 0.0;
    command.drive.steering_angle = result.valid ? result.steering : 0.0;
    command_pub_->publish(command);

    if (path_pub_->get_subscription_count() > 0) {
      publishPath(result.valid ? result.path : std::vector<Point2>{}, source_header);
    }
    if (status_pub_->get_subscription_count() > 0) {
      publishStatus(result, source_header.stamp);
    }
    if (publish_visualization_ && marker_pub_->get_subscription_count() > 0) {
      const auto wall_now = std::chrono::steady_clock::now();
      const bool visualization_state_changed =
        result.state != last_visualization_state_ ||
        result.reason != last_visualization_reason_;
      const double elapsed_sec = std::chrono::duration<double>(
        wall_now - last_visualization_publish_time_).count();
      if (!visualization_published_ || visualization_state_changed ||
        visualization_period_sec_ <= 0.0 || elapsed_sec >= visualization_period_sec_)
      {
        publishMarkers(result, source_header);
        visualization_published_ = true;
        last_visualization_state_ = result.state;
        last_visualization_reason_ = result.reason;
        last_visualization_publish_time_ = wall_now;
      }
    }

    if (!result.valid) {
      // Restart steering filtering from zero after a stop condition clears.
      previous_steering_ = 0.0;
    }
    logTerminalStatus(result);
  }

  void publishStop(
    const std::string & state, const std::string & reason,
    const std::string & frame_id)
  {
    // Represent every stop as an invalid PlanResult so all output channels are
    // updated consistently, including an explicit zero Ackermann command.
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
    // Convert the internal planar polyline into nav_msgs/Path. Pose yaw follows
    // the next segment (or the previous segment for the final point).
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
    // Publish machine-readable planner state and the same failure evidence used
    // by terminal diagnostics. A future safety layer can consume these fields.
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "reactive_control_v2/upper_corridor_follower";
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
      "swept_path_validation_enabled",
      enable_swept_path_validation_ ? "true" : "false");
    add("swept_path_failure_cycles", std::to_string(swept_path_failure_cycles_));
    add("swept_path_recovery_cycles", std::to_string(swept_path_recovery_cycles_));
    add(
      "swept_path_failure_latched",
      swept_path_failure_latched_ ? "true" : "false");
    add(
      "path_source",
      (result.state == "PATH_INVALID" || result.state == "PATH_VALIDATION_PENDING") ?
      "invalid" :
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
      "smoothed_nearest_obstacle_x_m",
      result.smoothed_failure.nearest_obstacle_available ?
      numberString(result.smoothed_failure.nearest_obstacle.x) : "nan");
    add(
      "smoothed_nearest_obstacle_y_m",
      result.smoothed_failure.nearest_obstacle_available ?
      numberString(result.smoothed_failure.nearest_obstacle.y) : "nan");
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
      "raw_nearest_obstacle_x_m",
      result.raw_failure.nearest_obstacle_available ?
      numberString(result.raw_failure.nearest_obstacle.x) : "nan");
    add(
      "raw_nearest_obstacle_y_m",
      result.raw_failure.nearest_obstacle_available ?
      numberString(result.raw_failure.nearest_obstacle.y) : "nan");
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
    // Shared initialization for all short-lived RViz markers.
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
    // Clear old marker IDs first so stale paths/targets disappear immediately
    // when the planner changes state or becomes invalid.
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    if (!result.branch.intervals.empty()) {
      // Orange boundaries and fill visualize the selected free-centre corridor,
      // not the raw physical road edges.
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
      // Red sphere: pure-pursuit lookahead target.
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

      // Blue arrow: filtered steering command direction from base_link.
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
        // Red/magenta spheres identify the first rejected smoothed/raw path
        // sample. They are validation evidence, not LaserScan obstacle points.
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
  // Standard ROS 2 lifecycle: initialize, process callbacks, then shut down.
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UpperCorridorFollower>());
  rclcpp::shutdown();
  return 0;
}
