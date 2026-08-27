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
#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "reactive_control_v2/raceline_stall_handoff.hpp"
#include "reactive_control_v2/reverse_swept_safety.hpp"
#include "reactive_control_v2/wrong_way_recovery.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/u_int8.hpp"

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

double quaternionYaw(const geometry_msgs::msg::Quaternion & orientation)
{
  const double norm_squared = orientation.x * orientation.x +
    orientation.y * orientation.y + orientation.z * orientation.z +
    orientation.w * orientation.w;
  if (!std::isfinite(norm_squared) || norm_squared <= 1e-12) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double scale = 1.0 / std::sqrt(norm_squared);
  const double x = orientation.x * scale;
  const double y = orientation.y * scale;
  const double z = orientation.z * scale;
  const double w = orientation.w * scale;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double wrappedAngleDifference(const double first, const double second)
{
  return std::atan2(std::sin(first - second), std::cos(first - second));
}

int64_t sourceStampNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  constexpr int64_t kNanosecondsPerSecond = 1000000000LL;
  if (stamp.sec < 0 || stamp.nanosec >= kNanosecondsPerSecond) {
    return 0;
  }
  return static_cast<int64_t>(stamp.sec) * kNanosecondsPerSecond +
    static_cast<int64_t>(stamp.nanosec);
}

std::string normalizedFrameId(const std::string & frame)
{
  const auto first = frame.find_first_not_of('/');
  return first == std::string::npos ? std::string{} : frame.substr(first);
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
    arbitration_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
      arbitration_mode_topic_, 10,
      std::bind(&LowerSafetyController::arbitrationModeCallback, this, _1));
    heading_error_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      wrong_way_heading_error_topic_, 10,
      std::bind(&LowerSafetyController::headingErrorCallback, this, _1));
    reverse_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      reverse_swept_map_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&LowerSafetyController::reverseMapCallback, this, _1));
    simulator_status_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      reverse_swept_agent_status_topic_, 10,
      std::bind(&LowerSafetyController::simulatorStatusCallback, this, _1));

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
      "reactive_control_v2 v0.3.0 lower_safety_controller ready: "
      "selected=%s, scan=%s, odom=%s, safe=%s, "
      "upper_failure_status_fallback=%s, arbitration_mode_required=%s, "
      "command_limits=|speed|<=%.1f m/s and |steering|<=%.1f deg, "
      "reverse_recovery=%s, raceline_stall_handoff=%s, "
      "wrong_way_recovery=%s, low_speed_assist=%s, ftg_debug=%s",
      selected_command_topic_.c_str(), scan_topic_.c_str(), odom_topic_.c_str(),
      safe_command_topic_.c_str(),
      enable_fallback_on_upper_failure_status_ ? "true" : "false",
      require_arbitration_mode_ ? "true" : "false",
      absolute_speed_limit_mps_, absolute_steering_limit_deg_,
      enable_reverse_recovery_ ? "true" : "false",
      enable_raceline_stall_handoff_ ? "true" : "false",
      enable_wrong_way_recovery_ ? "true" : "false",
      enable_low_speed_assist_ ? "true" : "false",
      fallback_terminal_debug_ ? "true" : "false");
  }

private:
  enum class Mode
  {
    NOMINAL,
    FALLBACK_FTG,
    EMERGENCY_STOP,
    REVERSE_RECOVERY,
    RECOVERY_SETTLE
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

  struct ReverseSafety
  {
    bool valid{false};
    std::string reason;
    double valid_ratio{0.0};
    double minimum_clearance_m{std::numeric_limits<double>::infinity()};
    double left_valid_ratio{0.0};
    double right_valid_ratio{0.0};
    double left_clearance_m{std::numeric_limits<double>::infinity()};
    double right_clearance_m{std::numeric_limits<double>::infinity()};
  };

  // ROS interfaces
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr upper_status_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr arbitration_mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr heading_error_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr reverse_map_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr simulator_status_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr safe_command_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // A monotonic clock owns all internal elapsed-time and freshness logic.
  // ROS time remains reserved for published message headers, so simulation
  // pause/reset or a missing /clock cannot freeze safety watchdogs/recovery.
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // Shared callback data. The current launch uses a single-threaded executor,
  // but the mutex keeps the ownership clear and remains correct if that changes.
  std::mutex mutex_;
  ackermann_msgs::msg::AckermannDriveStamped::SharedPtr latest_command_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_reverse_map_;
  std::vector<reactive_control_v2::reverse_swept_safety::Actor> simulator_actors_;
  reactive_control_v2::reverse_swept_safety::Pose2 simulator_ego_pose_;
  bool simulator_ego_pose_valid_{false};
  reactive_control_v2::reverse_swept_safety::Pose2 current_odom_pose_;
  std::string current_odom_frame_;
  rclcpp::Time last_command_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_scan_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_upper_status_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_arbitration_mode_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_heading_error_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_simulator_status_time_{0, 0, RCL_STEADY_TIME};
  int64_t heading_error_source_stamp_ns_{0};
  int64_t odom_source_stamp_ns_{0};
  int64_t simulator_status_source_stamp_ns_{0};
  uint64_t heading_error_sequence_{0};
  bool command_received_{false};
  bool scan_received_{false};
  bool odom_received_{false};
  bool upper_status_received_{false};
  bool arbitration_mode_received_{false};
  bool heading_error_received_{false};
  bool heading_error_frame_valid_{false};
  bool reverse_map_received_{false};
  bool simulator_status_received_{false};
  double current_speed_mps_{0.0};
  double current_heading_error_rad_{0.0};
  double current_raceline_distance_m_{std::numeric_limits<double>::infinity()};
  double current_raceline_station_m_{std::numeric_limits<double>::quiet_NaN()};
  std::string current_heading_error_frame_;
  std::string upper_state_;
  uint8_t arbitration_mode_{0};

  // Topic and timing parameters
  std::string selected_command_topic_;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string upper_status_topic_;
  std::string arbitration_mode_topic_;
  std::string wrong_way_heading_error_topic_;
  std::string safe_command_topic_;
  std::string lower_status_topic_;
  double control_frequency_hz_{20.0};
  double command_timeout_sec_{0.30};
  double scan_timeout_sec_{0.30};
  double odom_timeout_sec_{0.50};
  double upper_status_timeout_sec_{0.50};
  bool enable_fallback_on_upper_failure_status_{true};
  bool require_arbitration_mode_{false};
  double arbitration_mode_timeout_sec_{0.50};
  double wrong_way_heading_error_timeout_sec_{0.20};
  double wrong_way_max_raceline_distance_m_{4.0};
  bool enable_sim_reverse_swept_gate_{false};
  std::string reverse_swept_map_topic_{"/map"};
  std::string reverse_swept_agent_status_topic_{"/simulator/agent_status"};
  double reverse_swept_status_timeout_sec_{0.10};
  double reverse_swept_sample_step_m_{0.02};
  double reverse_vehicle_length_m_{0.58};
  double reverse_vehicle_width_m_{0.31};
  double reverse_swept_margin_m_{0.03};
  double reverse_wheelbase_m_{0.33};
  int reverse_map_occupied_threshold_{50};

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
  rclcpp::Time last_fallback_debug_time_{0, 0, RCL_STEADY_TIME};

  // Temporary bidirectional assistance for the VESC low-speed stall zone.
  // The demand ceiling and forced output are deliberately separate values.
  bool enable_low_speed_assist_{true};
  double low_speed_assist_demand_max_mps_{1.00};
  double low_speed_assist_output_mps_{1.00};
  double low_speed_assist_entry_shortfall_mps_{0.50};
  double low_speed_assist_stall_speed_mps_{0.30};
  double low_speed_assist_confirmation_sec_{0.30};
  double low_speed_assist_exit_shortfall_mps_{0.20};

  // Bounded reverse recovery. VESC-derived odometry is the primary motion
  // feedback. The structure intentionally leaves room for a later independent
  // scan-motion confidence source without making it mandatory now.
  bool enable_reverse_recovery_{true};
  bool enable_raceline_stall_handoff_{false};
  bool enable_wrong_way_recovery_{false};
  double stationary_speed_threshold_mps_{0.05};
  double dead_end_confirmation_sec_{1.0};
  double stuck_forward_command_threshold_mps_{0.20};
  double stuck_speed_threshold_mps_{0.05};
  double stuck_confirmation_sec_{1.0};
  double reverse_speed_mps_{0.25};
  double reverse_min_distance_m_{0.10};
  double reverse_min_duration_sec_{0.30};
  double reverse_max_distance_m_{0.50};
  double reverse_max_duration_sec_{2.0};
  int ftg_recovery_valid_cycles_{5};
  double recovery_settle_time_sec_{0.20};
  int reverse_max_attempts_{10};
  double reverse_attempt_reset_forward_time_sec_{0.30};
  double reverse_attempt_reset_speed_mps_{0.20};
  bool reverse_require_side_clearance_{true};
  double reverse_side_sector_min_angle_deg_{100.0};
  double reverse_side_sector_max_angle_deg_{135.0};
  double reverse_min_side_clearance_m_{0.20};
  double reverse_side_min_valid_ratio_{0.25};
  double wrong_way_stop_angle_rad_{degreesToRadians(90.0)};
  double wrong_way_entry_angle_rad_{degreesToRadians(90.0)};
  double wrong_way_exit_angle_rad_{degreesToRadians(80.0)};
  double wrong_way_confirmation_sec_{0.20};
  int wrong_way_confirmation_samples_{3};
  double wrong_way_exit_confirmation_sec_{0.15};
  int wrong_way_exit_confirmation_samples_{3};
  double wrong_way_reverse_steering_rad_{degreesToRadians(25.0)};
  double wrong_way_reverse_min_side_clearance_m_{0.50};
  int wrong_way_reverse_max_attempts_{4};
  bool reverse_terminal_debug_{false};
  double reverse_terminal_debug_period_sec_{0.50};

  // Recovery state and timers.
  bool reverse_active_{false};
  bool settle_active_{false};
  rclcpp::Time reverse_started_at_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time reverse_last_update_at_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time settle_stationary_since_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time dead_end_started_at_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time stuck_started_at_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time forward_motion_started_at_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_reverse_debug_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time low_speed_assist_candidate_since_{0, 0, RCL_STEADY_TIME};
  bool dead_end_timer_active_{false};
  bool stuck_timer_active_{false};
  bool settle_stationary_timer_active_{false};
  bool forward_motion_timer_active_{false};
  bool low_speed_assist_candidate_active_{false};
  bool low_speed_assist_active_{false};
  bool control_arbitration_mode_valid_{false};
  uint8_t last_control_arbitration_mode_{0};
  bool raceline_stall_handoff_evidence_{false};
  bool raceline_stall_handoff_active_{false};
  bool raceline_stall_handoff_latched_{false};
  bool wrong_way_timer_active_{false};
  bool wrong_way_suspect_latched_{false};
  bool wrong_way_recovery_latched_{false};
  bool wrong_way_reverse_active_{false};
  bool wrong_way_evidence_{false};
  bool wrong_way_alignment_recovered_{false};
  rclcpp::Time wrong_way_started_at_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time wrong_way_alignment_started_at_{0, 0, RCL_STEADY_TIME};
  uint64_t last_wrong_way_heading_sequence_{0};
  uint64_t last_wrong_way_alignment_sequence_{0};
  int wrong_way_confirmation_sample_count_{0};
  int wrong_way_alignment_sample_count_{0};
  bool wrong_way_alignment_timer_active_{false};
  int wrong_way_turn_sign_{0};
  int wrong_way_reverse_attempt_count_{0};
  bool wrong_way_reverse_attempt_committed_{false};
  bool reverse_swept_gate_valid_{false};
  double reverse_swept_checked_distance_m_{0.0};
  std::string reverse_swept_gate_reason_{"not evaluated"};
  double last_published_steering_rad_{0.0};
  int low_speed_assist_direction_{0};
  double low_speed_assist_requested_mps_{0.0};
  double low_speed_assist_output_command_mps_{0.0};
  double low_speed_assist_actual_magnitude_mps_{0.0};
  double low_speed_assist_shortfall_mps_{0.0};
  double reverse_distance_m_{0.0};
  int ftg_recovery_valid_count_{0};
  int reverse_attempt_count_{0};
  std::string reverse_trigger_reason_;
  std::string settle_reason_;

  // State retained for concise transition logs and diagnostics.
  Mode last_mode_{Mode::EMERGENCY_STOP};
  std::string last_reason_;
  rclcpp::Time stopped_since_{0, 0, RCL_STEADY_TIME};
  bool stopped_timer_active_{false};

  void declareParameters()
  {
    declare_parameter<std::string>(
      "selected_command_topic", "/reactive_control_v2/selected_cmd");
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
    declare_parameter<std::string>("upper_status_topic", "/reactive_control_v2/status");
    declare_parameter<std::string>(
      "arbitration_mode_topic", "/drive_arbitration_v2/selected_mode");
    declare_parameter<std::string>(
      "wrong_way_heading_error_topic",
      "/path_following_v2/raceline_heading_error_rad");
    declare_parameter<std::string>("safe_command_topic", "/reactive_control_v2/safe_cmd");
    declare_parameter<std::string>(
      "lower_status_topic", "/reactive_control_v2/lower_safety_status");
    declare_parameter<double>("control_frequency_hz", 20.0);
    declare_parameter<double>("command_timeout_sec", 0.30);
    declare_parameter<double>("scan_timeout_sec", 0.30);
    declare_parameter<double>("odom_timeout_sec", 0.50);
    declare_parameter<double>("upper_status_timeout_sec", 0.50);
    declare_parameter<bool>("enable_fallback_on_upper_failure_status", true);
    // Standalone Reactive V2 remains backward compatible when false. The
    // integrated master launch overrides this to true so FTG/reverse can only
    // run while drive_arbitration_v2 explicitly selects REACTIVE mode.
    declare_parameter<bool>("require_arbitration_mode", false);
    declare_parameter<double>("arbitration_mode_timeout_sec", 0.50);
    declare_parameter<double>("wrong_way_heading_error_timeout_sec", 0.20);
    declare_parameter<double>("wrong_way_max_raceline_distance_m", 4.0);
    declare_parameter<bool>("enable_sim_reverse_swept_gate", false);
    declare_parameter<std::string>("reverse_swept_map_topic", "/map");
    declare_parameter<std::string>(
      "reverse_swept_agent_status_topic", "/simulator/agent_status");
    declare_parameter<double>("reverse_swept_status_timeout_sec", 0.10);
    declare_parameter<double>("reverse_swept_sample_step_m", 0.02);
    declare_parameter<double>("reverse_vehicle_length_m", 0.58);
    declare_parameter<double>("reverse_vehicle_width_m", 0.31);
    declare_parameter<double>("reverse_swept_margin_m", 0.03);
    declare_parameter<double>("reverse_wheelbase_m", 0.33);
    declare_parameter<int>("reverse_map_occupied_threshold", 50);

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

    declare_parameter<bool>("enable_low_speed_assist", true);
    declare_parameter<double>("low_speed_assist_demand_max_mps", 1.00);
    declare_parameter<double>("low_speed_assist_output_mps", 1.00);
    declare_parameter<double>("low_speed_assist_entry_shortfall_mps", 0.50);
    declare_parameter<double>("low_speed_assist_stall_speed_mps", 0.30);
    declare_parameter<double>("low_speed_assist_confirmation_sec", 0.30);
    declare_parameter<double>("low_speed_assist_exit_shortfall_mps", 0.20);

    declare_parameter<bool>("enable_reverse_recovery", true);
    declare_parameter<bool>("enable_raceline_stall_handoff", false);
    declare_parameter<bool>("enable_wrong_way_recovery", false);
    declare_parameter<double>("stationary_speed_threshold_mps", 0.05);
    declare_parameter<double>("dead_end_confirmation_sec", 1.0);
    declare_parameter<double>("stuck_forward_command_threshold_mps", 0.20);
    declare_parameter<double>("stuck_speed_threshold_mps", 0.05);
    declare_parameter<double>("stuck_confirmation_sec", 1.0);
    declare_parameter<double>("reverse_speed_mps", 0.25);
    declare_parameter<double>("reverse_min_distance_m", 0.10);
    declare_parameter<double>("reverse_min_duration_sec", 0.30);
    declare_parameter<double>("reverse_max_distance_m", 0.50);
    declare_parameter<double>("reverse_max_duration_sec", 2.0);
    declare_parameter<int>("ftg_recovery_valid_cycles", 5);
    declare_parameter<double>("recovery_settle_time_sec", 0.20);
    declare_parameter<int>("reverse_max_attempts", 10);
    declare_parameter<double>("reverse_attempt_reset_forward_time_sec", 0.30);
    declare_parameter<double>("reverse_attempt_reset_speed_mps", 0.20);
    declare_parameter<bool>("reverse_require_side_clearance", true);
    declare_parameter<double>("reverse_side_sector_min_angle_deg", 100.0);
    declare_parameter<double>("reverse_side_sector_max_angle_deg", 135.0);
    declare_parameter<double>("reverse_min_side_clearance_m", 0.20);
    declare_parameter<double>("reverse_side_min_valid_ratio", 0.25);
    declare_parameter<double>("wrong_way_stop_angle_deg", 90.0);
    declare_parameter<double>("wrong_way_entry_angle_deg", 90.0);
    declare_parameter<double>("wrong_way_exit_angle_deg", 80.0);
    declare_parameter<double>("wrong_way_confirmation_sec", 0.20);
    declare_parameter<int>("wrong_way_confirmation_samples", 3);
    declare_parameter<double>("wrong_way_exit_confirmation_sec", 0.15);
    declare_parameter<int>("wrong_way_exit_confirmation_samples", 3);
    declare_parameter<double>("wrong_way_reverse_steering_deg", 25.0);
    declare_parameter<double>("wrong_way_reverse_min_side_clearance_m", 0.50);
    declare_parameter<int>("wrong_way_reverse_max_attempts", 4);
    declare_parameter<bool>("reverse_terminal_debug", false);
    declare_parameter<double>("reverse_terminal_debug_period_sec", 0.50);
  }

  void loadParameters()
  {
    selected_command_topic_ = get_parameter("selected_command_topic").as_string();
    scan_topic_ = get_parameter("scan_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    upper_status_topic_ = get_parameter("upper_status_topic").as_string();
    arbitration_mode_topic_ = get_parameter("arbitration_mode_topic").as_string();
    wrong_way_heading_error_topic_ =
      get_parameter("wrong_way_heading_error_topic").as_string();
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
    require_arbitration_mode_ = get_parameter("require_arbitration_mode").as_bool();
    arbitration_mode_timeout_sec_ = std::max(
      0.01, get_parameter("arbitration_mode_timeout_sec").as_double());
    wrong_way_heading_error_timeout_sec_ = std::max(
      0.01, get_parameter("wrong_way_heading_error_timeout_sec").as_double());
    wrong_way_max_raceline_distance_m_ = std::max(
      0.1, get_parameter("wrong_way_max_raceline_distance_m").as_double());
    enable_sim_reverse_swept_gate_ =
      get_parameter("enable_sim_reverse_swept_gate").as_bool();
    reverse_swept_map_topic_ = get_parameter("reverse_swept_map_topic").as_string();
    reverse_swept_agent_status_topic_ =
      get_parameter("reverse_swept_agent_status_topic").as_string();
    reverse_swept_status_timeout_sec_ = std::max(
      0.02, get_parameter("reverse_swept_status_timeout_sec").as_double());
    reverse_swept_sample_step_m_ = clampValue(
      get_parameter("reverse_swept_sample_step_m").as_double(), 0.005, 0.05);
    reverse_vehicle_length_m_ = std::max(
      0.1, get_parameter("reverse_vehicle_length_m").as_double());
    reverse_vehicle_width_m_ = std::max(
      0.1, get_parameter("reverse_vehicle_width_m").as_double());
    reverse_swept_margin_m_ = std::max(
      0.0, get_parameter("reverse_swept_margin_m").as_double());
    reverse_wheelbase_m_ = std::max(
      0.1, get_parameter("reverse_wheelbase_m").as_double());
    reverse_map_occupied_threshold_ = static_cast<int>(clampValue(
      static_cast<double>(get_parameter("reverse_map_occupied_threshold").as_int()),
      1.0, 100.0));

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

    enable_low_speed_assist_ = get_parameter("enable_low_speed_assist").as_bool();
    low_speed_assist_demand_max_mps_ =
      std::max(0.0, get_parameter("low_speed_assist_demand_max_mps").as_double());
    low_speed_assist_output_mps_ =
      std::max(0.0, get_parameter("low_speed_assist_output_mps").as_double());
    low_speed_assist_entry_shortfall_mps_ =
      std::max(0.0, get_parameter("low_speed_assist_entry_shortfall_mps").as_double());
    low_speed_assist_stall_speed_mps_ =
      std::max(0.0, get_parameter("low_speed_assist_stall_speed_mps").as_double());
    low_speed_assist_confirmation_sec_ =
      std::max(0.0, get_parameter("low_speed_assist_confirmation_sec").as_double());
    low_speed_assist_exit_shortfall_mps_ =
      std::max(0.0, get_parameter("low_speed_assist_exit_shortfall_mps").as_double());

    enable_reverse_recovery_ = get_parameter("enable_reverse_recovery").as_bool();
    enable_raceline_stall_handoff_ =
      get_parameter("enable_raceline_stall_handoff").as_bool();
    enable_wrong_way_recovery_ = get_parameter("enable_wrong_way_recovery").as_bool();
    stationary_speed_threshold_mps_ =
      std::max(0.0, get_parameter("stationary_speed_threshold_mps").as_double());
    dead_end_confirmation_sec_ =
      std::max(0.0, get_parameter("dead_end_confirmation_sec").as_double());
    stuck_forward_command_threshold_mps_ =
      std::max(0.0, get_parameter("stuck_forward_command_threshold_mps").as_double());
    stuck_speed_threshold_mps_ =
      std::max(0.0, get_parameter("stuck_speed_threshold_mps").as_double());
    stuck_confirmation_sec_ =
      std::max(0.0, get_parameter("stuck_confirmation_sec").as_double());
    reverse_speed_mps_ = std::max(0.0, get_parameter("reverse_speed_mps").as_double());
    reverse_min_distance_m_ =
      std::max(0.0, get_parameter("reverse_min_distance_m").as_double());
    reverse_min_duration_sec_ =
      std::max(0.0, get_parameter("reverse_min_duration_sec").as_double());
    reverse_max_distance_m_ = std::max(
      reverse_min_distance_m_, get_parameter("reverse_max_distance_m").as_double());
    reverse_max_duration_sec_ = std::max(
      reverse_min_duration_sec_, get_parameter("reverse_max_duration_sec").as_double());
    ftg_recovery_valid_cycles_ = std::max(
      1, static_cast<int>(get_parameter("ftg_recovery_valid_cycles").as_int()));
    recovery_settle_time_sec_ =
      std::max(0.0, get_parameter("recovery_settle_time_sec").as_double());
    reverse_max_attempts_ = std::max(
      1, static_cast<int>(get_parameter("reverse_max_attempts").as_int()));
    reverse_attempt_reset_forward_time_sec_ = std::max(
      0.0, get_parameter("reverse_attempt_reset_forward_time_sec").as_double());
    reverse_attempt_reset_speed_mps_ = std::max(
      0.0, get_parameter("reverse_attempt_reset_speed_mps").as_double());
    reverse_require_side_clearance_ =
      get_parameter("reverse_require_side_clearance").as_bool();
    reverse_side_sector_min_angle_deg_ = clampValue(
      get_parameter("reverse_side_sector_min_angle_deg").as_double(), 0.0, 180.0);
    reverse_side_sector_max_angle_deg_ = clampValue(
      get_parameter("reverse_side_sector_max_angle_deg").as_double(),
      reverse_side_sector_min_angle_deg_, 180.0);
    reverse_min_side_clearance_m_ =
      std::max(0.0, get_parameter("reverse_min_side_clearance_m").as_double());
    reverse_side_min_valid_ratio_ = clampValue(
      get_parameter("reverse_side_min_valid_ratio").as_double(), 0.0, 1.0);
    const double wrong_way_stop_angle_deg = clampValue(
      get_parameter("wrong_way_stop_angle_deg").as_double(), 90.0, 179.0);
    const double wrong_way_entry_angle_deg = clampValue(
      get_parameter("wrong_way_entry_angle_deg").as_double(), 90.0, 180.0);
    const double wrong_way_exit_angle_deg = clampValue(
      get_parameter("wrong_way_exit_angle_deg").as_double(), 0.0,
      wrong_way_stop_angle_deg - 1.0);
    wrong_way_stop_angle_rad_ = degreesToRadians(
      std::min(wrong_way_stop_angle_deg, wrong_way_entry_angle_deg));
    wrong_way_entry_angle_rad_ = degreesToRadians(wrong_way_entry_angle_deg);
    wrong_way_exit_angle_rad_ = degreesToRadians(wrong_way_exit_angle_deg);
    wrong_way_confirmation_sec_ = std::max(
      0.0, get_parameter("wrong_way_confirmation_sec").as_double());
    wrong_way_confirmation_samples_ = std::max(
      1, static_cast<int>(get_parameter("wrong_way_confirmation_samples").as_int()));
    wrong_way_exit_confirmation_sec_ = std::max(
      0.0, get_parameter("wrong_way_exit_confirmation_sec").as_double());
    wrong_way_exit_confirmation_samples_ = std::max(
      1, static_cast<int>(get_parameter("wrong_way_exit_confirmation_samples").as_int()));
    wrong_way_reverse_steering_rad_ = degreesToRadians(clampValue(
      get_parameter("wrong_way_reverse_steering_deg").as_double(), 0.0,
      std::min(absolute_steering_limit_deg_, fallback_steering_limit_deg_)));
    wrong_way_reverse_min_side_clearance_m_ = std::max(
      reverse_min_side_clearance_m_,
      get_parameter("wrong_way_reverse_min_side_clearance_m").as_double());
    wrong_way_reverse_max_attempts_ = std::clamp(
      static_cast<int>(get_parameter("wrong_way_reverse_max_attempts").as_int()), 1, 4);
    reverse_terminal_debug_ = get_parameter("reverse_terminal_debug").as_bool();
    reverse_terminal_debug_period_sec_ = std::max(
      0.05, get_parameter("reverse_terminal_debug_period_sec").as_double());
  }

  void commandCallback(
    const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr command)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_command_ = command;
    last_command_time_ = steady_clock_.now();
    command_received_ = true;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_scan_ = scan;
    last_scan_time_ = steady_clock_.now();
    scan_received_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_speed_mps_ = odom->twist.twist.linear.x;
    current_odom_pose_.x = odom->pose.pose.position.x;
    current_odom_pose_.y = odom->pose.pose.position.y;
    current_odom_pose_.yaw = quaternionYaw(odom->pose.pose.orientation);
    current_odom_frame_ = odom->header.frame_id;
    odom_source_stamp_ns_ = sourceStampNanoseconds(odom->header.stamp);
    last_odom_time_ = steady_clock_.now();
    odom_received_ = true;
  }

  void reverseMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_reverse_map_ = map;
    reverse_map_received_ = true;
  }

  void simulatorStatusCallback(
    const diagnostic_msgs::msg::DiagnosticArray::SharedPtr array)
  {
    const int64_t source_stamp_ns = sourceStampNanoseconds(array->header.stamp);
    if (source_stamp_ns <= 0) {
      return;
    }

    std::vector<reactive_control_v2::reverse_swept_safety::Actor> actors;
    reactive_control_v2::reverse_swept_safety::Pose2 ego_pose;
    bool ego_found = false;
    bool valid = !array->status.empty();
    for (const auto & status : array->status) {
      if (status.name.rfind("simulator/", 0) != 0) {
        continue;
      }
      double x = std::numeric_limits<double>::quiet_NaN();
      double y = std::numeric_limits<double>::quiet_NaN();
      double yaw = std::numeric_limits<double>::quiet_NaN();
      double signed_speed = std::numeric_limits<double>::quiet_NaN();
      double yaw_rate = std::numeric_limits<double>::quiet_NaN();
      try {
        for (const auto & value : status.values) {
          if (value.key == "x_m") {
            x = std::stod(value.value);
          } else if (value.key == "y_m") {
            y = std::stod(value.value);
          } else if (value.key == "yaw_rad") {
            yaw = std::stod(value.value);
          } else if (value.key == "body_speed_mps") {
            signed_speed = std::stod(value.value);
          } else if (value.key == "yaw_rate_radps") {
            yaw_rate = std::stod(value.value);
          }
        }
      } catch (const std::exception &) {
        valid = false;
        break;
      }
      reactive_control_v2::reverse_swept_safety::Pose2 pose{x, y, yaw};
      if (!reactive_control_v2::reverse_swept_safety::finitePose(pose) ||
        !std::isfinite(signed_speed) || !std::isfinite(yaw_rate))
      {
        valid = false;
        break;
      }
      if (status.name == "simulator/ego") {
        ego_pose = pose;
        ego_found = true;
      } else {
        reactive_control_v2::reverse_swept_safety::Actor actor;
        actor.pose = pose;
        actor.signed_speed_mps = signed_speed;
        actor.yaw_rate_radps = yaw_rate;
        actors.push_back(actor);
      }
    }
    if (!valid || !ego_found) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (source_stamp_ns <= simulator_status_source_stamp_ns_) {
      return;
    }
    simulator_actors_ = std::move(actors);
    simulator_ego_pose_ = ego_pose;
    simulator_ego_pose_valid_ = true;
    simulator_status_source_stamp_ns_ = source_stamp_ns;
    last_simulator_status_time_ = steady_clock_.now();
    simulator_status_received_ = true;
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
      last_upper_status_time_ = steady_clock_.now();
      upper_status_received_ = true;
      break;
    }
  }

  void arbitrationModeCallback(const std_msgs::msg::UInt8::SharedPtr mode)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    arbitration_mode_ = mode->data;
    last_arbitration_mode_time_ = steady_clock_.now();
    arbitration_mode_received_ = true;
  }

  void headingErrorCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr error)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const int64_t source_stamp_ns = sourceStampNanoseconds(error->header.stamp);
    if (source_stamp_ns <= 0 || source_stamp_ns <= heading_error_source_stamp_ns_) {
      return;
    }
    current_heading_error_rad_ = error->vector.x;
    current_raceline_distance_m_ = error->vector.y;
    current_raceline_station_m_ = error->vector.z;
    current_heading_error_frame_ = normalizedFrameId(error->header.frame_id);
    heading_error_frame_valid_ = !current_heading_error_frame_.empty();
    heading_error_source_stamp_ns_ = source_stamp_ns;
    last_heading_error_time_ = steady_clock_.now();
    ++heading_error_sequence_;
    heading_error_received_ = true;
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

  ReverseSafety evaluateReverseSafety(const sensor_msgs::msg::LaserScan & scan) const
  {
    ReverseSafety result;
    if (!reverse_require_side_clearance_) {
      result.valid = true;
      result.reason = "rear-side clearance check disabled";
      return result;
    }
    if (scan.ranges.empty() || !std::isfinite(scan.angle_increment) ||
      scan.angle_increment <= 0.0)
    {
      result.reason = "empty or malformed LaserScan for reverse-side check";
      return result;
    }

    const double minimum_angle = degreesToRadians(reverse_side_sector_min_angle_deg_);
    const double maximum_angle = degreesToRadians(reverse_side_sector_max_angle_deg_);
    const bool range_max_valid = std::isfinite(scan.range_max) && scan.range_max > 0.0;
    const double range_min = std::isfinite(scan.range_min) ?
      std::max(0.0, static_cast<double>(scan.range_min)) : 0.0;
    size_t sector_beams = 0;
    size_t valid_beams = 0;
    size_t left_sector_beams = 0;
    size_t right_sector_beams = 0;
    size_t left_valid_beams = 0;
    size_t right_valid_beams = 0;
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double absolute_angle = std::abs(angle);
      if (absolute_angle < minimum_angle || absolute_angle > maximum_angle) {
        continue;
      }
      ++sector_beams;
      const bool left = angle >= 0.0;
      if (left) {
        ++left_sector_beams;
      } else {
        ++right_sector_beams;
      }
      const double raw = scan.ranges[i];
      const bool finite_hit = std::isfinite(raw) && raw >= range_min &&
        (!range_max_valid || raw <= scan.range_max);
      const bool clear_to_max = std::isinf(raw) ||
        (std::isfinite(raw) && range_max_valid && raw > scan.range_max);
      if (finite_hit) {
        ++valid_beams;
        result.minimum_clearance_m = std::min(result.minimum_clearance_m, raw);
        if (left) {
          ++left_valid_beams;
          result.left_clearance_m = std::min(result.left_clearance_m, raw);
        } else {
          ++right_valid_beams;
          result.right_clearance_m = std::min(result.right_clearance_m, raw);
        }
      } else if (clear_to_max) {
        ++valid_beams;
        if (left) {
          ++left_valid_beams;
        } else {
          ++right_valid_beams;
        }
      }
    }

    if (sector_beams == 0) {
      result.reason = "no LaserScan beams in reverse-side sectors";
      return result;
    }
    result.valid_ratio = static_cast<double>(valid_beams) /
      static_cast<double>(sector_beams);
    result.left_valid_ratio = left_sector_beams > 0 ?
      static_cast<double>(left_valid_beams) / static_cast<double>(left_sector_beams) : 0.0;
    result.right_valid_ratio = right_sector_beams > 0 ?
      static_cast<double>(right_valid_beams) / static_cast<double>(right_sector_beams) : 0.0;
    if (result.valid_ratio < reverse_side_min_valid_ratio_) {
      result.reason = "too few valid reverse-side LaserScan beams";
      return result;
    }
    if (result.minimum_clearance_m < reverse_min_side_clearance_m_) {
      result.reason = "reverse-side obstacle inside clearance threshold";
      return result;
    }
    result.valid = true;
    result.reason = "available rear-side scan evidence is clear";
    return result;
  }

  bool frontEmergencyActive(
    const ScanData & scan, const double forward_speed, std::string * reason = nullptr) const
  {
    if (scan.front_min_distance_m < emergency_distance_m_) {
      if (reason) {
        *reason = "obstacle inside hard forward emergency distance";
      }
      return true;
    }
    const double available_distance = std::isfinite(scan.front_min_distance_m) ?
      std::max(0.0, scan.front_min_distance_m - emergency_ttc_clearance_m_) :
      std::numeric_limits<double>::infinity();
    const double ttc = forward_speed > 0.05 ?
      available_distance / forward_speed : std::numeric_limits<double>::infinity();
    if (ttc < emergency_ttc_sec_) {
      if (reason) {
        *reason = "forward time-to-collision below emergency threshold";
      }
      return true;
    }
    return false;
  }

  void resetLowSpeedAssist()
  {
    low_speed_assist_candidate_active_ = false;
    low_speed_assist_active_ = false;
    low_speed_assist_direction_ = 0;
  }

  void applyLowSpeedAssist(
    ackermann_msgs::msg::AckermannDriveStamped & command,
    const double measured_speed, const bool allowed,
    const rclcpp::Time & current_time)
  {
    const double requested_speed = static_cast<double>(command.drive.speed);
    const double requested_magnitude = std::abs(requested_speed);
    const double actual_magnitude = std::abs(measured_speed);
    const double shortfall = std::max(0.0, requested_magnitude - actual_magnitude);
    const int requested_direction = requested_speed > 0.0 ? 1 :
      (requested_speed < 0.0 ? -1 : 0);

    low_speed_assist_requested_mps_ = requested_speed;
    low_speed_assist_actual_magnitude_mps_ = actual_magnitude;
    low_speed_assist_shortfall_mps_ = shortfall;
    low_speed_assist_output_command_mps_ = requested_speed;

    const bool eligible = enable_low_speed_assist_ && allowed &&
      std::isfinite(requested_speed) && std::isfinite(measured_speed) &&
      requested_direction != 0 && requested_magnitude <= low_speed_assist_demand_max_mps_;
    if (!eligible) {
      resetLowSpeedAssist();
      return;
    }

    if (low_speed_assist_direction_ != 0 &&
      low_speed_assist_direction_ != requested_direction)
    {
      resetLowSpeedAssist();
    }
    low_speed_assist_direction_ = requested_direction;

    if (low_speed_assist_active_ &&
      shortfall <= low_speed_assist_exit_shortfall_mps_)
    {
      resetLowSpeedAssist();
      low_speed_assist_direction_ = requested_direction;
    }

    const bool actual_is_lower = actual_magnitude < requested_magnitude;
    const bool entry_evidence = actual_is_lower &&
      (shortfall >= low_speed_assist_entry_shortfall_mps_ ||
      actual_magnitude <= low_speed_assist_stall_speed_mps_);
    if (!low_speed_assist_active_) {
      if (entry_evidence) {
        if (!low_speed_assist_candidate_active_) {
          low_speed_assist_candidate_since_ = current_time;
          low_speed_assist_candidate_active_ = true;
        }
        if ((current_time - low_speed_assist_candidate_since_).seconds() >=
          low_speed_assist_confirmation_sec_)
        {
          low_speed_assist_active_ = true;
          low_speed_assist_candidate_active_ = false;
        }
      } else {
        low_speed_assist_candidate_active_ = false;
      }
    }

    if (low_speed_assist_active_) {
      command.drive.speed = static_cast<double>(requested_direction) *
        std::min(low_speed_assist_output_mps_, absolute_speed_limit_mps_);
      low_speed_assist_output_command_mps_ = command.drive.speed;
    }
  }

  void clearTriggerTimers()
  {
    dead_end_timer_active_ = false;
    stuck_timer_active_ = false;
  }

  void cancelReactiveRecovery()
  {
    reverse_active_ = false;
    wrong_way_reverse_active_ = false;
    settle_active_ = false;
    settle_stationary_timer_active_ = false;
    forward_motion_timer_active_ = false;
    ftg_recovery_valid_count_ = 0;
    reverse_distance_m_ = 0.0;
    resetLowSpeedAssist();
    clearTriggerTimers();
  }

  void startReverse(const rclcpp::Time & current_time, const std::string & trigger)
  {
    reverse_active_ = true;
    wrong_way_reverse_active_ = false;
    settle_active_ = false;
    reverse_started_at_ = current_time;
    reverse_last_update_at_ = current_time;
    reverse_distance_m_ = 0.0;
    resetLowSpeedAssist();
    ftg_recovery_valid_count_ = 0;
    ++reverse_attempt_count_;
    reverse_trigger_reason_ = trigger;
    clearTriggerTimers();
  }

  bool wrongWayReverseSafetyValid(
    const ReverseSafety & safety, const int turn_sign) const
  {
    return safety.valid &&
      reactive_control_v2::wrong_way_recovery::selectedReverseSideClear(
      turn_sign, safety.left_valid_ratio, safety.right_valid_ratio,
      reverse_side_min_valid_ratio_, safety.left_clearance_m,
      safety.right_clearance_m, wrong_way_reverse_min_side_clearance_m_);
  }

  bool reverseSweptSafetyValid(
    const nav_msgs::msg::OccupancyGrid::SharedPtr & map,
    const std::vector<reactive_control_v2::reverse_swept_safety::Actor> & actors,
    const reactive_control_v2::reverse_swept_safety::Pose2 & simulator_ego_pose,
    const bool simulator_ego_pose_valid,
    const reactive_control_v2::reverse_swept_safety::Pose2 & odom_pose,
    const std::string & odom_frame, const bool reverse_map_received,
    const bool simulator_status_received, const double odom_age,
    const double simulator_status_age, const int64_t odom_source_stamp_ns,
    const int64_t simulator_status_source_stamp_ns,
    const int64_t scan_source_stamp_ns, const double steering_rad,
    const double remaining_distance_m)
  {
    if (!enable_sim_reverse_swept_gate_) {
      reverse_swept_gate_valid_ = true;
      reverse_swept_gate_reason_ = "simulator reverse swept gate disabled";
      reverse_swept_checked_distance_m_ = 0.0;
      return true;
    }
    const double status_scan_skew_sec =
      (scan_source_stamp_ns > 0 && simulator_status_source_stamp_ns > 0) ?
      std::abs(static_cast<double>(
        scan_source_stamp_ns - simulator_status_source_stamp_ns)) * 1e-9 :
      std::numeric_limits<double>::infinity();
    const double odom_scan_skew_sec =
      (scan_source_stamp_ns > 0 && odom_source_stamp_ns > 0) ?
      std::abs(static_cast<double>(scan_source_stamp_ns - odom_source_stamp_ns)) * 1e-9 :
      std::numeric_limits<double>::infinity();
    if (!map || !simulator_ego_pose_valid || !reverse_map_received ||
      !simulator_status_received || odom_age < 0.0 ||
      odom_age > reverse_swept_status_timeout_sec_ || simulator_status_age < 0.0 ||
      simulator_status_age > reverse_swept_status_timeout_sec_ ||
      status_scan_skew_sec > reverse_swept_status_timeout_sec_ ||
      odom_scan_skew_sec > reverse_swept_status_timeout_sec_ ||
      !reactive_control_v2::reverse_swept_safety::finitePose(odom_pose) ||
      odom_frame.empty() || map->header.frame_id != odom_frame)
    {
      reverse_swept_gate_valid_ = false;
      reverse_swept_gate_reason_ = "reverse swept map, odometry, or agent state unavailable";
      reverse_swept_checked_distance_m_ = 0.0;
      return false;
    }
    if (std::hypot(
        odom_pose.x - simulator_ego_pose.x,
        odom_pose.y - simulator_ego_pose.y) > 0.15 ||
      std::abs(wrappedAngleDifference(odom_pose.yaw, simulator_ego_pose.yaw)) > 0.20)
    {
      reverse_swept_gate_valid_ = false;
      reverse_swept_gate_reason_ = "simulator ego pose disagrees with odometry";
      reverse_swept_checked_distance_m_ = 0.0;
      return false;
    }

    reactive_control_v2::reverse_swept_safety::Input input;
    input.map.valid = true;
    input.map.frame = map->header.frame_id;
    input.map.width = map->info.width;
    input.map.height = map->info.height;
    input.map.resolution = map->info.resolution;
    input.map.origin.x = map->info.origin.position.x;
    input.map.origin.y = map->info.origin.position.y;
    input.map.origin.yaw = quaternionYaw(map->info.origin.orientation);
    input.map.cells = &map->data;
    input.map.occupied_threshold = reverse_map_occupied_threshold_;
    input.ego = odom_pose;
    input.actors = actors;
    input.steering_rad = steering_rad;
    input.wheelbase_m = reverse_wheelbase_m_;
    input.reverse_speed_mps = reverse_speed_mps_;
    input.arc_distance_m = std::max(reverse_swept_sample_step_m_, remaining_distance_m);
    input.sample_step_m = reverse_swept_sample_step_m_;
    input.vehicle_length_m = reverse_vehicle_length_m_;
    input.vehicle_width_m = reverse_vehicle_width_m_;
    input.clearance_margin_m = reverse_swept_margin_m_;
    const auto result = reactive_control_v2::reverse_swept_safety::
      evaluateSteeringTransition(input, last_published_steering_rad_);
    reverse_swept_gate_valid_ = result.valid;
    reverse_swept_gate_reason_ = result.reason;
    reverse_swept_checked_distance_m_ = result.checked_distance_m;
    return result.valid;
  }

  void startWrongWayReverse(
    const rclcpp::Time & current_time, const double heading_error_rad,
    const ReverseSafety & safety)
  {
    startReverse(current_time, "confirmed reverse track heading; steering reverse toward alignment");
    wrong_way_reverse_active_ = true;
    ++wrong_way_reverse_attempt_count_;
    wrong_way_reverse_attempt_committed_ = true;
    wrong_way_turn_sign_ = reactive_control_v2::wrong_way_recovery::retainEpisodeTurnSign(
      wrong_way_turn_sign_, heading_error_rad,
      safety.left_clearance_m, safety.right_clearance_m,
      degreesToRadians(170.0));
  }

  void startRecoverySettle(const rclcpp::Time & current_time, const std::string & reason)
  {
    reverse_active_ = false;
    wrong_way_reverse_active_ = false;
    wrong_way_reverse_attempt_committed_ = false;
    settle_active_ = true;
    settle_stationary_since_ = current_time;
    settle_stationary_timer_active_ = false;
    settle_reason_ = reason;
    resetLowSpeedAssist();
    clearTriggerTimers();
  }

  ackermann_msgs::msg::AckermannDriveStamped reverseCommand(
    const double steering_angle = 0.0) const
  {
    ackermann_msgs::msg::AckermannDriveStamped output;
    output.header.stamp = now();
    output.header.frame_id = "base_link";
    output.drive.speed = -std::min(reverse_speed_mps_, absolute_speed_limit_mps_);
    output.drive.steering_angle = clampValue(
      steering_angle, -absolute_steering_limit_rad_, absolute_steering_limit_rad_);
    return output;
  }

  void logFallbackDebug(
    const Mode mode, const ScanData & scan, const FallbackResult & fallback,
    const ackermann_msgs::msg::AckermannDriveStamped & output)
  {
    if (!fallback_terminal_debug_) {
      return;
    }
    const rclcpp::Time current_time = steady_clock_.now();
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

  void logReverseDebug(
    const Mode mode, const double current_speed, const bool odom_healthy,
    const double odom_age,
    const bool front_emergency, const bool ftg_stably_available,
    const ReverseSafety & reverse_safety)
  {
    if (!reverse_terminal_debug_ ||
      (mode != Mode::EMERGENCY_STOP && mode != Mode::REVERSE_RECOVERY &&
      mode != Mode::RECOVERY_SETTLE))
    {
      return;
    }
    const rclcpp::Time current_time = steady_clock_.now();
    if (last_reverse_debug_time_.nanoseconds() != 0 &&
      (current_time - last_reverse_debug_time_).seconds() < reverse_terminal_debug_period_sec_)
    {
      return;
    }
    last_reverse_debug_time_ = current_time;
    const double reverse_duration = reverse_active_ ?
      std::max(0.0, (current_time - reverse_started_at_).seconds()) : 0.0;
    const double reverse_steering_deg = wrong_way_reverse_active_ ?
      reactive_control_v2::wrong_way_recovery::reverseSteeringAngle(
      wrong_way_turn_sign_, wrong_way_reverse_steering_rad_) * 180.0 / kPi : 0.0;
    RCLCPP_INFO(
      get_logger(),
      "reverse debug | mode=%s attempt=%d/%d trigger=%s | "
      "speed=%+.3f m/s odom_ok=%s age=%.3f s distance=%.3f m duration=%.2f s "
      "steer=%+.1f deg wrong_way=%s | assist=%s requested=%+.2f output=%+.2f shortfall=%.2f m/s | "
      "dead_end_timer=%s %.2f/%.2f s | "
      "front_emergency=%s ftg_valid=%d/%d stable=%s | side_safe=%s ratio=%.2f min=%.2f m",
      modeName(mode), reverse_attempt_count_, reverse_max_attempts_,
      reverse_trigger_reason_.c_str(), current_speed, odom_healthy ? "true" : "false",
      odom_age, reverse_distance_m_, reverse_duration, reverse_steering_deg,
      wrong_way_reverse_active_ ? "true" : "false",
      low_speed_assist_active_ ? "true" : "false",
      low_speed_assist_requested_mps_, low_speed_assist_output_command_mps_,
      low_speed_assist_shortfall_mps_,
      dead_end_timer_active_ ? "active" : "inactive",
      dead_end_timer_active_ ? std::max(0.0, (current_time - dead_end_started_at_).seconds()) :
      0.0, dead_end_confirmation_sec_,
      front_emergency ? "true" : "false", ftg_recovery_valid_count_,
      ftg_recovery_valid_cycles_, ftg_stably_available ? "true" : "false",
      reverse_safety.valid ? "true" : "false", reverse_safety.valid_ratio,
      reverse_safety.minimum_clearance_m);
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
      case Mode::REVERSE_RECOVERY: return "REVERSE_RECOVERY";
      case Mode::RECOVERY_SETTLE: return "RECOVERY_SETTLE";
    }
    return "UNKNOWN";
  }

  void publishStatus(
    const Mode mode, const std::string & reason, const ScanData & scan,
    const FallbackResult & fallback, const double command_age,
    const double scan_age, const double current_speed, const std::string & upper_state,
    const bool odom_healthy, const double odom_age, const bool front_emergency,
    const bool ftg_stably_available, const ReverseSafety & reverse_safety,
    const double heading_error_rad, const double heading_error_age,
    const bool heading_error_fresh,
    const uint8_t arbitration_mode, const bool arbitration_mode_healthy,
    const double arbitration_mode_age,
    const rclcpp::Time & steady_now)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "reactive_control_v2/lower_safety_controller";
    status.hardware_id = "final_command_gateway";
    status.level = (mode == Mode::EMERGENCY_STOP || mode == Mode::REVERSE_RECOVERY ||
      mode == Mode::RECOVERY_SETTLE) ?
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
    add("arbitration_mode_required", require_arbitration_mode_ ? "true" : "false");
    add("arbitration_mode", std::to_string(arbitration_mode));
    add("arbitration_mode_healthy", arbitration_mode_healthy ? "true" : "false");
    add("arbitration_mode_age_sec", std::to_string(arbitration_mode_age));
    add("upper_state", upper_state.empty() ? "unavailable" : upper_state);
    add("command_age_sec", std::to_string(command_age));
    add("scan_age_sec", std::to_string(scan_age));
    add("scan_valid_ratio", std::to_string(scan.valid_ratio));
    add(
      "front_min_distance_m",
      std::isfinite(scan.front_min_distance_m) ?
      std::to_string(scan.front_min_distance_m) : "inf");
    add("current_speed_mps", std::to_string(current_speed));
    add("odom_topic", odom_topic_);
    add("odom_age_sec", std::to_string(odom_age));
    add("odom_healthy", odom_healthy ? "true" : "false");
    add(
      "stationary_for_reverse",
      (odom_healthy && std::abs(current_speed) <= stationary_speed_threshold_mps_) ?
      "true" : "false");
    add("front_emergency_active", front_emergency ? "true" : "false");
    add("fallback_available", fallback.valid ? "true" : "false");
    add("fallback_recovery_valid_count", std::to_string(ftg_recovery_valid_count_));
    add("fallback_stably_available", ftg_stably_available ? "true" : "false");
    add("fallback_target_angle_rad", std::to_string(fallback.target_angle));
    add("fallback_target_range_m", std::to_string(fallback.target_range));
    add("fallback_gap_width_deg", std::to_string(fallback.gap_width_deg));
    add("fallback_gap_start_angle_deg", std::to_string(fallback.gap_start_angle_deg));
    add("fallback_gap_end_angle_deg", std::to_string(fallback.gap_end_angle_deg));
    add("fallback_gap_mean_depth_m", std::to_string(fallback.gap_mean_depth_m));
    add("fallback_gap_score", std::to_string(fallback.gap_score));
    add("fallback_terminal_debug_enabled", fallback_terminal_debug_ ? "true" : "false");
    add("low_speed_assist_enabled", enable_low_speed_assist_ ? "true" : "false");
    add("low_speed_assist_active", low_speed_assist_active_ ? "true" : "false");
    add("low_speed_assist_requested_mps", std::to_string(low_speed_assist_requested_mps_));
    add(
      "low_speed_assist_output_command_mps",
      std::to_string(low_speed_assist_output_command_mps_));
    add(
      "low_speed_assist_actual_magnitude_mps",
      std::to_string(low_speed_assist_actual_magnitude_mps_));
    add("low_speed_assist_shortfall_mps", std::to_string(low_speed_assist_shortfall_mps_));
    add("reverse_recovery_enabled", enable_reverse_recovery_ ? "true" : "false");
    add(
      "raceline_stall_handoff_enabled",
      enable_raceline_stall_handoff_ ? "true" : "false");
    add(
      "raceline_stall_handoff_evidence",
      raceline_stall_handoff_evidence_ ? "true" : "false");
    add(
      "raceline_stall_handoff_active",
      raceline_stall_handoff_active_ ? "true" : "false");
    add(
      "raceline_stall_handoff_latched",
      raceline_stall_handoff_latched_ ? "true" : "false");
    add("wrong_way_recovery_enabled", enable_wrong_way_recovery_ ? "true" : "false");
    add("wrong_way_heading_error_topic", wrong_way_heading_error_topic_);
    add("wrong_way_heading_error_age_sec", std::to_string(heading_error_age));
    add("wrong_way_heading_error_fresh", heading_error_fresh ? "true" : "false");
    add("wrong_way_heading_source_stamp_ns", std::to_string(heading_error_source_stamp_ns_));
    add("wrong_way_heading_frame", current_heading_error_frame_);
    add("wrong_way_raceline_distance_m", std::to_string(current_raceline_distance_m_));
    add("wrong_way_raceline_station_m", std::to_string(current_raceline_station_m_));
    add(
      "wrong_way_raceline_association_valid",
      (std::isfinite(current_raceline_distance_m_) &&
      current_raceline_distance_m_ >= 0.0 &&
      current_raceline_distance_m_ <= wrong_way_max_raceline_distance_m_) ?
      "true" : "false");
    add(
      "wrong_way_heading_error_deg",
      std::isfinite(heading_error_rad) ?
      std::to_string(heading_error_rad * 180.0 / kPi) : "nan");
    add("wrong_way_evidence", wrong_way_evidence_ ? "true" : "false");
    add("wrong_way_suspect_latched", wrong_way_suspect_latched_ ? "true" : "false");
    add("wrong_way_recovery_latched", wrong_way_recovery_latched_ ? "true" : "false");
    add("wrong_way_reverse_active", wrong_way_reverse_active_ ? "true" : "false");
    add(
      "wrong_way_alignment_recovered",
      wrong_way_alignment_recovered_ ? "true" : "false");
    add("wrong_way_reverse_attempt_count", std::to_string(wrong_way_reverse_attempt_count_));
    add(
      "wrong_way_reverse_attempt_committed",
      wrong_way_reverse_attempt_committed_ ? "true" : "false");
    add("wrong_way_turn_sign", std::to_string(wrong_way_turn_sign_));
    add(
      "wrong_way_confirmation_samples",
      std::to_string(wrong_way_confirmation_sample_count_));
    add(
      "wrong_way_confirmation_samples_required",
      std::to_string(wrong_way_confirmation_samples_));
    add(
      "wrong_way_alignment_confirmation_samples",
      std::to_string(wrong_way_alignment_sample_count_));
    add(
      "wrong_way_alignment_confirmation_samples_required",
      std::to_string(wrong_way_exit_confirmation_samples_));
    add("reverse_swept_gate_enabled", enable_sim_reverse_swept_gate_ ? "true" : "false");
    add("reverse_swept_gate_valid", reverse_swept_gate_valid_ ? "true" : "false");
    add("reverse_swept_gate_reason", reverse_swept_gate_reason_);
    add(
      "reverse_swept_checked_distance_m",
      std::to_string(reverse_swept_checked_distance_m_));
    add(
      "wrong_way_confirmation_sec",
      wrong_way_timer_active_ ?
      std::to_string(std::max(0.0, (steady_now - wrong_way_started_at_).seconds())) :
      "0.000000");
    add("dead_end_timer_active", dead_end_timer_active_ ? "true" : "false");
    add(
      "dead_end_timer_sec",
      dead_end_timer_active_ ?
      std::to_string(std::max(0.0, (steady_now - dead_end_started_at_).seconds())) :
      "0.000000");
    add("stuck_timer_active", stuck_timer_active_ ? "true" : "false");
    add(
      "stuck_timer_sec",
      stuck_timer_active_ ?
      std::to_string(std::max(0.0, (steady_now - stuck_started_at_).seconds())) :
      "0.000000");
    add("reverse_attempt_count", std::to_string(reverse_attempt_count_));
    add("reverse_distance_m", std::to_string(reverse_distance_m_));
    add(
      "reverse_duration_sec",
      reverse_active_ ?
      std::to_string(std::max(0.0, (steady_now - reverse_started_at_).seconds())) :
      "0.000000");
    add("reverse_trigger", reverse_trigger_reason_);
    add("reverse_side_safe", reverse_safety.valid ? "true" : "false");
    add("reverse_side_reason", reverse_safety.reason);
    add("reverse_side_valid_ratio", std::to_string(reverse_safety.valid_ratio));
    add(
      "reverse_side_min_clearance_m",
      std::isfinite(reverse_safety.minimum_clearance_m) ?
      std::to_string(reverse_safety.minimum_clearance_m) : "inf");
    add(
      "reverse_left_clearance_m",
      std::isfinite(reverse_safety.left_clearance_m) ?
      std::to_string(reverse_safety.left_clearance_m) : "inf");
    add(
      "reverse_right_clearance_m",
      std::isfinite(reverse_safety.right_clearance_m) ?
      std::to_string(reverse_safety.right_clearance_m) : "inf");
    const double stop_duration = stopped_timer_active_ ?
      std::max(0.0, (steady_now - stopped_since_).seconds()) : 0.0;
    add("stop_duration_sec", std::to_string(stop_duration));
    array.status.push_back(status);
    status_pub_->publish(array);
  }

  void logTransition(const Mode mode, const std::string & reason)
  {
    if (mode == last_mode_ && reason == last_reason_) {
      return;
    }
    if (mode == Mode::EMERGENCY_STOP || mode == Mode::RECOVERY_SETTLE) {
      RCLCPP_WARN(get_logger(), "mode=%s -> STOP | reason=%s", modeName(mode), reason.c_str());
    } else if (mode == Mode::REVERSE_RECOVERY) {
      RCLCPP_WARN(get_logger(), "mode=%s | reason=%s", modeName(mode), reason.c_str());
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
    nav_msgs::msg::OccupancyGrid::SharedPtr reverse_map;
    std::vector<reactive_control_v2::reverse_swept_safety::Actor> simulator_actors;
    reactive_control_v2::reverse_swept_safety::Pose2 simulator_ego_pose;
    reactive_control_v2::reverse_swept_safety::Pose2 odom_pose;
    std::string odom_frame;
    rclcpp::Time command_time(0, 0, RCL_STEADY_TIME);
    rclcpp::Time scan_time(0, 0, RCL_STEADY_TIME);
    rclcpp::Time status_time(0, 0, RCL_STEADY_TIME);
    rclcpp::Time arbitration_mode_time(0, 0, RCL_STEADY_TIME);
    rclcpp::Time heading_error_time(0, 0, RCL_STEADY_TIME);
    rclcpp::Time simulator_status_time(0, 0, RCL_STEADY_TIME);
    rclcpp::Time odom_time(0, 0, RCL_STEADY_TIME);
    bool command_received = false;
    bool scan_received = false;
    bool status_received = false;
    bool arbitration_mode_received = false;
    bool heading_error_received = false;
    bool heading_error_frame_valid = false;
    bool simulator_ego_pose_valid = false;
    bool reverse_map_received = false;
    bool simulator_status_received = false;
    bool odom_received = false;
    double current_speed = 0.0;
    std::string upper_state;
    uint8_t arbitration_mode = 0;
    double heading_error_rad = 0.0;
    double raceline_distance_m = std::numeric_limits<double>::infinity();
    double raceline_station_m = std::numeric_limits<double>::quiet_NaN();
    std::string heading_error_frame;
    int64_t heading_error_source_stamp_ns = 0;
    int64_t odom_source_stamp_ns = 0;
    int64_t simulator_status_source_stamp_ns = 0;
    uint64_t heading_error_sequence = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command = latest_command_;
      scan = latest_scan_;
      reverse_map = latest_reverse_map_;
      simulator_actors = simulator_actors_;
      simulator_ego_pose = simulator_ego_pose_;
      odom_pose = current_odom_pose_;
      odom_frame = current_odom_frame_;
      command_time = last_command_time_;
      scan_time = last_scan_time_;
      status_time = last_upper_status_time_;
      arbitration_mode_time = last_arbitration_mode_time_;
      heading_error_time = last_heading_error_time_;
      simulator_status_time = last_simulator_status_time_;
      odom_time = last_odom_time_;
      command_received = command_received_;
      scan_received = scan_received_;
      status_received = upper_status_received_;
      arbitration_mode_received = arbitration_mode_received_;
      heading_error_received = heading_error_received_;
      heading_error_frame_valid = heading_error_frame_valid_;
      simulator_ego_pose_valid = simulator_ego_pose_valid_;
      reverse_map_received = reverse_map_received_;
      simulator_status_received = simulator_status_received_;
      odom_received = odom_received_;
      current_speed = current_speed_mps_;
      upper_state = upper_state_;
      arbitration_mode = arbitration_mode_;
      heading_error_rad = current_heading_error_rad_;
      raceline_distance_m = current_raceline_distance_m_;
      raceline_station_m = current_raceline_station_m_;
      heading_error_frame = current_heading_error_frame_;
      heading_error_source_stamp_ns = heading_error_source_stamp_ns_;
      odom_source_stamp_ns = odom_source_stamp_ns_;
      simulator_status_source_stamp_ns = simulator_status_source_stamp_ns_;
      heading_error_sequence = heading_error_sequence_;
    }

    const rclcpp::Time current_time = steady_clock_.now();
    const double command_age = command_received ?
      (current_time - command_time).seconds() : std::numeric_limits<double>::infinity();
    const double scan_age = scan_received ?
      (current_time - scan_time).seconds() : std::numeric_limits<double>::infinity();
    const double status_age = status_received ?
      (current_time - status_time).seconds() : std::numeric_limits<double>::infinity();
    const double arbitration_mode_age = arbitration_mode_received ?
      (current_time - arbitration_mode_time).seconds() :
      std::numeric_limits<double>::infinity();
    const bool arbitration_mode_healthy = !require_arbitration_mode_ ||
      (arbitration_mode_received &&
      arbitration_mode_age <= arbitration_mode_timeout_sec_ && arbitration_mode <= 3);
    const bool reactive_mode_authorized = !require_arbitration_mode_ ||
      (arbitration_mode_healthy && arbitration_mode == 2);
    const bool selected_command_authorized = !require_arbitration_mode_ ||
      (arbitration_mode_healthy && (arbitration_mode == 1 || arbitration_mode == 2));
    const double heading_error_age = heading_error_received ?
      (current_time - heading_error_time).seconds() :
      std::numeric_limits<double>::infinity();
    const double simulator_status_age = simulator_status_received ?
      (current_time - simulator_status_time).seconds() :
      std::numeric_limits<double>::infinity();
    const int64_t scan_source_stamp_ns = scan ?
      sourceStampNanoseconds(scan->header.stamp) : 0;
    const double heading_scan_skew_sec =
      (scan_source_stamp_ns > 0 && heading_error_source_stamp_ns > 0) ?
      static_cast<double>(scan_source_stamp_ns - heading_error_source_stamp_ns) * 1e-9 :
      std::numeric_limits<double>::infinity();
    const bool heading_scan_aligned = heading_scan_skew_sec >= 0.0 &&
      heading_scan_skew_sec <= wrong_way_heading_error_timeout_sec_;
    const bool raceline_association_valid = heading_error_frame_valid &&
      !odom_frame.empty() && heading_error_frame == normalizedFrameId(odom_frame) &&
      std::isfinite(raceline_distance_m) && raceline_distance_m >= 0.0 &&
      raceline_distance_m <= wrong_way_max_raceline_distance_m_ &&
      std::isfinite(raceline_station_m);
    const bool heading_error_fresh = heading_error_received &&
      heading_error_age >= 0.0 &&
      heading_error_age <= wrong_way_heading_error_timeout_sec_ &&
      heading_scan_aligned && std::isfinite(heading_error_rad) &&
      std::abs(heading_error_rad) <= kPi + 1e-6;

    if (require_arbitration_mode_) {
      const bool continuity_broken =
        reactive_control_v2::raceline_stall_handoff::modeContinuityBroken(
        control_arbitration_mode_valid_, last_control_arbitration_mode_,
        arbitration_mode_healthy, arbitration_mode);
      if (continuity_broken) {
        stuck_timer_active_ = false;
        wrong_way_timer_active_ = false;
        wrong_way_confirmation_sample_count_ = 0;
      }
      control_arbitration_mode_valid_ = arbitration_mode_healthy;
      if (arbitration_mode_healthy) {
        last_control_arbitration_mode_ = arbitration_mode;
      }
    }
    raceline_stall_handoff_evidence_ = false;
    raceline_stall_handoff_active_ = false;

    if (!reactive_mode_authorized && (reverse_active_ || settle_active_)) {
      cancelReactiveRecovery();
    }
    const double odom_age = odom_received ?
      (current_time - odom_time).seconds() : std::numeric_limits<double>::infinity();
    const bool odom_healthy = odom_received && odom_age <= odom_timeout_sec_ &&
      std::isfinite(current_speed);
    if (!odom_healthy) {
      // Nominal and ordinary FTG operation retain the previous behavior: stale
      // odometry only disables TTC speed feedback. Reverse entry and motion,
      // however, require fresh finite VESC-derived odometry.
      current_speed = 0.0;
    }

    ScanData scan_data;
    ReverseSafety reverse_safety;
    if (scan_received && scan_age <= scan_timeout_sec_ && scan) {
      scan_data = preprocessScan(*scan);
      reverse_safety = evaluateReverseSafety(*scan);
    } else {
      scan_data.invalid_reason = scan_received ? "LaserScan timeout" : "no LaserScan received";
      reverse_safety.reason = scan_data.invalid_reason;
    }
    const FallbackResult fallback = makeFallback(scan_data);

    const bool command_fresh = command_received && command_age <= command_timeout_sec_;
    const bool command_valid = commandNumericallyValid(command);
    const bool explicit_fallback = reactive_mode_authorized && upperRequestsFallback(
      status_received, status_age, upper_state);
    const double nominal_forward_speed = command_valid ?
      std::max(0.0, static_cast<double>(command->drive.speed)) : 0.0;
    const bool selected_command_would_be_used = selected_command_authorized &&
      command_fresh && command_valid && !explicit_fallback;
    const bool fallback_would_be_used = reactive_mode_authorized && fallback.valid &&
      (explicit_fallback || !command_fresh || !command_valid);
    const bool selected_forward_intent = selected_command_would_be_used &&
      nominal_forward_speed > 0.0;
    const bool fallback_forward_intent = fallback_would_be_used && fallback.speed > 0.0;
    const bool pre_safety_forward_intent =
      selected_forward_intent || fallback_forward_intent;
    const double normal_forward_reference = std::max(
      std::max(0.0, current_speed), nominal_forward_speed);
    std::string front_emergency_reason;
    bool front_emergency = scan_data.valid && frontEmergencyActive(
      scan_data, normal_forward_reference, &front_emergency_reason);
    const double fallback_forward_reference = fallback.valid ?
      std::max(std::max(0.0, current_speed), fallback.speed) : 0.0;
    const bool fallback_front_emergency = scan_data.valid && fallback.valid &&
      frontEmergencyActive(scan_data, fallback_forward_reference);
    const bool ftg_recovery_candidate = scan_data.valid && fallback.valid &&
      !fallback_front_emergency;
    if (ftg_recovery_candidate) {
      ftg_recovery_valid_count_ = std::min(
        ftg_recovery_valid_count_ + 1, ftg_recovery_valid_cycles_);
    } else {
      ftg_recovery_valid_count_ = 0;
    }
    bool ftg_stably_available =
      ftg_recovery_valid_count_ >= ftg_recovery_valid_cycles_;

    // First compute the unchanged forward-only decision. Recovery may replace
    // it below, but stuck detection deliberately compares this meaningful
    // forward request against VESC-reported speed rather than against /drive.
    Mode base_mode = Mode::EMERGENCY_STOP;
    std::string base_reason;
    ackermann_msgs::msg::AckermannDriveStamped base_output;
    if (!arbitration_mode_healthy) {
      base_reason = "arbitration selected-mode heartbeat unavailable or stale";
      base_output = stopCommand();
    } else if (!selected_command_authorized) {
      base_reason = "arbitration selected WAITING or STOP mode";
      base_output = stopCommand();
    } else if (!scan_data.valid) {
      base_reason = scan_data.invalid_reason;
      base_output = stopCommand();
    } else if (front_emergency) {
      base_reason = front_emergency_reason;
      base_output = stopCommand();
    } else if (command_fresh && command_valid && !explicit_fallback) {
      base_mode = Mode::NOMINAL;
      base_reason = "fresh finite selected command passed through";
      base_output = boundedNominal(*command);
    } else if (reactive_mode_authorized && fallback.valid) {
      base_mode = Mode::FALLBACK_FTG;
      if (explicit_fallback) {
        base_reason = "upper state " + upper_state + "; using conservative FTG";
      } else if (!command_received || !command_fresh) {
        base_reason = "selected command unavailable or stale; using conservative FTG";
      } else {
        base_reason = "selected command contains non-finite value; using conservative FTG";
      }
      base_output = fallbackCommand(fallback);
    } else {
      if (!reactive_mode_authorized) {
        base_reason = "selected raceline command unavailable; reactive fallback not authorized";
      } else {
        base_reason = fallback.reason.empty() ?
          "no valid nominal or fallback command" : fallback.reason;
      }
      base_output = stopCommand();
    }

    if (!reverse_active_ && !settle_active_) {
      const bool assist_allowed = odom_healthy && scan_data.valid &&
        (base_mode == Mode::NOMINAL || base_mode == Mode::FALLBACK_FTG);
      applyLowSpeedAssist(base_output, current_speed, assist_allowed, current_time);

      // Assistance may raise a forward command, so repeat the independent
      // emergency/TTC check using the speed that would actually be published.
      if (low_speed_assist_active_ && base_output.drive.speed > 0.0 &&
        frontEmergencyActive(
          scan_data, std::max(std::max(0.0, current_speed), static_cast<double>(base_output.drive.speed)),
          &front_emergency_reason))
      {
        front_emergency = true;
        ftg_recovery_valid_count_ = 0;
        ftg_stably_available = false;
        base_mode = Mode::EMERGENCY_STOP;
        base_reason = front_emergency_reason;
        base_output = stopCommand();
        resetLowSpeedAssist();
      }
    } else if (settle_active_) {
      resetLowSpeedAssist();
    }

    Mode mode = base_mode;
    std::string reason = base_reason;
    ackermann_msgs::msg::AckermannDriveStamped output = base_output;

    reactive_control_v2::wrong_way_recovery::Evidence wrong_way_input;
    wrong_way_input.enabled = enable_wrong_way_recovery_ && enable_reverse_recovery_;
    wrong_way_input.arbitration_required = require_arbitration_mode_;
    wrong_way_input.arbitration_mode_healthy = arbitration_mode_healthy;
    wrong_way_input.arbitration_mode = arbitration_mode;
    wrong_way_input.heading_sample_fresh = heading_error_fresh;
    wrong_way_input.path_association_valid = raceline_association_valid;
    wrong_way_input.scan_valid = scan_data.valid;
    wrong_way_input.odom_healthy = odom_healthy;
    wrong_way_input.positive_forward_request = pre_safety_forward_intent;
    wrong_way_input.heading_error_rad = heading_error_rad;
    wrong_way_input.entry_angle_rad = wrong_way_entry_angle_rad_;
    const bool wrong_way_geometry_valid =
      reactive_control_v2::wrong_way_recovery::currentGeometryValid(wrong_way_input);
    wrong_way_evidence_ = wrong_way_input.positive_forward_request &&
      reactive_control_v2::wrong_way_recovery::reverseHeadingPresent(
      wrong_way_input, wrong_way_stop_angle_rad_);
    const bool wrong_way_direction_unverified =
      reactive_control_v2::wrong_way_recovery::forwardDirectionUnverified(
      wrong_way_input);
    const bool raw_alignment_recovered =
      reactive_control_v2::wrong_way_recovery::alignmentRecovered(
      wrong_way_input, wrong_way_exit_angle_rad_);
    const bool new_heading_sample = heading_error_sequence != 0 &&
      heading_error_sequence != last_wrong_way_alignment_sequence_;
    if (new_heading_sample) {
      last_wrong_way_alignment_sequence_ = heading_error_sequence;
      if (raw_alignment_recovered) {
        if (!wrong_way_alignment_timer_active_) {
          wrong_way_alignment_started_at_ = current_time;
          wrong_way_alignment_timer_active_ = true;
          wrong_way_alignment_sample_count_ = 0;
        }
        ++wrong_way_alignment_sample_count_;
      } else {
        wrong_way_alignment_timer_active_ = false;
        wrong_way_alignment_sample_count_ = 0;
      }
    } else if (!wrong_way_geometry_valid)
    {
      wrong_way_alignment_timer_active_ = false;
      wrong_way_alignment_sample_count_ = 0;
    }
    const double wrong_way_alignment_elapsed_sec = wrong_way_alignment_timer_active_ ?
      std::max(0.0, (current_time - wrong_way_alignment_started_at_).seconds()) : 0.0;
    const bool alignment_confirmed = raw_alignment_recovered &&
      reactive_control_v2::wrong_way_recovery::confirmationReached(
      wrong_way_alignment_timer_active_, wrong_way_alignment_elapsed_sec,
      wrong_way_exit_confirmation_sec_, wrong_way_alignment_sample_count_,
      wrong_way_exit_confirmation_samples_);
    if (alignment_confirmed) {
      wrong_way_alignment_recovered_ = true;
    } else if (!wrong_way_geometry_valid ||
      std::abs(heading_error_rad) >= wrong_way_stop_angle_rad_)
    {
      wrong_way_alignment_recovered_ = false;
    }
    if (wrong_way_recovery_latched_) {
      wrong_way_suspect_latched_ = false;
    } else if (wrong_way_evidence_) {
      wrong_way_suspect_latched_ = true;
    } else if (wrong_way_suspect_latched_ && wrong_way_alignment_recovered_)
    {
      wrong_way_suspect_latched_ = false;
    }

    if (reverse_active_ && !wrong_way_reverse_active_ &&
      (wrong_way_suspect_latched_ || wrong_way_recovery_latched_))
    {
      startRecoverySettle(
        current_time, "ordinary reverse preempted by track-direction recovery");
    }
    const auto reverse_swept_clear_for = [&](const int turn_sign,
        const double remaining_distance_m) {
        return reverseSweptSafetyValid(
          reverse_map, simulator_actors, simulator_ego_pose,
          simulator_ego_pose_valid, odom_pose, odom_frame,
          reverse_map_received, simulator_status_received,
          odom_age, simulator_status_age, odom_source_stamp_ns,
          simulator_status_source_stamp_ns, scan_source_stamp_ns,
          reactive_control_v2::wrong_way_recovery::reverseSteeringAngle(
            turn_sign, wrong_way_reverse_steering_rad_),
          remaining_distance_m);
      };
    const auto select_safe_wrong_way_turn = [&](const double remaining_distance_m) {
        if (wrong_way_turn_sign_ == -1 || wrong_way_turn_sign_ == 1) {
          return wrongWayReverseSafetyValid(reverse_safety, wrong_way_turn_sign_) &&
                 reverse_swept_clear_for(wrong_way_turn_sign_, remaining_distance_m);
        }
        const int preferred_sign =
          reactive_control_v2::wrong_way_recovery::chooseTurnSign(
          heading_error_rad, reverse_safety.left_clearance_m,
          reverse_safety.right_clearance_m, degreesToRadians(170.0));
        const bool ambiguous = std::abs(heading_error_rad) >= degreesToRadians(170.0);
        const int candidate_count = ambiguous ? 2 : 1;
        for (int index = 0; index < candidate_count; ++index) {
          const int candidate_sign = index == 0 ? preferred_sign : -preferred_sign;
          if (wrongWayReverseSafetyValid(reverse_safety, candidate_sign) &&
            reverse_swept_clear_for(candidate_sign, remaining_distance_m))
          {
            wrong_way_turn_sign_ = candidate_sign;
            return true;
          }
        }
        return false;
      };

    if (reverse_active_) {
      const bool correcting_wrong_way = wrong_way_reverse_active_;
      const double update_dt = std::max(
        0.0, std::min(0.20, (current_time - reverse_last_update_at_).seconds()));
      reverse_last_update_at_ = current_time;
      if (odom_healthy) {
        reverse_distance_m_ += std::abs(current_speed) * update_dt;
      }
      const double reverse_duration = std::max(
        0.0, (current_time - reverse_started_at_).seconds());
      const bool minimum_reverse_satisfied =
        reverse_distance_m_ >= reverse_min_distance_m_ ||
        reverse_duration >= reverse_min_duration_sec_;
      const bool reverse_limit_reached =
        reverse_distance_m_ >= reverse_max_distance_m_ ||
        reverse_duration >= reverse_max_duration_sec_;

      if (!scan_data.valid || !odom_healthy ||
        (correcting_wrong_way &&
        !reactive_control_v2::wrong_way_recovery::reverseAuthorityValid(wrong_way_input)))
      {
        startRecoverySettle(current_time, "reverse aborted because scan or odometry became invalid");
      } else if (!reverse_safety.valid) {
        startRecoverySettle(current_time, "reverse aborted: " + reverse_safety.reason);
      } else if (correcting_wrong_way && !wrongWayReverseSafetyValid(
          reverse_safety, wrong_way_turn_sign_))
      {
        startRecoverySettle(
          current_time, "wrong-way reverse aborted because its selected rear-side sector is not clear");
      } else if (!reverse_swept_clear_for(
          correcting_wrong_way ? wrong_way_turn_sign_ : 0,
          std::max(0.0, reverse_max_distance_m_ - reverse_distance_m_)))
      {
        startRecoverySettle(
          current_time, "reverse swept gate blocked: " +
          reverse_swept_gate_reason_);
      } else if (correcting_wrong_way && minimum_reverse_satisfied &&
        wrong_way_alignment_recovered_)
      {
        startRecoverySettle(current_time, "track-relative heading recovered");
      } else if (!correcting_wrong_way && minimum_reverse_satisfied && ftg_stably_available) {
        startRecoverySettle(current_time, "forward FTG route recovered stably");
      } else if (reverse_limit_reached) {
        startRecoverySettle(
          current_time, correcting_wrong_way ?
          "wrong-way reverse segment reached its distance or duration limit" :
          "reverse distance or duration limit reached");
      }

      if (settle_active_) {
        mode = Mode::RECOVERY_SETTLE;
        reason = settle_reason_;
        output = stopCommand();
      } else {
        mode = Mode::REVERSE_RECOVERY;
        reason = reverse_trigger_reason_;
        const double steering = correcting_wrong_way ?
          reactive_control_v2::wrong_way_recovery::reverseSteeringAngle(
          wrong_way_turn_sign_, wrong_way_reverse_steering_rad_) : 0.0;
        output = reverseCommand(steering);
      }
    } else if (settle_active_) {
      mode = Mode::RECOVERY_SETTLE;
      reason = settle_reason_;
      output = stopCommand();
      if (odom_healthy && std::abs(current_speed) <= stationary_speed_threshold_mps_) {
        if (!settle_stationary_timer_active_) {
          settle_stationary_since_ = current_time;
          settle_stationary_timer_active_ = true;
        }
      } else {
        settle_stationary_timer_active_ = false;
      }
      const bool fully_settled = settle_stationary_timer_active_ &&
        (current_time - settle_stationary_since_).seconds() >= recovery_settle_time_sec_;
      if (fully_settled) {
        settle_active_ = false;
        settle_stationary_timer_active_ = false;
        const bool alignment_release_window =
          reactive_control_v2::wrong_way_recovery::alignmentReleaseWindow(
          wrong_way_recovery_latched_, wrong_way_alignment_recovered_,
          wrong_way_input, wrong_way_stop_angle_rad_);
        const bool wrong_way_correction_required =
          wrong_way_recovery_latched_ && !alignment_release_window;
        if (wrong_way_suspect_latched_ && !wrong_way_recovery_latched_) {
          mode = Mode::EMERGENCY_STOP;
          reason = "wrong-way suspicion held while confirmation restarts after reverse preemption";
          output = stopCommand();
        } else if (wrong_way_correction_required &&
          reactive_control_v2::wrong_way_recovery::reverseAuthorityValid(wrong_way_input) &&
          wrong_way_reverse_attempt_count_ < wrong_way_reverse_max_attempts_ &&
          reverse_attempt_count_ < reverse_max_attempts_)
        {
          if (select_safe_wrong_way_turn(reverse_max_distance_m_)) {
            startWrongWayReverse(current_time, heading_error_rad, reverse_safety);
            mode = Mode::REVERSE_RECOVERY;
            reason = reverse_trigger_reason_;
            output = reverseCommand(
              reactive_control_v2::wrong_way_recovery::reverseSteeringAngle(
                wrong_way_turn_sign_, wrong_way_reverse_steering_rad_));
          } else {
            mode = Mode::EMERGENCY_STOP;
            reason = "wrong-way recovery swept gate blocked: " +
              reverse_swept_gate_reason_;
            output = stopCommand();
          }
        } else if (wrong_way_correction_required) {
          mode = Mode::EMERGENCY_STOP;
          if (!reactive_control_v2::wrong_way_recovery::reverseAuthorityValid(
              wrong_way_input))
          {
            reason = "wrong-way recovery waiting for explicit Reactive authority";
          } else if (wrong_way_reverse_attempt_count_ >= wrong_way_reverse_max_attempts_ ||
            reverse_attempt_count_ >= reverse_max_attempts_)
          {
            reason = "wrong-way reverse attempt limit reached";
          } else {
            reason = "wrong-way recovery held because rear-side clearance is unavailable";
          }
          output = stopCommand();
        } else if (wrong_way_recovery_latched_ && !wrong_way_alignment_recovered_) {
          mode = Mode::EMERGENCY_STOP;
          reason = "wrong-way recovery held because heading evidence is stale or invalid";
          output = stopCommand();
        } else if (ftg_stably_available) {
          mode = Mode::FALLBACK_FTG;
          reason = wrong_way_recovery_latched_ ?
            "track-relative heading recovered; resuming through stable FTG" :
            "reverse complete and stationary; resuming through stable FTG";
          output = fallbackCommand(fallback);
          applyLowSpeedAssist(output, current_speed, odom_healthy, current_time);
        } else {
          mode = Mode::EMERGENCY_STOP;
          reason = "reverse complete but no stable forward FTG route";
          output = stopCommand();
        }
      }
    } else {
      const bool new_confirmation_sample = heading_error_sequence != 0 &&
        heading_error_sequence != last_wrong_way_heading_sequence_;
      if (!wrong_way_suspect_latched_ && !wrong_way_recovery_latched_) {
        wrong_way_timer_active_ = false;
        wrong_way_confirmation_sample_count_ = 0;
      } else if (!wrong_way_geometry_valid) {
        wrong_way_timer_active_ = false;
        wrong_way_confirmation_sample_count_ = 0;
      } else if (new_confirmation_sample) {
        last_wrong_way_heading_sequence_ = heading_error_sequence;
        const bool confirmation_present =
          reactive_control_v2::wrong_way_recovery::suspicionConfirmationPresent(
          wrong_way_suspect_latched_, wrong_way_input, wrong_way_exit_angle_rad_);
        if (confirmation_present && !wrong_way_recovery_latched_) {
          if (!wrong_way_timer_active_) {
            wrong_way_started_at_ = current_time;
            wrong_way_timer_active_ = true;
            wrong_way_confirmation_sample_count_ = 0;
          }
          ++wrong_way_confirmation_sample_count_;
        } else if (!wrong_way_recovery_latched_) {
          wrong_way_timer_active_ = false;
          wrong_way_confirmation_sample_count_ = 0;
        }
      }
      const double wrong_way_elapsed_sec = wrong_way_timer_active_ ?
        std::max(0.0, (current_time - wrong_way_started_at_).seconds()) : 0.0;
      const bool wrong_way_confirmed = wrong_way_suspect_latched_ &&
        reactive_control_v2::wrong_way_recovery::suspicionConfirmationPresent(
        wrong_way_suspect_latched_, wrong_way_input, wrong_way_exit_angle_rad_) &&
        reactive_control_v2::wrong_way_recovery::confirmationReached(
        wrong_way_timer_active_, wrong_way_elapsed_sec, wrong_way_confirmation_sec_,
        wrong_way_confirmation_sample_count_, wrong_way_confirmation_samples_);
      if (wrong_way_confirmed) {
        wrong_way_recovery_latched_ =
          reactive_control_v2::wrong_way_recovery::updatePersistentLatch(
          wrong_way_recovery_latched_, true, false, false);
        raceline_stall_handoff_latched_ =
          reactive_control_v2::raceline_stall_handoff::updatePersistentLatch(
          raceline_stall_handoff_latched_, true, false);
        wrong_way_timer_active_ = false;
        wrong_way_confirmation_sample_count_ = 0;
        wrong_way_suspect_latched_ = false;
      }

      const bool alignment_release_window =
        reactive_control_v2::wrong_way_recovery::alignmentReleaseWindow(
        wrong_way_recovery_latched_, wrong_way_alignment_recovered_,
        wrong_way_input, wrong_way_stop_angle_rad_);
      const bool wrong_way_control_owns = wrong_way_suspect_latched_ ||
        (wrong_way_recovery_latched_ && !alignment_release_window) ||
        wrong_way_direction_unverified;
      if (wrong_way_control_owns) {
        clearTriggerTimers();
        resetLowSpeedAssist();
        const bool stationary = odom_healthy &&
          std::abs(current_speed) <= stationary_speed_threshold_mps_;
        if (wrong_way_direction_unverified && !wrong_way_suspect_latched_ &&
          !wrong_way_recovery_latched_)
        {
          mode = Mode::EMERGENCY_STOP;
          reason = "forward motion held until fresh track-direction geometry is available";
          output = stopCommand();
        } else if (!wrong_way_recovery_latched_) {
          mode = Mode::EMERGENCY_STOP;
          reason = wrong_way_geometry_valid ?
            "negative track progress detected; braking for confirmation" :
            "possible wrong-way motion held because heading evidence is stale or invalid";
          output = stopCommand();
        } else if (!wrong_way_geometry_valid) {
          mode = Mode::EMERGENCY_STOP;
          reason = "wrong-way recovery held because heading evidence is stale or invalid";
          output = stopCommand();
        } else if (!reactive_control_v2::wrong_way_recovery::reverseAuthorityValid(
            wrong_way_input))
        {
          mode = Mode::EMERGENCY_STOP;
          reason = "wrong-way recovery waiting for explicit Reactive authority";
          output = stopCommand();
        } else if (!stationary) {
          mode = Mode::EMERGENCY_STOP;
          reason = "wrong-way recovery waiting for a complete stop before reverse";
          output = stopCommand();
        } else if (wrong_way_reverse_attempt_count_ >= wrong_way_reverse_max_attempts_ ||
          reverse_attempt_count_ >= reverse_max_attempts_)
        {
          mode = Mode::EMERGENCY_STOP;
          reason = "wrong-way reverse attempt limit reached";
          output = stopCommand();
        } else {
          if (select_safe_wrong_way_turn(reverse_max_distance_m_)) {
            startWrongWayReverse(current_time, heading_error_rad, reverse_safety);
            mode = Mode::REVERSE_RECOVERY;
            reason = reverse_trigger_reason_;
            output = reverseCommand(
              reactive_control_v2::wrong_way_recovery::reverseSteeringAngle(
                wrong_way_turn_sign_, wrong_way_reverse_steering_rad_));
          } else {
            mode = Mode::EMERGENCY_STOP;
            reason = "wrong-way recovery swept gate blocked: " +
              reverse_swept_gate_reason_;
            output = stopCommand();
          }
        }
      }

      if (!wrong_way_control_owns) {
      const bool stationary = odom_healthy &&
        std::abs(current_speed) <= stationary_speed_threshold_mps_;
      const bool dead_end_evidence = reactive_mode_authorized &&
        enable_reverse_recovery_ && scan_data.valid &&
        stationary && base_mode == Mode::EMERGENCY_STOP &&
        (front_emergency || !fallback.valid);
      const bool reactive_stuck_evidence = reactive_mode_authorized &&
        enable_reverse_recovery_ && scan_data.valid &&
        odom_healthy && base_output.drive.speed >= stuck_forward_command_threshold_mps_ &&
        std::abs(current_speed) <= stuck_speed_threshold_mps_;
      reactive_control_v2::raceline_stall_handoff::Evidence handoff_evidence;
      handoff_evidence.enabled = enable_raceline_stall_handoff_;
      handoff_evidence.arbitration_required = require_arbitration_mode_;
      handoff_evidence.arbitration_mode_healthy = arbitration_mode_healthy;
      handoff_evidence.arbitration_mode = arbitration_mode;
      handoff_evidence.command_fresh = command_fresh;
      handoff_evidence.command_valid = command_valid;
      handoff_evidence.nominal_mode = base_mode == Mode::NOMINAL;
      handoff_evidence.scan_valid = scan_data.valid;
      handoff_evidence.odom_healthy = odom_healthy;
      // Use the selected primary request before low-speed assistance. A tiny
      // planned braking/yield command must not become strong-motion evidence
      // merely because assistance temporarily raises the final command.
      handoff_evidence.requested_forward_speed_mps = nominal_forward_speed;
      handoff_evidence.measured_speed_mps = current_speed;
      handoff_evidence.command_threshold_mps = stuck_forward_command_threshold_mps_;
      handoff_evidence.stopped_speed_threshold_mps = stuck_speed_threshold_mps_;
      raceline_stall_handoff_evidence_ =
        reactive_control_v2::raceline_stall_handoff::present(handoff_evidence);
      const bool stuck_evidence =
        reactive_stuck_evidence || raceline_stall_handoff_evidence_;

      if (dead_end_evidence) {
        if (!dead_end_timer_active_) {
          dead_end_started_at_ = current_time;
          dead_end_timer_active_ = true;
        }
      } else if (!enable_reverse_recovery_ || !scan_data.valid || !odom_healthy ||
        !stationary ||
        (base_mode != Mode::EMERGENCY_STOP && ftg_stably_available && !front_emergency))
      {
        // Once a stationary emergency starts the dead-end confirmation timer,
        // do not erase it because one scan briefly changes the instantaneous
        // FTG/emergency classification. Clear the latch only for unhealthy
        // inputs, actual motion, disabled recovery, or a stable usable FTG
        // route. Reverse entry below still requires dead_end_evidence to be
        // true in the current control cycle.
        dead_end_timer_active_ = false;
      }
      if (stuck_evidence) {
        if (!stuck_timer_active_) {
          stuck_started_at_ = current_time;
          stuck_timer_active_ = true;
        }
      } else {
        stuck_timer_active_ = false;
      }

      const bool dead_end_confirmed = dead_end_evidence && dead_end_timer_active_ &&
        (current_time - dead_end_started_at_).seconds() >= dead_end_confirmation_sec_;
      const double stuck_elapsed_sec = stuck_timer_active_ ?
        std::max(0.0, (current_time - stuck_started_at_).seconds()) : 0.0;
      const bool stuck_confirmed =
        reactive_control_v2::raceline_stall_handoff::confirmationReached(
        stuck_timer_active_, stuck_elapsed_sec, stuck_confirmation_sec_);
      const bool raceline_stall_confirmed =
        reactive_control_v2::raceline_stall_handoff::requestConfirmed(
        handoff_evidence, stuck_timer_active_, stuck_elapsed_sec,
        stuck_confirmation_sec_);
      const bool reactive_stuck_confirmed = reactive_stuck_evidence && stuck_confirmed;
      const bool attempts_available = reverse_attempt_count_ < reverse_max_attempts_;
      if (raceline_stall_confirmed) {
        // The lower layer owns the final command, so stop before asking the
        // arbitrator to transfer ownership. Existing lower-status coordination
        // recognizes EMERGENCY_STOP under mode 1 and selects REACTIVE. Reverse
        // remains forbidden until mode 2 is observed on a later control cycle.
        raceline_stall_handoff_active_ = true;
        raceline_stall_handoff_latched_ =
          reactive_control_v2::raceline_stall_handoff::updatePersistentLatch(
          raceline_stall_handoff_latched_, true, false);
        mode = Mode::EMERGENCY_STOP;
        reason = "positive RACELINE command persisted while odometry stayed low; "
          "requesting REACTIVE handoff";
        output = stopCommand();
        resetLowSpeedAssist();
      } else if ((reactive_stuck_confirmed || dead_end_confirmed) && attempts_available &&
        reverse_safety.valid && reverse_swept_clear_for(0, reverse_max_distance_m_))
      {
        const std::string trigger = reactive_stuck_confirmed ?
          "forward command persisted while VESC speed stayed low" :
          "stationary forward dead end persisted";
        startReverse(current_time, trigger);
        mode = Mode::REVERSE_RECOVERY;
        reason = trigger;
        output = reverseCommand();
      } else if ((reactive_stuck_confirmed || dead_end_confirmed) && !attempts_available) {
        mode = Mode::EMERGENCY_STOP;
        reason = "reverse recovery attempt limit reached";
        output = stopCommand();
      } else if ((reactive_stuck_confirmed || dead_end_confirmed) && !reverse_safety.valid) {
        mode = Mode::EMERGENCY_STOP;
        reason = "reverse recovery blocked: " + reverse_safety.reason;
        output = stopCommand();
      } else if ((reactive_stuck_confirmed || dead_end_confirmed) &&
        !reverse_swept_gate_valid_)
      {
        mode = Mode::EMERGENCY_STOP;
        reason = "reverse recovery swept gate blocked: " + reverse_swept_gate_reason_;
        output = stopCommand();
      }
      }
    }

    // A successful measured forward movement resets the bounded-attempt budget.
    // This remains VESC telemetry based; a future scan-motion confidence source
    // can be combined here without changing the recovery state machine.
    if (!reverse_active_ && !settle_active_ && output.drive.speed > 0.0 && odom_healthy &&
      current_speed >= reverse_attempt_reset_speed_mps_)
    {
      if (!forward_motion_timer_active_) {
        forward_motion_started_at_ = current_time;
        forward_motion_timer_active_ = true;
      } else if ((current_time - forward_motion_started_at_).seconds() >=
        reverse_attempt_reset_forward_time_sec_)
      {
        reverse_attempt_count_ = 0;
        reverse_trigger_reason_.clear();
        wrong_way_recovery_latched_ =
          reactive_control_v2::wrong_way_recovery::updatePersistentLatch(
          wrong_way_recovery_latched_, false,
          wrong_way_alignment_recovered_, true);
        if (!wrong_way_recovery_latched_) {
          wrong_way_reverse_attempt_count_ = 0;
          wrong_way_turn_sign_ = 0;
          raceline_stall_handoff_latched_ =
            reactive_control_v2::raceline_stall_handoff::updatePersistentLatch(
            raceline_stall_handoff_latched_, false, true);
        }
      }
    } else {
      forward_motion_timer_active_ = false;
    }

    if (mode == Mode::EMERGENCY_STOP || mode == Mode::RECOVERY_SETTLE) {
      if (!stopped_timer_active_) {
        stopped_since_ = current_time;
        stopped_timer_active_ = true;
      }
    } else {
      stopped_timer_active_ = false;
    }

    // Keep diagnostics aligned with the actual final speed after all safety
    // state transitions, emergency checks, and assistance decisions.
    low_speed_assist_output_command_mps_ = output.drive.speed;

    safe_command_pub_->publish(output);
    last_published_steering_rad_ = output.drive.steering_angle;
    logFallbackDebug(mode, scan_data, fallback, output);
    logReverseDebug(
      mode, current_speed, odom_healthy, odom_age, front_emergency,
      ftg_stably_available, reverse_safety);
    publishStatus(
      mode, reason, scan_data, fallback, command_age, scan_age, current_speed, upper_state,
      odom_healthy, odom_age, front_emergency, ftg_stably_available, reverse_safety,
      heading_error_rad, heading_error_age, heading_error_fresh,
      arbitration_mode, arbitration_mode_healthy, arbitration_mode_age,
      current_time);
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
