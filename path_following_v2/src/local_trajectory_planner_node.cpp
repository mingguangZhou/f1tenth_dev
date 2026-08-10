// Persistent localization-based local trajectory planner.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using std::placeholders::_1;

namespace
{
struct Point2
{
  double x{0.0};
  double y{0.0};
};

double distance(const Point2 & a, const Point2 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double pointToSegmentDistance(const Point2 & point, const Point2 & a, const Point2 & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length2 = dx * dx + dy * dy;
  if (length2 <= 1e-12) {
    return distance(point, a);
  }
  const double projection = std::clamp(
    ((point.x - a.x) * dx + (point.y - a.y) * dy) / length2, 0.0, 1.0);
  return std::hypot(
    point.x - (a.x + projection * dx),
    point.y - (a.y + projection * dy));
}

double quinticSmoothstep(const double u)
{
  const double x = std::clamp(u, 0.0, 1.0);
  return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

double quinticBoundaryValue(
  const double u, const double start, const double finish,
  const double initial_slope, const double transition_length)
{
  // Quintic Hermite boundary conditions in raceline Frenet coordinates:
  // start position and heading slope are respected; start acceleration and
  // final slope/acceleration are zero.  Curvature is still checked on the
  // resulting Cartesian path before acceptance.
  const double x = std::clamp(u, 0.0, 1.0);
  const double delta = finish - start;
  const double v0 = initial_slope * transition_length;
  const double a3 = 10.0 * delta - 6.0 * v0;
  const double a4 = -15.0 * delta + 8.0 * v0;
  const double a5 = 6.0 * delta - 3.0 * v0;
  return start + v0 * x + a3 * x * x * x +
         a4 * x * x * x * x + a5 * x * x * x * x * x;
}
}  // namespace

class LocalTrajectoryPlannerNode : public rclcpp::Node
{
public:
  LocalTrajectoryPlannerNode()
  : Node("local_trajectory_planner"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declareParameters();
    loadParameters();

    raw_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      raw_path_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&LocalTrajectoryPlannerNode::rawPathCallback, this, _1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalTrajectoryPlannerNode::scanCallback, this, _1));

    final_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      final_path_topic_, rclcpp::QoS(1).reliable().transient_local());
    speed_cap_pub_ = create_publisher<std_msgs::msg::Float64>(speed_cap_topic_, 10);
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(status_topic_, 10);
    if (publish_markers_) {
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    }

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LocalTrajectoryPlannerNode::controlLoop, this));

    RCLCPP_INFO(
      get_logger(),
      "local_trajectory_planner ready: raw=%s final=%s scan=%s envelope=%.2f m/side "
      "planning/scan=%.2f/%.2f m recovery_enter/exit=%.2f/%.2f m",
      raw_path_topic_.c_str(), final_path_topic_.c_str(), scan_topic_.c_str(),
      safety_half_width_m_, planning_distance_m_, scan_range_cap_m_,
      recovery_enter_lateral_error_m_,
      recovery_exit_lateral_error_m_);
  }

private:
  struct PathModel
  {
    std::vector<Point2> points;
    std::vector<double> s;
    std::string frame;
    double length{0.0};
  };

  struct Projection
  {
    bool valid{false};
    double s{0.0};
    double d{0.0};
    double yaw{0.0};
    double distance{std::numeric_limits<double>::infinity()};
    std::size_t segment_index{0};
  };

  struct ScanHit
  {
    Point2 point;
    Projection projection;
    std::size_t beam_index{0};
    double range{0.0};
  };

  struct ObstacleCluster
  {
    std::vector<ScanHit> hits;
    int interfering_points{0};
    double s_min{std::numeric_limits<double>::infinity()};
    double s_max{-std::numeric_limits<double>::infinity()};
    double d_min{std::numeric_limits<double>::infinity()};
    double d_max{-std::numeric_limits<double>::infinity()};
    double minimum_range{std::numeric_limits<double>::infinity()};
  };

  struct Candidate
  {
    bool valid{false};
    int side{0};  // +1 left, -1 right
    double peak_offset{0.0};
    double min_clearance{std::numeric_limits<double>::infinity()};
    double max_curvature{0.0};
    double rejoin_s{0.0};
    double start_lateral_offset{0.0};
    std::size_t pass_start_index{0};
    std::size_t pass_end_index{0};
    std::size_t rejoin_index{0};
    int maximum_connected_interference{0};
    std::string reason{"not evaluated"};
    std::string trajectory_mode{"NONE"};
    nav_msgs::msg::Path path;
  };

  struct ActivePlan
  {
    bool valid{false};
    unsigned long id{0};
    int side{0};
    std::string source_mode{"NONE"};
    std::string reason;
    nav_msgs::msg::Path path;
    std::size_t pass_end_index{0};
    std::size_t rejoin_index{0};
    std::size_t progress_index{0};
    double minimum_clearance{std::numeric_limits<double>::infinity()};
    double maximum_curvature{0.0};
    double peak_offset{0.0};
    double start_lateral_offset{0.0};
    bool side_committed{true};
    rclcpp::Time created_at{0, 0, RCL_STEADY_TIME};
    rclcpp::Time decision_started_at{0, 0, RCL_STEADY_TIME};
  };

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raw_path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr final_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_cap_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  std::mutex mutex_;
  nav_msgs::msg::Path::SharedPtr latest_raw_path_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  rclcpp::Time raw_path_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time scan_time_{0, 0, RCL_STEADY_TIME};
  bool raw_path_received_{false};
  bool scan_received_{false};

  std::string raw_path_topic_;
  std::string final_path_topic_;
  std::string scan_topic_;
  std::string speed_cap_topic_;
  std::string status_topic_;
  std::string marker_topic_;
  std::string robot_frame_;

  double control_rate_hz_{20.0};
  double raw_path_timeout_sec_{0.30};
  double scan_timeout_sec_{0.30};
  double transform_timeout_sec_{0.05};
  double scan_range_cap_m_{10.0};
  double planning_distance_m_{10.0};
  double vehicle_width_m_{0.28};
  double lateral_safety_margin_m_{0.10};
  double safety_half_width_m_{0.24};
  double min_valid_beam_ratio_{0.35};
  int blocked_min_points_{3};
  int cluster_max_beam_gap_{2};
  double cluster_point_gap_m_{0.18};
  double max_obstacle_size_m_{0.50};
  double obstacle_size_tolerance_m_{0.15};
  double critical_obstacle_distance_m_{0.80};
  double detour_longitudinal_buffer_m_{0.20};
  double detour_extra_clearance_m_{0.0};
  double detour_return_min_length_m_{2.50};
  double detour_return_max_length_m_{5.00};
  double candidate_lateral_step_m_{0.08};
  int max_candidates_per_side_{6};
  double max_lateral_shift_m_{0.80};
  double wheelbase_m_{0.33};
  double steering_max_deg_{20.6};
  double curvature_safety_factor_{0.95};
  double normal_speed_cap_mps_{2.0};
  double detour_speed_cap_mps_{1.0};
  double recovery_speed_cap_mps_{1.0};
  double replan_pending_speed_cap_mps_{0.4};
  double recovery_enter_lateral_error_m_{0.18};
  double recovery_exit_lateral_error_m_{0.08};
  double recovery_exit_heading_error_deg_{10.0};
  double recovery_min_rejoin_length_m_{2.5};
  double recovery_max_rejoin_length_m_{5.0};
  double recovery_length_gain_{3.0};
  double minimum_plan_hold_sec_{0.50};
  double maximum_plan_hold_sec_{12.0};
  double plan_deviation_replan_m_{0.35};
  double side_switch_clearance_advantage_m_{0.10};
  double side_commit_obstacle_distance_m_{3.0};
  double side_commit_lateral_progress_m_{0.08};
  double precommit_max_sec_{0.75};
  int precommit_refine_interval_scans_{2};
  int side_switch_confirmation_evaluations_{2};
  int rejoin_confirmation_scans_{4};
  int active_path_blocked_confirmation_scans_{3};
  int no_safe_path_confirmation_scans_{3};
  bool enable_detour_planning_{true};
  bool publish_markers_{true};

  ActivePlan active_plan_;
  unsigned long next_plan_id_{1};
  int rejoin_stable_cycles_{0};
  int active_blocked_cycles_{0};
  int no_safe_path_cycles_{0};
  int precommit_scan_cycles_{0};
  int alternative_side_win_cycles_{0};
  int alternative_side_{0};
  double last_effective_detection_distance_m_{0.0};
  rclcpp::Time last_processed_scan_time_{0, 0, RCL_STEADY_TIME};

  void declareParameters()
  {
    declare_parameter<std::string>(
      "raw_path_topic", "/path_following_v2/raceline_local_path");
    declare_parameter<std::string>("final_path_topic", "/path_following_v2/local_path");
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>(
      "speed_cap_topic", "/path_following_v2/trajectory_speed_cap_mps");
    declare_parameter<std::string>("status_topic", "/path_following_v2/path_status");
    declare_parameter<std::string>("marker_topic", "/path_following_v2/detour_markers");
    declare_parameter<std::string>("robot_frame", "base_link");

    declare_parameter<double>("control_rate_hz", 20.0);
    declare_parameter<double>("raw_path_timeout_sec", 0.30);
    declare_parameter<double>("scan_timeout_sec", 0.30);
    declare_parameter<double>("transform_timeout_sec", 0.05);
    // The raw path, scan use, and requested planning region share a 10 m
    // ceiling.  The effective obstacle-trigger distance is reduced only when
    // necessary to leave room for a complete, curvature-valid return.
    declare_parameter<double>("scan_range_cap_m", 10.0);
    declare_parameter<double>("planning_distance_m", 10.0);

    // Common footprint definition.  The effective per-side envelope is
    // vehicle_width / 2 + lateral_safety_margin = 0.24 m by default.
    declare_parameter<double>("vehicle_width_m", 0.28);
    declare_parameter<double>("lateral_safety_margin_m", 0.10);
    declare_parameter<double>("min_valid_beam_ratio", 0.35);
    declare_parameter<int>("blocked_min_points", 3);

    // Adjacent LaserScan returns are grouped before applying the competition
    // rule that an obstacle is smaller than 0.5 m x 0.5 m.
    declare_parameter<int>("cluster_max_beam_gap", 2);
    declare_parameter<double>("cluster_point_gap_m", 0.18);
    declare_parameter<double>("max_obstacle_size_m", 0.50);
    declare_parameter<double>("obstacle_size_tolerance_m", 0.15);
    declare_parameter<double>("critical_obstacle_distance_m", 0.80);

    declare_parameter<double>("detour_longitudinal_buffer_m", 0.20);
    declare_parameter<double>("detour_extra_clearance_m", 0.0);
    declare_parameter<double>("detour_return_min_length_m", 2.50);
    declare_parameter<double>("detour_return_max_length_m", 5.00);
    declare_parameter<double>("candidate_lateral_step_m", 0.08);
    declare_parameter<int>("max_candidates_per_side", 6);
    declare_parameter<double>("max_lateral_shift_m", 0.80);
    declare_parameter<double>("wheelbase_m", 0.33);
    declare_parameter<double>("steering_max_deg", 20.6);
    declare_parameter<double>("curvature_safety_factor", 0.95);
    declare_parameter<double>("normal_speed_cap_mps", 2.0);
    declare_parameter<double>("detour_speed_cap_mps", 1.0);
    declare_parameter<double>("recovery_speed_cap_mps", 1.0);
    declare_parameter<double>("replan_pending_speed_cap_mps", 0.4);

    // Enter/exit hysteresis for localization-based convergence to the global
    // raceline.  A local plan is not released merely because an obstacle
    // disappears from one scan.
    declare_parameter<double>("recovery_enter_lateral_error_m", 0.18);
    declare_parameter<double>("recovery_exit_lateral_error_m", 0.08);
    declare_parameter<double>("recovery_exit_heading_error_deg", 10.0);
    declare_parameter<double>("recovery_min_rejoin_length_m", 2.5);
    declare_parameter<double>("recovery_max_rejoin_length_m", 5.0);
    declare_parameter<double>("recovery_length_gain", 3.0);

    // A plan is map-frame anchored and normally held until the car passes its
    // rejoin anchor.  Time is only a stale-plan backstop, not the normal hold
    // condition.
    declare_parameter<double>("minimum_plan_hold_sec", 0.50);
    declare_parameter<double>("maximum_plan_hold_sec", 12.0);
    declare_parameter<double>("plan_deviation_replan_m", 0.35);
    declare_parameter<double>("side_switch_clearance_advantage_m", 0.10);
    declare_parameter<double>("side_commit_obstacle_distance_m", 3.0);
    declare_parameter<double>("side_commit_lateral_progress_m", 0.08);
    declare_parameter<double>("precommit_max_sec", 0.75);
    declare_parameter<int>("precommit_refine_interval_scans", 2);
    declare_parameter<int>("side_switch_confirmation_evaluations", 2);
    declare_parameter<int>("rejoin_confirmation_scans", 4);
    declare_parameter<int>("active_path_blocked_confirmation_scans", 3);
    declare_parameter<int>("no_safe_path_confirmation_scans", 3);
    declare_parameter<bool>("enable_detour_planning", true);
    declare_parameter<bool>("publish_markers", true);
  }

  void loadParameters()
  {
    raw_path_topic_ = get_parameter("raw_path_topic").as_string();
    final_path_topic_ = get_parameter("final_path_topic").as_string();
    scan_topic_ = get_parameter("scan_topic").as_string();
    speed_cap_topic_ = get_parameter("speed_cap_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    marker_topic_ = get_parameter("marker_topic").as_string();
    robot_frame_ = get_parameter("robot_frame").as_string();
    control_rate_hz_ = std::max(1.0, get_parameter("control_rate_hz").as_double());
    raw_path_timeout_sec_ = std::max(0.01, get_parameter("raw_path_timeout_sec").as_double());
    scan_timeout_sec_ = std::max(0.01, get_parameter("scan_timeout_sec").as_double());
    transform_timeout_sec_ = std::max(0.0, get_parameter("transform_timeout_sec").as_double());
    scan_range_cap_m_ = std::max(0.10, get_parameter("scan_range_cap_m").as_double());
    planning_distance_m_ = std::max(0.50, get_parameter("planning_distance_m").as_double());
    vehicle_width_m_ = std::max(0.01, get_parameter("vehicle_width_m").as_double());
    lateral_safety_margin_m_ = std::max(
      0.0, get_parameter("lateral_safety_margin_m").as_double());
    safety_half_width_m_ = 0.5 * vehicle_width_m_ + lateral_safety_margin_m_;
    min_valid_beam_ratio_ = std::clamp(
      get_parameter("min_valid_beam_ratio").as_double(), 0.0, 1.0);
    blocked_min_points_ = std::max(
      1, static_cast<int>(get_parameter("blocked_min_points").as_int()));
    cluster_max_beam_gap_ = std::max(
      1, static_cast<int>(get_parameter("cluster_max_beam_gap").as_int()));
    cluster_point_gap_m_ = std::max(0.01, get_parameter("cluster_point_gap_m").as_double());
    max_obstacle_size_m_ = std::max(0.01, get_parameter("max_obstacle_size_m").as_double());
    obstacle_size_tolerance_m_ = std::max(
      0.0, get_parameter("obstacle_size_tolerance_m").as_double());
    critical_obstacle_distance_m_ = std::max(
      0.0, get_parameter("critical_obstacle_distance_m").as_double());
    detour_longitudinal_buffer_m_ = std::max(
      0.0, get_parameter("detour_longitudinal_buffer_m").as_double());
    detour_extra_clearance_m_ = std::max(
      0.0, get_parameter("detour_extra_clearance_m").as_double());
    detour_return_min_length_m_ = std::max(
      0.5, get_parameter("detour_return_min_length_m").as_double());
    detour_return_max_length_m_ = std::max(
      detour_return_min_length_m_,
      get_parameter("detour_return_max_length_m").as_double());
    candidate_lateral_step_m_ = std::max(
      0.02, get_parameter("candidate_lateral_step_m").as_double());
    max_candidates_per_side_ = std::max(
      1, static_cast<int>(get_parameter("max_candidates_per_side").as_int()));
    max_lateral_shift_m_ = std::max(
      safety_half_width_m_, get_parameter("max_lateral_shift_m").as_double());
    wheelbase_m_ = std::max(0.01, get_parameter("wheelbase_m").as_double());
    steering_max_deg_ = std::max(0.1, get_parameter("steering_max_deg").as_double());
    curvature_safety_factor_ = std::clamp(
      get_parameter("curvature_safety_factor").as_double(), 0.1, 1.0);
    normal_speed_cap_mps_ = std::max(0.0, get_parameter("normal_speed_cap_mps").as_double());
    detour_speed_cap_mps_ = std::max(0.0, get_parameter("detour_speed_cap_mps").as_double());
    recovery_speed_cap_mps_ = std::max(
      0.0, get_parameter("recovery_speed_cap_mps").as_double());
    replan_pending_speed_cap_mps_ = std::max(
      0.0, get_parameter("replan_pending_speed_cap_mps").as_double());
    recovery_enter_lateral_error_m_ = std::max(
      0.01, get_parameter("recovery_enter_lateral_error_m").as_double());
    recovery_exit_lateral_error_m_ = std::clamp(
      get_parameter("recovery_exit_lateral_error_m").as_double(),
      0.0, recovery_enter_lateral_error_m_);
    recovery_exit_heading_error_deg_ = std::max(
      0.1, get_parameter("recovery_exit_heading_error_deg").as_double());
    recovery_min_rejoin_length_m_ = std::max(
      0.5, get_parameter("recovery_min_rejoin_length_m").as_double());
    recovery_max_rejoin_length_m_ = std::max(
      recovery_min_rejoin_length_m_,
      get_parameter("recovery_max_rejoin_length_m").as_double());
    recovery_length_gain_ = std::max(
      0.0, get_parameter("recovery_length_gain").as_double());
    minimum_plan_hold_sec_ = std::max(
      0.0, get_parameter("minimum_plan_hold_sec").as_double());
    maximum_plan_hold_sec_ = std::max(
      minimum_plan_hold_sec_ + 0.1,
      get_parameter("maximum_plan_hold_sec").as_double());
    plan_deviation_replan_m_ = std::max(
      recovery_exit_lateral_error_m_,
      get_parameter("plan_deviation_replan_m").as_double());
    side_switch_clearance_advantage_m_ = std::max(
      0.0, get_parameter("side_switch_clearance_advantage_m").as_double());
    side_commit_obstacle_distance_m_ = std::max(
      critical_obstacle_distance_m_,
      get_parameter("side_commit_obstacle_distance_m").as_double());
    side_commit_lateral_progress_m_ = std::max(
      0.0, get_parameter("side_commit_lateral_progress_m").as_double());
    precommit_max_sec_ = std::max(0.0, get_parameter("precommit_max_sec").as_double());
    precommit_refine_interval_scans_ = std::max(
      1, static_cast<int>(get_parameter("precommit_refine_interval_scans").as_int()));
    side_switch_confirmation_evaluations_ = std::max(
      1, static_cast<int>(
        get_parameter("side_switch_confirmation_evaluations").as_int()));
    rejoin_confirmation_scans_ = std::max(
      1, static_cast<int>(get_parameter("rejoin_confirmation_scans").as_int()));
    active_path_blocked_confirmation_scans_ = std::max(
      1, static_cast<int>(
        get_parameter("active_path_blocked_confirmation_scans").as_int()));
    no_safe_path_confirmation_scans_ = std::max(
      1, static_cast<int>(get_parameter("no_safe_path_confirmation_scans").as_int()));
    enable_detour_planning_ = get_parameter("enable_detour_planning").as_bool();
    publish_markers_ = get_parameter("publish_markers").as_bool();
  }

  void rawPathCallback(const nav_msgs::msg::Path::SharedPtr path)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_raw_path_ = path;
    raw_path_time_ = steady_clock_.now();
    raw_path_received_ = true;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_scan_ = scan;
    scan_time_ = steady_clock_.now();
    scan_received_ = true;
  }

  bool buildPathModel(const nav_msgs::msg::Path & path, PathModel & model) const
  {
    model = PathModel{};
    model.frame = path.header.frame_id;
    if (model.frame.empty() || path.poses.size() < 2) {
      return false;
    }
    model.points.reserve(path.poses.size());
    model.s.reserve(path.poses.size());
    for (const auto & pose : path.poses) {
      const Point2 point{pose.pose.position.x, pose.pose.position.y};
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        return false;
      }
      if (!model.points.empty()) {
        model.length += distance(model.points.back(), point);
      }
      model.points.push_back(point);
      model.s.push_back(model.length);
    }
    return model.length >= std::min(1.0, planning_distance_m_);
  }

  double effectiveObstacleDetectionDistance(const PathModel & raw) const
  {
    // Do not trigger a detour so far ahead that the current 10 m path cannot
    // contain the obstacle model plus the minimum smooth return.  This keeps
    // the early trigger useful instead of publishing a predictable
    // NO_SAFE_PATH while the obstacle is still far away.
    const double complete_plan_room =
      raw.length - 2.0 * detour_longitudinal_buffer_m_ -
      max_obstacle_size_m_ - detour_return_min_length_m_ - 0.20;
    return std::max(0.0, std::min({
        planning_distance_m_, scan_range_cap_m_, complete_plan_room}));
  }

  Projection projectToPath(const Point2 & point, const PathModel & path) const
  {
    Projection best;
    // Restrict Frenet association to the planning region.  This matters on a
    // small closed track where a later part of the 10 m raw window can be
    // spatially close to the current part of the lap.
    const double projection_limit = std::min(
      path.length, planning_distance_m_ + max_obstacle_size_m_);
    for (std::size_t i = 0; i + 1 < path.points.size(); ++i) {
      if (path.s[i] > projection_limit) {
        break;
      }
      const Point2 & a = path.points[i];
      const Point2 & b = path.points[i + 1];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double length2 = dx * dx + dy * dy;
      if (length2 <= 1e-12) {
        continue;
      }
      const double segment_length = std::sqrt(length2);
      const double u = std::clamp(
        ((point.x - a.x) * dx + (point.y - a.y) * dy) / length2, 0.0, 1.0);
      const Point2 closest{a.x + u * dx, a.y + u * dy};
      const double px = point.x - closest.x;
      const double py = point.y - closest.y;
      const double separation = std::hypot(px, py);
      if (separation < best.distance) {
        best.valid = true;
        best.distance = separation;
        best.s = path.s[i] + u * segment_length;
        best.d = (dx * py - dy * px) / segment_length;  // positive is path-left
        best.yaw = std::atan2(dy, dx);
        best.segment_index = i;
      }
    }
    return best;
  }

  bool lookupRobotPose(
    const std::string & frame, Point2 & position, double & yaw)
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        frame, robot_frame_, tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_sec_));
      position.x = transform.transform.translation.x;
      position.y = transform.transform.translation.y;
      tf2::Quaternion quaternion;
      tf2::fromMsg(transform.transform.rotation, quaternion);
      double roll = 0.0;
      double pitch = 0.0;
      tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
      return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(yaw);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 2000,
        "local trajectory robot-pose TF failed: %s", exception.what());
      return false;
    }
  }

  double angleDifference(const double first, const double second) const
  {
    return std::atan2(std::sin(first - second), std::cos(first - second));
  }

  bool scanHitsInPathFrame(
    const sensor_msgs::msg::LaserScan & scan, const PathModel & path,
    std::vector<ScanHit> & hits, double & valid_beam_ratio)
  {
    hits.clear();
    valid_beam_ratio = 0.0;
    if (scan.ranges.empty() || scan.header.frame_id.empty() ||
      !std::isfinite(scan.angle_increment) || scan.angle_increment <= 0.0)
    {
      return false;
    }

    tf2::Transform path_from_scan;
    try {
      const auto transform = tf_buffer_.lookupTransform(
        path.frame, scan.header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_sec_));
      tf2::fromMsg(transform.transform, path_from_scan);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 2000, "local trajectory scan TF failed: %s", exception.what());
      return false;
    }

    int valid_beams = 0;
    const double configured_min = std::isfinite(scan.range_min) ? scan.range_min : 0.0;
    const double sensor_max = std::isfinite(scan.range_max) && scan.range_max > 0.0 ?
      scan.range_max : scan_range_cap_m_;
    const double usable_max = std::min(sensor_max, scan_range_cap_m_);
    for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
      const double range = scan.ranges[index];
      if (!std::isnan(range) && range >= configured_min) {
        ++valid_beams;
      }
      if (!std::isfinite(range) || range < configured_min || range > usable_max) {
        continue;
      }
      const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
      const tf2::Vector3 transformed = path_from_scan * tf2::Vector3(
        range * std::cos(angle), range * std::sin(angle), 0.0);
      ScanHit hit;
      hit.point = Point2{transformed.x(), transformed.y()};
      hit.projection = projectToPath(hit.point, path);
      hit.beam_index = index;
      hit.range = range;
      if (hit.projection.valid) {
        hits.push_back(hit);
      }
    }
    valid_beam_ratio = static_cast<double>(valid_beams) /
      static_cast<double>(scan.ranges.size());
    return true;
  }

  std::vector<ObstacleCluster> clusterScanHits(
    const std::vector<ScanHit> & hits, const double detection_distance) const
  {
    std::vector<ObstacleCluster> clusters;
    for (const auto & hit : hits) {
      const bool start_new = clusters.empty() || clusters.back().hits.empty() ||
        hit.beam_index > clusters.back().hits.back().beam_index +
        static_cast<std::size_t>(cluster_max_beam_gap_) ||
        distance(hit.point, clusters.back().hits.back().point) > cluster_point_gap_m_;
      if (start_new) {
        clusters.emplace_back();
      }
      auto & cluster = clusters.back();
      cluster.hits.push_back(hit);
      cluster.s_min = std::min(cluster.s_min, hit.projection.s);
      cluster.s_max = std::max(cluster.s_max, hit.projection.s);
      cluster.d_min = std::min(cluster.d_min, hit.projection.d);
      cluster.d_max = std::max(cluster.d_max, hit.projection.d);
      cluster.minimum_range = std::min(cluster.minimum_range, hit.range);
      if (hit.projection.s >= 0.0 && hit.projection.s <= detection_distance &&
        std::abs(hit.projection.d) <= safety_half_width_m_)
      {
        ++cluster.interfering_points;
      }
    }
    return clusters;
  }

  bool selectNearestObstacle(
    const std::vector<ObstacleCluster> & clusters, ObstacleCluster & selected) const
  {
    bool found = false;
    double nearest_s = std::numeric_limits<double>::infinity();
    for (const auto & cluster : clusters) {
      if (cluster.interfering_points < blocked_min_points_) {
        continue;
      }
      if (cluster.s_min < nearest_s) {
        nearest_s = cluster.s_min;
        selected = cluster;
        found = true;
      }
    }
    return found;
  }

  geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw) const
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(quaternion);
  }

  double detourOffsetAt(
    const double s, const double departure_end, const double plateau_end,
    const double return_end, const double start_offset, const double peak_offset,
    const double initial_slope) const
  {
    if (s <= 0.0) {
      return start_offset;
    }
    if (s < departure_end) {
      return quinticBoundaryValue(
        s / std::max(1e-6, departure_end), start_offset, peak_offset,
        initial_slope, departure_end);
    }
    if (s <= plateau_end) {
      return peak_offset;
    }
    if (s < return_end) {
      return peak_offset * (1.0 - quinticSmoothstep(
        (s - plateau_end) / std::max(1e-6, return_end - plateau_end)));
    }
    return 0.0;
  }

  double recoveryOffsetAt(
    const double s, const double rejoin_end, const double start_offset,
    const double initial_slope) const
  {
    if (s <= 0.0) {
      return start_offset;
    }
    if (s >= rejoin_end) {
      return 0.0;
    }
    return quinticBoundaryValue(
      s / std::max(1e-6, rejoin_end), start_offset, 0.0,
      initial_slope, rejoin_end);
  }

  std::size_t indexAtOrAfter(const PathModel & path, const double target_s) const
  {
    const auto iterator = std::lower_bound(path.s.begin(), path.s.end(), target_s);
    if (iterator == path.s.end()) {
      return path.s.empty() ? 0 : path.s.size() - 1;
    }
    return static_cast<std::size_t>(std::distance(path.s.begin(), iterator));
  }

  bool fillCandidatePath(
    Candidate & candidate, const PathModel & raw,
    const nav_msgs::msg::Path & raw_message, const Point2 & robot_position,
    const double robot_yaw, const std::vector<double> & offsets,
    const std::vector<ScanHit> & all_hits) const
  {
    if (offsets.size() != raw.points.size() || raw.points.size() < 3) {
      candidate.reason = "offset/path size mismatch";
      return false;
    }

    std::vector<Point2> shifted;
    shifted.reserve(raw.points.size());
    for (std::size_t i = 0; i < raw.points.size(); ++i) {
      if (!std::isfinite(offsets[i]) || std::abs(offsets[i]) > max_lateral_shift_m_ + 1e-6) {
        candidate.reason = "generated trajectory exceeds max_lateral_shift_m";
        return false;
      }
      const std::size_t first = i == 0 ? 0 : i - 1;
      const std::size_t second = i + 1 < raw.points.size() ? i + 1 : i;
      double tx = raw.points[second].x - raw.points[first].x;
      double ty = raw.points[second].y - raw.points[first].y;
      const double tangent_norm = std::hypot(tx, ty);
      if (tangent_norm <= 1e-9) {
        candidate.reason = "raw path contains a zero-length tangent";
        return false;
      }
      tx /= tangent_norm;
      ty /= tangent_norm;
      shifted.push_back(Point2{
        raw.points[i].x - ty * offsets[i],
        raw.points[i].y + tx * offsets[i]});
    }

    // Make the plan start at the actual localized car pose.  The remainder is
    // generated in raceline Frenet coordinates, so it converges smoothly from
    // the current lateral position instead of assuming d=0 at every scan.
    shifted.front() = robot_position;
    candidate.path.header = raw_message.header;
    candidate.path.header.stamp = now();
    candidate.path.poses.resize(shifted.size());
    for (std::size_t i = 0; i < shifted.size(); ++i) {
      const std::size_t first = i == 0 ? 0 : i - 1;
      const std::size_t second = i + 1 < shifted.size() ? i + 1 : i;
      const double yaw = i == 0 ? robot_yaw : std::atan2(
        shifted[second].y - shifted[first].y,
        shifted[second].x - shifted[first].x);
      auto & pose = candidate.path.poses[i];
      pose.header = candidate.path.header;
      pose.pose.position.x = shifted[i].x;
      pose.pose.position.y = shifted[i].y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = yawToQuaternion(yaw);
    }

    const double curvature_limit = curvature_safety_factor_ *
      std::tan(steering_max_deg_ * M_PI / 180.0) / wheelbase_m_;
    // Curvature feasibility belongs only to geometry introduced by this
    // planner. Stop at the rejoin boundary: i < rejoin_index still checks the
    // final modified segment into the zero-offset rejoin point, but it does
    // not reject a candidate because of curvature farther ahead on the
    // unchanged global raceline.
    const std::size_t curvature_end = std::min(
      shifted.size() - 1, candidate.rejoin_index);
    for (std::size_t i = 1; i < curvature_end; ++i) {
      const double a = distance(shifted[i - 1], shifted[i]);
      const double b = distance(shifted[i], shifted[i + 1]);
      const double c = distance(shifted[i - 1], shifted[i + 1]);
      if (a <= 1e-6 || b <= 1e-6 || c <= 1e-6) {
        continue;
      }
      const double cross = std::abs(
        (shifted[i].x - shifted[i - 1].x) * (shifted[i + 1].y - shifted[i - 1].y) -
        (shifted[i].y - shifted[i - 1].y) * (shifted[i + 1].x - shifted[i - 1].x));
      candidate.max_curvature = std::max(
        candidate.max_curvature, 2.0 * cross / (a * b * c));
    }
    if (candidate.max_curvature > curvature_limit) {
      candidate.reason = "candidate curvature exceeds steering limit";
      return false;
    }

    // Use the same widened-band policy as the final guard.  One isolated beam
    // no longer rejects a whole candidate: rejection requires a connected run
    // of blocked_min_points scan returns inside the 0.24 m half-envelope.
    int connected = 0;
    std::size_t previous_beam = 0;
    Point2 previous_point;
    bool previous_interfered = false;
    const std::size_t collision_end = std::min(
      shifted.size() - 1, candidate.rejoin_index + static_cast<std::size_t>(3));
    for (const auto & hit : all_hits) {
      if (hit.projection.s < -0.20 || hit.projection.s > candidate.rejoin_s + 0.50) {
        continue;
      }
      double clearance = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < collision_end; ++i) {
        clearance = std::min(
          clearance, pointToSegmentDistance(hit.point, shifted[i], shifted[i + 1]));
      }

      // Score only the actual obstacle-passing section.  Departure and return
      // are shared smoothness requirements; including them here previously
      // biased the result toward the smaller offset rather than the wider gap.
      double pass_clearance = std::numeric_limits<double>::infinity();
      const std::size_t pass_begin = std::min(candidate.pass_start_index, collision_end);
      const std::size_t pass_end = std::min(candidate.pass_end_index + 1, collision_end);
      for (std::size_t i = pass_begin; i < pass_end; ++i) {
        pass_clearance = std::min(
          pass_clearance, pointToSegmentDistance(hit.point, shifted[i], shifted[i + 1]));
      }
      candidate.min_clearance = std::min(candidate.min_clearance, pass_clearance);

      const bool interfered = clearance <= safety_half_width_m_;
      if (interfered) {
        const bool connected_to_previous = previous_interfered &&
          hit.beam_index <= previous_beam + static_cast<std::size_t>(cluster_max_beam_gap_) &&
          distance(hit.point, previous_point) <= cluster_point_gap_m_;
        connected = connected_to_previous ? connected + 1 : 1;
        candidate.maximum_connected_interference = std::max(
          candidate.maximum_connected_interference, connected);
        previous_beam = hit.beam_index;
        previous_point = hit.point;
        previous_interfered = true;
      } else {
        connected = 0;
        previous_interfered = false;
      }
    }
    if (candidate.maximum_connected_interference >= blocked_min_points_) {
      candidate.reason = "widened candidate band has confirmed connected interference";
      return false;
    }
    if (!std::isfinite(candidate.min_clearance)) {
      candidate.min_clearance = scan_range_cap_m_;
    }
    return true;
  }

  Candidate buildDetourCandidateAtOffset(
    const PathModel & raw, const nav_msgs::msg::Path & raw_message,
    const ObstacleCluster & obstacle, const std::vector<ScanHit> & all_hits,
    const Point2 & robot_position, const double robot_yaw,
    const double car_lateral_offset, const double car_heading_error,
    const int side, const double peak_offset) const
  {
    Candidate candidate;
    candidate.side = side;
    candidate.trajectory_mode = "AVOIDING";
    candidate.peak_offset = peak_offset;
    candidate.start_lateral_offset = car_lateral_offset;

    const double departure_end = std::max(
      0.25, obstacle.s_min - detour_longitudinal_buffer_m_);
    const double conservative_obstacle_end = std::max(
      obstacle.s_max, obstacle.s_min + max_obstacle_size_m_);
    const double plateau_end = conservative_obstacle_end + detour_longitudinal_buffer_m_;
    const double available_return_length = raw.length - 0.20 - plateau_end;
    if (available_return_length < detour_return_min_length_m_) {
      candidate.reason = "raw path is too short for minimum smooth return";
      return candidate;
    }
    // Use the longest configured return that fits in the known raw raceline.
    // This lowers curvature and is allowed to extend beyond currently visible
    // scan returns; newly visible portions are checked while the plan is held.
    const double return_length = std::min(
      detour_return_max_length_m_, available_return_length);
    const double return_end = plateau_end + return_length;
    candidate.rejoin_s = return_end;
    candidate.pass_start_index = indexAtOrAfter(raw, departure_end);
    candidate.pass_end_index = indexAtOrAfter(raw, plateau_end);
    candidate.rejoin_index = indexAtOrAfter(raw, return_end);

    if (side * candidate.peak_offset <= 0.0) {
      candidate.reason = "required offset crosses the requested side";
      return candidate;
    }
    if (std::abs(candidate.peak_offset) > max_lateral_shift_m_) {
      candidate.reason = "required lateral shift exceeds max_lateral_shift_m";
      return candidate;
    }
    const double bounded_heading_error = std::clamp(
      car_heading_error, -M_PI / 4.0, M_PI / 4.0);
    const double initial_slope = std::tan(bounded_heading_error);
    std::vector<double> offsets;
    offsets.reserve(raw.s.size());
    for (const double s : raw.s) {
      offsets.push_back(detourOffsetAt(
        s, departure_end, plateau_end, return_end,
        car_lateral_offset, candidate.peak_offset, initial_slope));
    }
    if (!fillCandidatePath(
        candidate, raw, raw_message, robot_position, robot_yaw, offsets, all_hits))
    {
      return candidate;
    }
    candidate.valid = true;
    candidate.reason = "sampled corridor detour is curvature- and widened-band-valid";
    return candidate;
  }

  Candidate bestDetourCandidateForSide(
    const PathModel & raw, const nav_msgs::msg::Path & raw_message,
    const ObstacleCluster & obstacle, const std::vector<ScanHit> & all_hits,
    const Point2 & robot_position, const double robot_yaw,
    const double car_lateral_offset, const double car_heading_error,
    const int side) const
  {
    Candidate best;
    best.side = side;
    best.trajectory_mode = "AVOIDING";
    const double required_offset = side > 0 ?
      obstacle.d_max + safety_half_width_m_ + detour_extra_clearance_m_ :
      obstacle.d_min - safety_half_width_m_ - detour_extra_clearance_m_;
    if (side * required_offset <= 0.0) {
      best.reason = "required offset crosses the requested side";
      return best;
    }
    const double required_magnitude = std::abs(required_offset);
    if (required_magnitude > max_lateral_shift_m_) {
      best.reason = "required lateral shift exceeds max_lateral_shift_m";
      return best;
    }

    int evaluated = 0;
    std::string last_reason = "no sampled offset evaluated";
    for (int sample = 0; sample < max_candidates_per_side_; ++sample) {
      const double magnitude = required_magnitude +
        static_cast<double>(sample) * candidate_lateral_step_m_;
      if (magnitude > max_lateral_shift_m_ + 1e-9) {
        break;
      }
      ++evaluated;
      Candidate candidate = buildDetourCandidateAtOffset(
        raw, raw_message, obstacle, all_hits, robot_position, robot_yaw,
        car_lateral_offset, car_heading_error, side,
        static_cast<double>(side) * magnitude);
      last_reason = candidate.reason;
      if (!candidate.valid) {
        continue;
      }
      const bool clearly_safer = !best.valid ||
        candidate.min_clearance > best.min_clearance + 1e-3;
      const bool clearance_tied = best.valid &&
        std::abs(candidate.min_clearance - best.min_clearance) <= 1e-3;
      if (clearly_safer ||
        (clearance_tied && candidate.max_curvature < best.max_curvature))
      {
        best = std::move(candidate);
      }
    }
    if (!best.valid) {
      best.reason = "no valid corridor sample among " + std::to_string(evaluated) +
        " bounded offsets; last: " + last_reason;
    }
    return best;
  }

  Candidate buildRecoveryCandidate(
    const PathModel & raw, const nav_msgs::msg::Path & raw_message,
    const std::vector<ScanHit> & all_hits, const Point2 & robot_position,
    const double robot_yaw, const double car_lateral_offset,
    const double car_heading_error) const
  {
    Candidate candidate;
    candidate.side = 0;
    candidate.trajectory_mode = "RECOVERING_TO_RACELINE";
    candidate.peak_offset = car_lateral_offset;
    candidate.start_lateral_offset = car_lateral_offset;
    candidate.rejoin_s = std::clamp(
      recovery_min_rejoin_length_m_ +
      recovery_length_gain_ * std::abs(car_lateral_offset),
      recovery_min_rejoin_length_m_, recovery_max_rejoin_length_m_);
    candidate.pass_start_index = 0;
    candidate.pass_end_index = indexAtOrAfter(raw, candidate.rejoin_s * 0.5);
    candidate.rejoin_index = indexAtOrAfter(raw, candidate.rejoin_s);
    if (candidate.rejoin_s > raw.length - 0.20) {
      candidate.reason = "raw path is too short for lateral recovery rejoin";
      return candidate;
    }

    const double bounded_heading_error = std::clamp(
      car_heading_error, -M_PI / 4.0, M_PI / 4.0);
    const double initial_slope = std::tan(bounded_heading_error);
    std::vector<double> offsets;
    offsets.reserve(raw.s.size());
    for (const double s : raw.s) {
      offsets.push_back(recoveryOffsetAt(
        s, candidate.rejoin_s, car_lateral_offset, initial_slope));
    }
    if (!fillCandidatePath(
        candidate, raw, raw_message, robot_position, robot_yaw, offsets, all_hits))
    {
      return candidate;
    }
    candidate.valid = true;
    candidate.reason = "persistent current-pose-to-raceline recovery is valid";
    return candidate;
  }

  const Candidate * chooseCandidate(const Candidate & left, const Candidate & right) const
  {
    if (left.valid && !right.valid) {
      return &left;
    }
    if (right.valid && !left.valid) {
      return &right;
    }
    if (!left.valid && !right.valid) {
      return nullptr;
    }

    // Preserve the active side unless the alternative is wider by a material
    // configurable amount.  This prevents scan noise from flipping the plan
    // between left and right during an avoidance maneuver.
    if (active_plan_.valid && active_plan_.side > 0 && left.valid &&
      right.min_clearance < left.min_clearance + side_switch_clearance_advantage_m_)
    {
      return &left;
    }
    if (active_plan_.valid && active_plan_.side < 0 && right.valid &&
      left.min_clearance < right.min_clearance + side_switch_clearance_advantage_m_)
    {
      return &right;
    }
    if (std::abs(left.min_clearance - right.min_clearance) > 1e-3) {
      return left.min_clearance > right.min_clearance ? &left : &right;
    }
    if (std::abs(left.max_curvature - right.max_curvature) > 1e-3) {
      return left.max_curvature < right.max_curvature ? &left : &right;
    }
    return std::abs(left.peak_offset) <= std::abs(right.peak_offset) ? &left : &right;
  }

  void activatePlan(
    const Candidate & candidate, const std::string & reason,
    const bool preserve_decision_start = false)
  {
    const bool preserve = preserve_decision_start && active_plan_.valid;
    const rclcpp::Time decision_start = preserve ?
      active_plan_.decision_started_at : steady_clock_.now();
    active_plan_ = ActivePlan();
    active_plan_.valid = true;
    active_plan_.id = next_plan_id_++;
    active_plan_.side = candidate.side;
    active_plan_.source_mode = candidate.trajectory_mode;
    active_plan_.reason = reason;
    active_plan_.path = candidate.path;
    active_plan_.pass_end_index = candidate.pass_end_index;
    active_plan_.rejoin_index = candidate.rejoin_index;
    active_plan_.minimum_clearance = candidate.min_clearance;
    active_plan_.maximum_curvature = candidate.max_curvature;
    active_plan_.peak_offset = candidate.peak_offset;
    active_plan_.start_lateral_offset = candidate.start_lateral_offset;
    active_plan_.side_committed = candidate.side == 0;
    active_plan_.created_at = steady_clock_.now();
    active_plan_.decision_started_at = decision_start;
    rejoin_stable_cycles_ = 0;
    active_blocked_cycles_ = 0;
    no_safe_path_cycles_ = 0;
    precommit_scan_cycles_ = 0;
    alternative_side_win_cycles_ = 0;
    alternative_side_ = 0;
    RCLCPP_INFO(
      get_logger(),
      "activated local plan %lu: mode=%s side=%s rejoin_index=%zu reason=%s",
      active_plan_.id, active_plan_.source_mode.c_str(),
      active_plan_.side > 0 ? "LEFT" : (active_plan_.side < 0 ? "RIGHT" : "NONE"),
      active_plan_.rejoin_index, reason.c_str());
  }

  void commitActiveSide(const std::string & reason)
  {
    if (!active_plan_.valid || active_plan_.side == 0 || active_plan_.side_committed) {
      return;
    }
    active_plan_.side_committed = true;
    alternative_side_win_cycles_ = 0;
    alternative_side_ = 0;
    RCLCPP_INFO(
      get_logger(), "committed local plan %lu to %s: %s",
      active_plan_.id, active_plan_.side > 0 ? "LEFT" : "RIGHT", reason.c_str());
  }

  void releaseActivePlan(const std::string & reason)
  {
    if (active_plan_.valid) {
      RCLCPP_INFO(
        get_logger(), "released local plan %lu: %s", active_plan_.id, reason.c_str());
    }
    active_plan_ = ActivePlan();
    rejoin_stable_cycles_ = 0;
    active_blocked_cycles_ = 0;
    precommit_scan_cycles_ = 0;
    alternative_side_win_cycles_ = 0;
    alternative_side_ = 0;
  }

  double updateActiveProgress(const Point2 & robot_position)
  {
    if (!active_plan_.valid || active_plan_.path.poses.empty()) {
      return std::numeric_limits<double>::infinity();
    }
    std::size_t best_index = active_plan_.progress_index;
    double best_distance = std::numeric_limits<double>::infinity();
    const std::size_t begin = active_plan_.progress_index > 5 ?
      active_plan_.progress_index - 5 : 0;
    for (std::size_t i = begin; i < active_plan_.path.poses.size(); ++i) {
      const Point2 point{
        active_plan_.path.poses[i].pose.position.x,
        active_plan_.path.poses[i].pose.position.y};
      const double separation = distance(robot_position, point);
      if (separation < best_distance) {
        best_distance = separation;
        best_index = i;
      }
    }
    // Progress is monotonic so a close loop segment behind the car cannot
    // rewind a map-anchored local plan.
    active_plan_.progress_index = std::max(active_plan_.progress_index, best_index);
    return best_distance;
  }

  nav_msgs::msg::Path continuousActivePath(
    const nav_msgs::msg::Path & raw_path) const
  {
    nav_msgs::msg::Path output;
    if (!active_plan_.valid || active_plan_.path.poses.size() < 2) {
      return output;
    }

    output.header = active_plan_.path.header;
    output.header.stamp = now();
    const double target_length = planning_distance_m_;
    double output_length = 0.0;

    // Append one pose while keeping the published trajectory within the same
    // physical horizon used by the raw raceline generator. An exact duplicate
    // at the replan/raceline splice is skipped.
    const auto append_pose = [&output, &output_length, target_length](
        geometry_msgs::msg::PoseStamped pose) -> bool
      {
        pose.header = output.header;
        if (output.poses.empty()) {
          output.poses.push_back(pose);
          return true;
        }

        const auto & previous = output.poses.back().pose.position;
        const auto & current = pose.pose.position;
        const double segment = std::hypot(
          current.x - previous.x, current.y - previous.y);
        if (segment <= 1e-6) {
          return true;
        }
        if (output_length + segment > target_length) {
          return false;
        }
        output_length += segment;
        output.poses.push_back(pose);
        return true;
      };

    const std::size_t start = active_plan_.progress_index > 0 ?
      active_plan_.progress_index - 1 : 0;

    // Keep only the still-relevant, locally modified part of the stored plan.
    // The stale raw-raceline tail saved when the plan was created is replaced
    // below by the newest raw 10 m window.
    if (start <= active_plan_.rejoin_index) {
      const std::size_t local_end = std::min(
        active_plan_.rejoin_index, active_plan_.path.poses.size() - 1);
      output.poses.reserve(
        local_end - start + 1 + raw_path.poses.size());
      for (std::size_t i = start; i <= local_end; ++i) {
        if (!append_pose(active_plan_.path.poses[i])) {
          return output;
        }
      }
    } else {
      output.poses.reserve(raw_path.poses.size());
    }

    // Once the local curve reaches d=0, splice in the current raw raceline.
    // If the car has already passed the rejoin index during the short release
    // confirmation, publish the fresh raw path directly. Otherwise locate the
    // matching rejoin point in the current raw window and append only the
    // raceline ahead of it.
    std::size_t raw_start = 0;
    if (!output.poses.empty() && !raw_path.poses.empty()) {
      const Point2 anchor{
        output.poses.back().pose.position.x,
        output.poses.back().pose.position.y};
      double best_distance = std::numeric_limits<double>::infinity();
      std::size_t best_index = 0;
      for (std::size_t i = 0; i < raw_path.poses.size(); ++i) {
        const Point2 point{
          raw_path.poses[i].pose.position.x,
          raw_path.poses[i].pose.position.y};
        const double separation = distance(anchor, point);
        if (separation < best_distance) {
          best_distance = separation;
          best_index = i;
        }
      }
      raw_start = std::min(best_index + 1, raw_path.poses.size());
    }

    for (std::size_t i = raw_start; i < raw_path.poses.size(); ++i) {
      if (!append_pose(raw_path.poses[i])) {
        break;
      }
    }
    return output;
  }

  std::string activeTrajectoryMode() const
  {
    if (!active_plan_.valid) {
      return "NONE";
    }
    if (active_plan_.side == 0) {
      return "RECOVERING_TO_RACELINE";
    }
    return active_plan_.progress_index >= active_plan_.pass_end_index ?
      "REJOINING" : "AVOIDING";
  }

  bool activePathBlocked(const std::vector<ScanHit> & hits) const
  {
    if (!active_plan_.valid || active_plan_.path.poses.size() < 2) {
      return true;
    }
    const std::size_t start = active_plan_.progress_index > 0 ?
      active_plan_.progress_index - 1 : 0;
    const std::size_t end = std::min(
      active_plan_.path.poses.size() - 1,
      std::max(active_plan_.rejoin_index, active_plan_.progress_index) +
      static_cast<std::size_t>(3));
    int connected = 0;
    int maximum_connected = 0;
    std::size_t previous_beam = 0;
    Point2 previous_point;
    bool previous_interfered = false;
    for (const auto & hit : hits) {
      double clearance = std::numeric_limits<double>::infinity();
      for (std::size_t i = start; i < end; ++i) {
        const Point2 a{
          active_plan_.path.poses[i].pose.position.x,
          active_plan_.path.poses[i].pose.position.y};
        const Point2 b{
          active_plan_.path.poses[i + 1].pose.position.x,
          active_plan_.path.poses[i + 1].pose.position.y};
        clearance = std::min(clearance, pointToSegmentDistance(hit.point, a, b));
      }
      const bool interfered = clearance <= safety_half_width_m_;
      if (interfered) {
        const bool connected_to_previous = previous_interfered &&
          hit.beam_index <= previous_beam + static_cast<std::size_t>(cluster_max_beam_gap_) &&
          distance(hit.point, previous_point) <= cluster_point_gap_m_;
        connected = connected_to_previous ? connected + 1 : 1;
        maximum_connected = std::max(maximum_connected, connected);
        previous_beam = hit.beam_index;
        previous_point = hit.point;
        previous_interfered = true;
      } else {
        connected = 0;
        previous_interfered = false;
      }
    }
    return maximum_connected >= blocked_min_points_;
  }

  double activeSpeedCap() const
  {
    return active_plan_.side == 0 ? recovery_speed_cap_mps_ : detour_speed_cap_mps_;
  }

  void publishSpeedCap(const double cap)
  {
    std_msgs::msg::Float64 message;
    message.data = std::max(0.0, cap);
    speed_cap_pub_->publish(message);
  }

  void publishPath(nav_msgs::msg::Path path)
  {
    path.header.stamp = now();
    for (auto & pose : path.poses) {
      pose.header = path.header;
    }
    final_path_pub_->publish(path);
  }

  void publishStatus(
    const std::string & state, const std::string & reason,
    const std::string & trajectory_mode, const int side,
    const double raw_path_age, const double scan_age,
    const double valid_beam_ratio = 0.0,
    const ObstacleCluster * obstacle = nullptr,
    const Candidate * candidate = nullptr)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    // Keep this diagnostic name for drive_arbitrator compatibility: it now
    // represents the complete primary local-trajectory generation chain.
    status.name = "path_following_v2/path_generator";
    status.hardware_id = "persistent_local_trajectory_planner";
    status.level = state == "READY" ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = state + ": " + reason;

    auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue pair;
        pair.key = key;
        pair.value = value;
        status.values.push_back(pair);
      };
    add("state", state);
    add("reason", reason);
    add("trajectory_mode", trajectory_mode);
    add("detour_side", side > 0 ? "LEFT" : (side < 0 ? "RIGHT" : "NONE"));
    add("raw_path_age_sec", std::to_string(raw_path_age));
    add("scan_age_sec", std::to_string(scan_age));
    add("valid_beam_ratio", std::to_string(valid_beam_ratio));
    add("vehicle_width_m", std::to_string(vehicle_width_m_));
    add("lateral_safety_margin_m", std::to_string(lateral_safety_margin_m_));
    add("safety_half_width_m", std::to_string(safety_half_width_m_));
    add("planning_distance_m", std::to_string(planning_distance_m_));
    add("scan_range_cap_m", std::to_string(scan_range_cap_m_));
    add(
      "effective_obstacle_detection_distance_m",
      std::to_string(last_effective_detection_distance_m_));
    double reported_speed_cap = 0.0;
    if (state == "READY") {
      if (trajectory_mode == "RACELINE") {
        reported_speed_cap = normal_speed_cap_mps_;
      } else if (trajectory_mode == "REPLAN_PENDING") {
        reported_speed_cap = replan_pending_speed_cap_mps_;
      } else {
        reported_speed_cap = activeSpeedCap();
      }
    }
    add("speed_cap_mps", std::to_string(reported_speed_cap));
    add("local_plan_active", active_plan_.valid ? "true" : "false");
    add(
      "side_committed",
      active_plan_.valid && active_plan_.side != 0 ?
      (active_plan_.side_committed ? "true" : "false") : "n/a");
    add("plan_id", active_plan_.valid ? std::to_string(active_plan_.id) : "0");
    add(
      "plan_age_sec",
      active_plan_.valid ?
      std::to_string((steady_clock_.now() - active_plan_.created_at).seconds()) : "0.0");
    add(
      "side_decision_age_sec",
      active_plan_.valid && active_plan_.side != 0 ?
      std::to_string(
        (steady_clock_.now() - active_plan_.decision_started_at).seconds()) : "0.0");
    add(
      "plan_progress_index",
      active_plan_.valid ? std::to_string(active_plan_.progress_index) : "0");
    add(
      "plan_rejoin_index",
      active_plan_.valid ? std::to_string(active_plan_.rejoin_index) : "0");
    add("rejoin_stable_cycles", std::to_string(rejoin_stable_cycles_));
    add("active_blocked_cycles", std::to_string(active_blocked_cycles_));
    add("no_safe_path_cycles", std::to_string(no_safe_path_cycles_));
    add("precommit_scan_cycles", std::to_string(precommit_scan_cycles_));
    add("alternative_side_win_cycles", std::to_string(alternative_side_win_cycles_));
    add(
      "hold_rule",
      "until rejoin anchor + lateral/heading convergence; time only bounds stale plans");
    if (obstacle) {
      add("obstacle_distance_m", std::to_string(obstacle->minimum_range));
      add("obstacle_s_min_m", std::to_string(obstacle->s_min));
      add("obstacle_s_max_m", std::to_string(obstacle->s_max));
      add("obstacle_d_min_m", std::to_string(obstacle->d_min));
      add("obstacle_d_max_m", std::to_string(obstacle->d_max));
      add("interfering_points", std::to_string(obstacle->interfering_points));
    }
    if (candidate) {
      add("minimum_clearance_m", std::to_string(candidate->min_clearance));
      add("maximum_curvature_inv_m", std::to_string(candidate->max_curvature));
      add("peak_offset_m", std::to_string(candidate->peak_offset));
      add("rejoin_distance_m", std::to_string(candidate->rejoin_s));
    }
    array.status.push_back(status);
    status_pub_->publish(array);
  }

  visualization_msgs::msg::Marker lineMarker(
    const nav_msgs::msg::Path & path, const int id,
    const float red, const float green, const float blue,
    const float alpha, const double width) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.ns = "local_trajectory_candidates";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = width;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    for (const auto & pose : path.poses) {
      geometry_msgs::msg::Point point;
      point.x = pose.pose.position.x;
      point.y = pose.pose.position.y;
      point.z = 0.02;
      marker.points.push_back(point);
    }
    return marker;
  }

  void clearMarkers(const std::string & frame)
  {
    if (!publish_markers_) {
      return;
    }
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = now();
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(marker);
    marker_pub_->publish(array);
  }

  void publishMarkers(
    const ObstacleCluster & obstacle, const Candidate & left,
    const Candidate & right, const int selected_side, const std::string & frame)
  {
    if (!publish_markers_) {
      return;
    }
    visualization_msgs::msg::MarkerArray array;
    if (!left.path.poses.empty()) {
      array.markers.push_back(lineMarker(
        left.path, 1,
        selected_side > 0 ? 0.20F : 0.55F,
        selected_side > 0 ? 0.80F : 0.55F,
        selected_side > 0 ? 1.00F : 0.55F,
        selected_side > 0 ? 1.0F : 0.25F, selected_side > 0 ? 0.06 : 0.03));
    }
    if (!right.path.poses.empty()) {
      array.markers.push_back(lineMarker(
        right.path, 2,
        selected_side < 0 ? 0.20F : 0.55F,
        selected_side < 0 ? 0.80F : 0.55F,
        selected_side < 0 ? 1.00F : 0.55F,
        selected_side < 0 ? 1.0F : 0.25F, selected_side < 0 ? 0.06 : 0.03));
    }

    visualization_msgs::msg::Marker points;
    points.header.frame_id = frame;
    points.header.stamp = now();
    points.ns = "local_trajectory_obstacle";
    points.id = 3;
    points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    points.action = visualization_msgs::msg::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.scale.x = 0.08;
    points.scale.y = 0.08;
    points.scale.z = 0.08;
    points.color.r = 1.0F;
    points.color.g = 0.1F;
    points.color.b = 0.1F;
    points.color.a = 1.0F;
    for (const auto & hit : obstacle.hits) {
      geometry_msgs::msg::Point point;
      point.x = hit.point.x;
      point.y = hit.point.y;
      point.z = 0.04;
      points.points.push_back(point);
    }
    array.markers.push_back(points);
    marker_pub_->publish(array);
  }

  void failPrimary(
    const std::string & state, const std::string & reason,
    const double raw_path_age, const double scan_age,
    const double valid_beam_ratio = 0.0,
    const ObstacleCluster * obstacle = nullptr,
    const Candidate * candidate = nullptr)
  {
    // A zero cap stops the primary follower immediately while the arbitrator
    // observes the non-READY status and transfers ownership to Reactive.
    publishSpeedCap(0.0);
    publishStatus(
      state, reason, "NONE", 0, raw_path_age, scan_age,
      valid_beam_ratio, obstacle, candidate);
  }

  void controlLoop()
  {
    nav_msgs::msg::Path::SharedPtr raw_message;
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    rclcpp::Time raw_time(0, 0, RCL_STEADY_TIME);
    rclcpp::Time current_scan_time(0, 0, RCL_STEADY_TIME);
    bool raw_received = false;
    bool scan_received = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      raw_message = latest_raw_path_;
      scan = latest_scan_;
      raw_time = raw_path_time_;
      current_scan_time = scan_time_;
      raw_received = raw_path_received_;
      scan_received = scan_received_;
    }

    const auto current_time = steady_clock_.now();
    const double raw_age = raw_received ? (current_time - raw_time).seconds() :
      std::numeric_limits<double>::infinity();
    const double scan_age = scan_received ? (current_time - current_scan_time).seconds() :
      std::numeric_limits<double>::infinity();
    if (!raw_received || !raw_message || raw_age > raw_path_timeout_sec_) {
      failPrimary(
        "INPUT_INVALID", raw_received ? "raw raceline path is stale" :
        "no raw raceline path received", raw_age, scan_age);
      return;
    }
    if (!scan_received || !scan || scan_age > scan_timeout_sec_) {
      failPrimary(
        "SCAN_INVALID", scan_received ? "LaserScan is stale" :
        "no LaserScan received", raw_age, scan_age);
      return;
    }

    PathModel raw;
    if (!buildPathModel(*raw_message, raw)) {
      failPrimary("INPUT_INVALID", "raw raceline path is malformed or too short", raw_age, scan_age);
      return;
    }

    // Re-evaluate safety at the control rate, but do not rebuild a successful
    // plan every cycle.  Active-plan geometry is map anchored and only its
    // already-passed prefix is trimmed for downstream consumers.
    std::vector<ScanHit> hits;
    double valid_beam_ratio = 0.0;
    if (!scanHitsInPathFrame(*scan, raw, hits, valid_beam_ratio)) {
      failPrimary("TF_UNAVAILABLE", "scan-to-path transform or LaserScan is invalid", raw_age, scan_age);
      return;
    }
    if (valid_beam_ratio < min_valid_beam_ratio_) {
      failPrimary(
        "SCAN_INVALID", "insufficient valid LaserScan beams",
        raw_age, scan_age, valid_beam_ratio);
      return;
    }

    Point2 robot_position;
    double robot_yaw = 0.0;
    if (!lookupRobotPose(raw.frame, robot_position, robot_yaw)) {
      failPrimary(
        "TF_UNAVAILABLE", "path-frame-to-robot transform is unavailable",
        raw_age, scan_age, valid_beam_ratio);
      return;
    }
    const Projection robot_on_raw = projectToPath(robot_position, raw);
    if (!robot_on_raw.valid) {
      failPrimary(
        "INPUT_INVALID", "robot pose cannot be projected onto raw raceline",
        raw_age, scan_age, valid_beam_ratio);
      return;
    }
    const double car_heading_error = angleDifference(robot_yaw, robot_on_raw.yaw);

    const bool new_scan = current_scan_time.nanoseconds() !=
      last_processed_scan_time_.nanoseconds();
    if (new_scan) {
      last_processed_scan_time_ = current_scan_time;
    }

    last_effective_detection_distance_m_ = effectiveObstacleDetectionDistance(raw);
    const auto clusters = clusterScanHits(hits, last_effective_detection_distance_m_);
    ObstacleCluster obstacle;
    const bool obstacle_found = selectNearestObstacle(clusters, obstacle);
    const double allowed_extent = max_obstacle_size_m_ + obstacle_size_tolerance_m_;
    const bool obstacle_size_valid = !obstacle_found ||
      ((obstacle.s_max - obstacle.s_min <= allowed_extent) &&
      (obstacle.d_max - obstacle.d_min <= allowed_extent));
    if (!active_plan_.valid && obstacle_found &&
      obstacle.minimum_range <= critical_obstacle_distance_m_)
    {
      failPrimary(
        "CRITICAL_OBSTACLE", "obstacle is too close to begin a smooth detour",
        raw_age, scan_age, valid_beam_ratio, &obstacle);
      return;
    }

    // First service an already accepted plan.  Obstacle disappearance does
    // not release it.  The car must physically reach and stably converge at
    // its stored rejoin anchor.
    if (active_plan_.valid) {
      const double deviation = updateActiveProgress(robot_position);
      const double plan_age = (steady_clock_.now() - active_plan_.created_at).seconds();
      const double heading_error = std::abs(car_heading_error);
      const bool rejoin_geometry_reached =
        plan_age >= minimum_plan_hold_sec_ &&
        active_plan_.progress_index >= active_plan_.rejoin_index &&
        std::abs(robot_on_raw.d) <= recovery_exit_lateral_error_m_ &&
        heading_error <= recovery_exit_heading_error_deg_ * M_PI / 180.0;
      if (new_scan) {
        rejoin_stable_cycles_ = rejoin_geometry_reached ?
          rejoin_stable_cycles_ + 1 : 0;
      }
      if (rejoin_stable_cycles_ >= rejoin_confirmation_scans_) {
        releaseActivePlan("rejoin anchor passed and lateral/heading convergence confirmed");
      }

      if (active_plan_.valid) {
        const bool blocked_now = activePathBlocked(hits);
        if (new_scan) {
          active_blocked_cycles_ = blocked_now ? active_blocked_cycles_ + 1 : 0;
        }
        const bool stale_plan = plan_age >= maximum_plan_hold_sec_;
        const bool excessive_deviation = deviation > plan_deviation_replan_m_;
        const bool end_without_rejoin =
          active_plan_.progress_index + 10 >= active_plan_.path.poses.size() &&
          !rejoin_geometry_reached;
        const bool blocked_confirmed =
          active_blocked_cycles_ >= active_path_blocked_confirmation_scans_;
        const bool update_requested = blocked_confirmed || stale_plan ||
          excessive_deviation || end_without_rejoin;

        // Refine the wider-side decision only for a short early window.  This
        // lets fresh scan geometry correct an initial partial view, but locks
        // the side before a late left/right switch would destabilize control.
        if (!update_requested && active_plan_.side != 0 &&
          !active_plan_.side_committed)
        {
          if (new_scan) {
            ++precommit_scan_cycles_;
          }
          const double decision_age =
            (steady_clock_.now() - active_plan_.decision_started_at).seconds();
          const double signed_lateral_progress = static_cast<double>(active_plan_.side) *
            (robot_on_raw.d - active_plan_.start_lateral_offset);
          if (!obstacle_found) {
            commitActiveSide("raw-raceline obstacle is no longer available for side comparison");
          } else if (obstacle.s_min <= side_commit_obstacle_distance_m_) {
            commitActiveSide("obstacle reached configured side-commit distance");
          } else if (signed_lateral_progress >= side_commit_lateral_progress_m_) {
            commitActiveSide("vehicle began meaningful lateral motion");
          } else if (decision_age >= precommit_max_sec_) {
            commitActiveSide("pre-commit refinement time elapsed");
          } else if (new_scan && obstacle_size_valid && enable_detour_planning_ &&
            precommit_scan_cycles_ % precommit_refine_interval_scans_ == 0)
          {
            Candidate left = bestDetourCandidateForSide(
              raw, *raw_message, obstacle, hits, robot_position, robot_yaw,
              robot_on_raw.d, car_heading_error, +1);
            Candidate right = bestDetourCandidateForSide(
              raw, *raw_message, obstacle, hits, robot_position, robot_yaw,
              robot_on_raw.d, car_heading_error, -1);
            const Candidate * refined = chooseCandidate(left, right);
            publishMarkers(
              obstacle, left, right, refined ? refined->side : active_plan_.side, raw.frame);
            if (refined && refined->side != active_plan_.side) {
              if (alternative_side_ == refined->side) {
                ++alternative_side_win_cycles_;
              } else {
                alternative_side_ = refined->side;
                alternative_side_win_cycles_ = 1;
              }
              if (alternative_side_win_cycles_ >= side_switch_confirmation_evaluations_) {
                activatePlan(
                  *refined,
                  "early confirmed switch to materially wider passage", true);
                publishPath(continuousActivePath(*raw_message));
                publishSpeedCap(activeSpeedCap());
                publishStatus(
                  "READY", "pre-commit scan refinement selected the wider side",
                  activeTrajectoryMode(), active_plan_.side,
                  raw_age, scan_age, valid_beam_ratio, &obstacle, refined);
                return;
              }
            } else {
              alternative_side_ = 0;
              alternative_side_win_cycles_ = 0;
              // A clearly better centre on the same side may be accepted
              // once; the decision deadline is preserved so refinement cannot
              // postpone commitment indefinitely.
              if (refined && refined->side == active_plan_.side &&
                refined->min_clearance >= active_plan_.minimum_clearance +
                0.5 * side_switch_clearance_advantage_m_ &&
                std::abs(refined->peak_offset - active_plan_.peak_offset) >=
                0.5 * candidate_lateral_step_m_)
              {
                activatePlan(
                  *refined, "early same-side refinement toward safer corridor centre", true);
                publishPath(continuousActivePath(*raw_message));
                publishSpeedCap(activeSpeedCap());
                publishStatus(
                  "READY", "pre-commit scan refinement improved corridor centring",
                  activeTrajectoryMode(), active_plan_.side,
                  raw_age, scan_age, valid_beam_ratio, &obstacle, refined);
                return;
              }
            }
          }
        }

        if (!update_requested) {
          no_safe_path_cycles_ = 0;
          const auto continuous_path = continuousActivePath(*raw_message);
          publishPath(continuous_path);
          if (blocked_now) {
            publishSpeedCap(replan_pending_speed_cap_mps_);
            publishStatus(
              "READY", "active-path blockage awaiting fresh-scan confirmation",
              "REPLAN_PENDING", active_plan_.side,
              raw_age, scan_age, valid_beam_ratio,
              obstacle_found ? &obstacle : nullptr);
          } else {
            publishSpeedCap(activeSpeedCap());
            publishStatus(
              "READY", "holding and trimming map-anchored local plan",
              activeTrajectoryMode(), active_plan_.side,
              raw_age, scan_age, valid_beam_ratio,
              obstacle_found ? &obstacle : nullptr);
          }
          return;
        }

        // A material update request may replace the plan, but never silently
        // flips geometry on every scan.  A new plan receives a new plan_id.
        Candidate replacement;
        Candidate left;
        Candidate right;
        const Candidate * selected = nullptr;
        std::string update_reason;
        if (obstacle_found && enable_detour_planning_ && obstacle_size_valid) {
          left = bestDetourCandidateForSide(
            raw, *raw_message, obstacle, hits, robot_position, robot_yaw,
            robot_on_raw.d, car_heading_error, +1);
          right = bestDetourCandidateForSide(
            raw, *raw_message, obstacle, hits, robot_position, robot_yaw,
            robot_on_raw.d, car_heading_error, -1);
          selected = chooseCandidate(left, right);
          publishMarkers(obstacle, left, right, selected ? selected->side : 0, raw.frame);
          update_reason = "active local path update around observed obstacle";
        } else if (!obstacle_found) {
          clearMarkers(raw.frame);
          replacement = buildRecoveryCandidate(
            raw, *raw_message, hits, robot_position, robot_yaw,
            robot_on_raw.d, car_heading_error);
          selected = replacement.valid ? &replacement : nullptr;
          update_reason = "active local path update toward raceline";
        }

        if (selected) {
          activatePlan(*selected, update_reason);
          publishPath(continuousActivePath(*raw_message));
          publishSpeedCap(activeSpeedCap());
          publishStatus(
            "READY", "active plan was materially updated and assigned a new plan_id",
            activeTrajectoryMode(), active_plan_.side,
            raw_age, scan_age, valid_beam_ratio,
            obstacle_found ? &obstacle : nullptr, selected);
          return;
        }

        if (new_scan) {
          no_safe_path_cycles_ = std::min(
            no_safe_path_cycles_ + 1, no_safe_path_confirmation_scans_);
        }
        const bool update_failure_confirmable =
          blocked_confirmed ||
          stale_plan || excessive_deviation || end_without_rejoin;
        if (update_failure_confirmable &&
          no_safe_path_cycles_ >= no_safe_path_confirmation_scans_)
        {
          failPrimary(
            "NO_SAFE_PATH_CONFIRMED",
            "active plan remained blocked and no valid replacement was found",
            raw_age, scan_age, valid_beam_ratio, obstacle_found ? &obstacle : nullptr);
          return;
        }

        publishPath(continuousActivePath(*raw_message));
        publishSpeedCap(replan_pending_speed_cap_mps_);
        publishStatus(
          "READY", "active-plan update pending multi-scan confirmation",
          "REPLAN_PENDING", active_plan_.side,
          raw_age, scan_age, valid_beam_ratio, obstacle_found ? &obstacle : nullptr);
        return;
      }
    }

    // No active plan: create one when the raw raceline is obstructed or when
    // localization shows that the car has laterally left the raceline band.
    const bool recovery_requested =
      std::abs(robot_on_raw.d) >= recovery_enter_lateral_error_m_;
    if (obstacle_found || recovery_requested) {
      Candidate recovery;
      Candidate left;
      Candidate right;
      const Candidate * selected = nullptr;
      std::string failure_reason;
      if (obstacle_found && enable_detour_planning_ && obstacle_size_valid) {
        left = bestDetourCandidateForSide(
          raw, *raw_message, obstacle, hits, robot_position, robot_yaw,
          robot_on_raw.d, car_heading_error, +1);
        right = bestDetourCandidateForSide(
          raw, *raw_message, obstacle, hits, robot_position, robot_yaw,
          robot_on_raw.d, car_heading_error, -1);
        selected = chooseCandidate(left, right);
        publishMarkers(obstacle, left, right, selected ? selected->side : 0, raw.frame);
        failure_reason = "no valid local passage (left: " + left.reason +
          "; right: " + right.reason + ")";
      } else if (obstacle_found && !enable_detour_planning_) {
        failure_reason = "raw raceline is blocked and local planning is disabled";
      } else if (obstacle_found && !obstacle_size_valid) {
        failure_reason = "interfering cluster exceeds configured obstacle-size rule";
      } else {
        clearMarkers(raw.frame);
        recovery = buildRecoveryCandidate(
          raw, *raw_message, hits, robot_position, robot_yaw,
          robot_on_raw.d, car_heading_error);
        selected = recovery.valid ? &recovery : nullptr;
        failure_reason = "no valid raceline-recovery trajectory: " + recovery.reason;
      }

      if (selected) {
        activatePlan(
          *selected,
          obstacle_found ? "obstacle avoidance requested" : "lateral recovery requested");
        publishPath(continuousActivePath(*raw_message));
        publishSpeedCap(activeSpeedCap());
        publishStatus(
          "READY", "persistent local trajectory accepted",
          activeTrajectoryMode(), active_plan_.side,
          raw_age, scan_age, valid_beam_ratio,
          obstacle_found ? &obstacle : nullptr, selected);
        return;
      }

      if (new_scan) {
        no_safe_path_cycles_ = std::min(
          no_safe_path_cycles_ + 1, no_safe_path_confirmation_scans_);
      }
      if (no_safe_path_cycles_ >= no_safe_path_confirmation_scans_) {
        failPrimary(
          "NO_SAFE_PATH_CONFIRMED", failure_reason,
          raw_age, scan_age, valid_beam_ratio, obstacle_found ? &obstacle : nullptr);
        return;
      }

      // Keep a fresh path heartbeat and slow sharply while confirming a
      // non-critical planning failure.  This avoids a one-scan Reactive latch;
      // the lower emergency layer remains authoritative throughout.
      publishPath(*raw_message);
      publishSpeedCap(replan_pending_speed_cap_mps_);
      publishStatus(
        "READY", failure_reason + "; waiting for confirmation",
        "REPLAN_PENDING", 0, raw_age, scan_age, valid_beam_ratio,
        obstacle_found ? &obstacle : nullptr);
      return;
    }

    no_safe_path_cycles_ = 0;
    active_blocked_cycles_ = 0;
    publishPath(*raw_message);
    publishSpeedCap(normal_speed_cap_mps_);
    publishStatus(
      "READY", "raw local raceline is clear and car is within recovery threshold",
      "RACELINE", 0, raw_age, scan_age, valid_beam_ratio);
    clearMarkers(raw.frame);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalTrajectoryPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
