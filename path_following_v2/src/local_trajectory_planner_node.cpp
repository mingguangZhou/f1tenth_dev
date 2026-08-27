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
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "path_following_v2/active_path_safety.hpp"
#include "path_following_v2/bounded_corridor_smoother.hpp"
#include "path_following_v2/candidate_selection.hpp"
#include "path_following_v2/lattice_clearance.hpp"
#include "path_following_v2/lightweight_frenet_lattice.hpp"
#include "path_following_v2/maneuver_speed_policy.hpp"
#include "path_following_v2/path_splice.hpp"
#include "path_following_v2/polyline_query.hpp"
#include "path_following_v2/scan_pooling.hpp"

using std::placeholders::_1;
namespace selection = path_following_v2::selection;
namespace active_path_safety = path_following_v2::active_path_safety;
namespace maneuver_speed = path_following_v2::maneuver_speed;
namespace path_splice = path_following_v2::path_splice;
namespace polyline_query = path_following_v2::polyline_query;

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
    tf_system_clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)),
    tf_buffer_(tf_system_clock_),
    tf_listener_(tf_buffer_)
  {
    declareParameters();
    loadParameters();
    planning_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    input_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    heartbeat_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions input_options;
    input_options.callback_group = input_callback_group_;
    rclcpp::SubscriptionOptions planning_options;
    planning_options.callback_group = planning_callback_group_;

    raw_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      raw_path_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&LocalTrajectoryPlannerNode::rawPathCallback, this, _1), input_options);
    raceline_reference_sub_ = create_subscription<nav_msgs::msg::Path>(
      raceline_reference_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&LocalTrajectoryPlannerNode::racelineReferenceCallback, this, _1),
      planning_options);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalTrajectoryPlannerNode::scanCallback, this, _1), input_options);
    reset_sub_ = create_subscription<std_msgs::msg::Bool>(
      reset_topic_, 10,
      std::bind(&LocalTrajectoryPlannerNode::resetCallback, this, _1), planning_options);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&LocalTrajectoryPlannerNode::mapCallback, this, _1), planning_options);

    final_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      final_path_topic_, rclcpp::QoS(1).reliable().transient_local());
    speed_cap_pub_ = create_publisher<std_msgs::msg::Float64>(speed_cap_topic_, 10);
    heading_error_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(heading_error_topic_, 10);
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(status_topic_, 10);
    if (publish_markers_) {
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    }

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LocalTrajectoryPlannerNode::controlLoop, this),
      planning_callback_group_);
    heartbeat_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LocalTrajectoryPlannerNode::planningHeartbeatLoop, this),
      heartbeat_callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "local_trajectory_planner ready: raw=%s final=%s scan=%s "
      "physical/planning_clearance=%.2f/%.2f m planning/scan=%.2f/%.2f m "
      "recovery_enter/exit=%.2f/%.2f m",
      raw_path_topic_.c_str(), final_path_topic_.c_str(), scan_topic_.c_str(),
      safety_half_width_m_, planningClearance(), planning_distance_m_, scan_range_cap_m_,
      recovery_enter_lateral_error_m_,
      recovery_exit_lateral_error_m_);
    RCLCPP_INFO(
      get_logger(),
      "lightweight Frenet lattice: %s; static-obstacle fast mode: %s; "
      "waiting for transient reference on %s",
      enable_frenet_lattice_planner_ ? "enabled" : "disabled",
      static_obstacle_fast_mode_ ? "enabled" : "disabled",
      raceline_reference_topic_.c_str());
  }

private:
  struct PathModel
  {
    std::vector<Point2> points;
    std::vector<double> s;
    std::vector<polyline_query::Segment> segments;
    std::string frame;
    double length{0.0};
  };

  struct MapClearanceField
  {
    bool valid{false};
    std::string frame;
    std::size_t width{0};
    std::size_t height{0};
    double resolution{0.0};
    double origin_x{0.0};
    double origin_y{0.0};
    double origin_yaw{0.0};
    double origin_cos{1.0};
    double origin_sin{0.0};
    std::vector<float> clearance_m;
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
    std::size_t support_count{1};
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

  struct ObstacleTrack
  {
    bool valid{false};
    Point2 center;
    Point2 velocity;
    rclcpp::Time updated_at{0, 0, RCL_STEADY_TIME};
    int observations{0};
    int moving_evidence{0};
    int missing_scans{0};
    double longitudinal_speed{0.0};
    bool moving{false};
  };

  struct Candidate
  {
    bool valid{false};
    int side{0};  // +1 left, -1 right
    double peak_offset{0.0};
    double min_clearance{std::numeric_limits<double>::infinity()};
    double max_curvature{0.0};
    double objective_cost{std::numeric_limits<double>::infinity()};
    selection::ObjectiveDomain objective_domain{selection::ObjectiveDomain::NONE};
    double rejoin_s{0.0};
    double start_lateral_offset{0.0};
    std::size_t pass_start_index{0};
    std::size_t pass_end_index{0};
    std::size_t rejoin_index{0};
    int maximum_connected_interference{0};
    bool rolling_pass{false};
    std::string reason{"not evaluated"};
    std::string trajectory_mode{"NONE"};
    std::string planning_reference{"raceline"};
    nav_msgs::msg::Path path;
  };

  struct CandidateSummary
  {
    bool evaluated{false};
    bool valid{false};
    std::string reason{"not evaluated"};
    double minimum_clearance{std::numeric_limits<double>::infinity()};
    double objective_cost{std::numeric_limits<double>::infinity()};
    selection::ObjectiveDomain objective_domain{selection::ObjectiveDomain::NONE};
    double maximum_curvature{0.0};
    double peak_offset{0.0};
    int maximum_connected_interference{0};
  };

  struct SafeYieldPlan
  {
    bool valid{false};
    nav_msgs::msg::Path path;
    double path_length{0.0};
    double collision_distance{std::numeric_limits<double>::infinity()};
    double speed_cap{0.0};
    std::string reason{"not evaluated"};
  };

  enum class ManeuverPhase
  {
    OPEN,
    DEPARTING,
    PASSING,
    RETURNING,
    RECOVERING
  };

  struct ActivePlan
  {
    bool valid{false};
    unsigned long id{0};
    int side{0};
    std::string source_mode{"NONE"};
    std::string planning_reference{"raceline"};
    selection::ObjectiveDomain objective_domain{selection::ObjectiveDomain::NONE};
    std::string reason;
    nav_msgs::msg::Path path;
    std::size_t pass_start_index{0};
    std::size_t pass_end_index{0};
    std::size_t rejoin_index{0};
    std::size_t progress_index{0};
    double minimum_clearance{std::numeric_limits<double>::infinity()};
    double maximum_curvature{0.0};
    double peak_offset{0.0};
    double start_lateral_offset{0.0};
    bool side_committed{true};
    bool rolling_pass{false};
    bool raceline_handoff_started{false};
    ManeuverPhase phase{ManeuverPhase::OPEN};
    rclcpp::Time created_at{0, 0, RCL_STEADY_TIME};
  };

  struct RacelinePlanningContext
  {
    bool valid{false};
    bool obstacle_valid{false};
    std::string reason{"not prepared"};
    PathModel reference;
    nav_msgs::msg::Path reference_message;
    std::vector<ScanHit> hits;
    std::vector<Point2> trusted_points;
    Projection robot_projection;
    double robot_heading_error{0.0};
    ObstacleCluster obstacle;
  };

  struct PlanningHeartbeat
  {
    bool active{false};
    rclcpp::Time started_at{0, 0, RCL_STEADY_TIME};
    nav_msgs::msg::Path path;
    std_msgs::msg::Float64 speed_cap;
    diagnostic_msgs::msg::DiagnosticArray status;
  };

  struct InputFreshness
  {
    bool raw_received{false};
    bool scan_received{false};
    double raw_age{std::numeric_limits<double>::infinity()};
    double scan_age{std::numeric_limits<double>::infinity()};
  };

  struct LatestCandidateValidation
  {
    active_path_safety::CandidateValidationOutcome outcome{
      active_path_safety::CandidateValidationOutcome::INPUT_FAILURE};
    std::string failure_state{"SCAN_INVALID"};
    std::string failure_reason{"candidate has not been validated"};
    std::vector<ScanHit> hits;
    double valid_beam_ratio{0.0};
    rclcpp::Time scan_time{0, 0, RCL_STEADY_TIME};
    Point2 robot_position;
    double robot_yaw{0.0};
  };

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raw_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raceline_reference_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr final_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_cap_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr heading_error_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
  rclcpp::CallbackGroup::SharedPtr input_callback_group_;
  rclcpp::CallbackGroup::SharedPtr heartbeat_callback_group_;
  rclcpp::Clock::SharedPtr tf_system_clock_;
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

  std::mutex planning_heartbeat_mutex_;
  PlanningHeartbeat planning_heartbeat_;

  std::string raw_path_topic_;
  std::string raceline_reference_topic_;
  std::string final_path_topic_;
  std::string scan_topic_;
  std::string speed_cap_topic_;
  std::string heading_error_topic_;
  std::string status_topic_;
  std::string marker_topic_;
  std::string reset_topic_;
  std::string map_topic_;
  std::string robot_frame_;

  double control_rate_hz_{20.0};
  double raw_path_timeout_sec_{0.30};
  double scan_timeout_sec_{0.30};
  double planning_heartbeat_timeout_sec_{1.0};
  double transform_timeout_sec_{0.05};
  double scan_range_cap_m_{10.0};
  double planning_distance_m_{10.0};
  int planning_scan_pool_size_{1};
  double vehicle_width_m_{0.28};
  double lateral_safety_margin_m_{0.10};
  double safety_half_width_m_{0.24};
  double planning_clearance_reserve_m_{0.06};
  double raceline_splice_max_gap_m_{0.15};
  double raceline_splice_max_heading_error_deg_{15.0};
  double min_valid_beam_ratio_{0.35};
  int blocked_min_points_{3};
  int cluster_max_beam_gap_{2};
  double cluster_point_gap_m_{0.18};
  double max_obstacle_size_m_{0.50};
  double obstacle_size_tolerance_m_{0.15};
  double obstacle_track_match_distance_m_{0.80};
  double obstacle_velocity_filter_gain_{0.25};
  double moving_obstacle_min_speed_mps_{0.55};
  int moving_obstacle_confirmation_scans_{4};
  int obstacle_track_drop_scans_{12};
  bool require_map_clearance_{true};
  int map_occupied_threshold_{50};
  double critical_obstacle_distance_m_{0.80};
  double detour_longitudinal_buffer_m_{0.20};
  double detour_post_obstacle_hold_m_{0.80};
  double detour_extra_clearance_m_{0.0};
  double detour_return_min_length_m_{2.50};
  double detour_return_max_length_m_{5.00};
  double candidate_lateral_step_m_{0.08};
  int max_candidates_per_side_{6};
  bool static_obstacle_fast_mode_{false};
  int static_analytic_candidates_per_side_{2};
  double static_analytic_extra_clearance_m_{0.04};
  bool static_first_valid_side_{true};
  double max_lateral_shift_m_{0.80};
  double wheelbase_m_{0.33};
  double steering_max_deg_{20.6};
  double curvature_safety_factor_{0.95};
  std::string raceline_frame_{"map"};
  bool raceline_closed_loop_{false};
  bool require_raceline_reference_{true};
  double raceline_reference_match_tolerance_m_{0.10};
  bool enable_frenet_lattice_planner_{true};
  bool lattice_fallback_to_legacy_planner_{true};
  double lattice_station_step_m_{0.25};
  double lattice_lateral_step_m_{0.05};
  double lattice_max_abs_slope_{0.55};
  int lattice_beam_width_{120};
  int lattice_max_final_candidates_{6};
  double lattice_max_compute_time_ms_{12.0};
  double lattice_reference_weight_{4.0};
  double lattice_slope_weight_{1.0};
  double lattice_curvature_weight_{8.0};
  double lattice_curvature_rate_weight_{2.0};
  double lattice_continuity_weight_{10.0};
  double lattice_clearance_weight_{2.0};
  double lattice_preferred_clearance_m_{0.45};
  double lattice_terminal_distance_m_{1.0};
  double lattice_terminal_weight_multiplier_{5.0};
  double lattice_rejoin_alignment_length_m_{0.75};
  double side_clearance_tie_m_{0.03};
  int corridor_smoothing_iterations_{12};
  double corridor_smoothing_relaxation_{0.85};
  double corridor_reference_weight_{1.0};
  double corridor_continuity_weight_{10.0};
  double corridor_slope_weight_{0.5};
  double corridor_curvature_weight_{10.0};
  double corridor_curvature_rate_weight_{2.0};
  double command_speed_max_mps_{2.0};
  double avoidance_speed_cap_mps_{1.0};
  double recovery_speed_cap_mps_{1.0};
  double replan_pending_speed_cap_mps_{0.4};
  double maneuver_lateral_acceleration_limit_mps2_{2.0};
  bool enable_primary_safe_yield_{true};
  double yield_standoff_m_{1.0};
  double yield_min_path_length_m_{0.50};
  double yield_max_speed_mps_{1.0};
  double yield_deceleration_mps2_{3.0};
  double safe_yield_terminal_speed_mps_{0.05};
  double recovery_enter_lateral_error_m_{0.18};
  double recovery_exit_lateral_error_m_{0.08};
  double recovery_exit_heading_error_deg_{10.0};
  double recovery_min_rejoin_length_m_{2.5};
  double recovery_max_rejoin_length_m_{5.0};
  double recovery_length_gain_{3.0};
  double minimum_plan_hold_sec_{0.50};
  double maximum_plan_hold_sec_{12.0};
  double plan_deviation_replan_m_{0.35};
  int rejoin_confirmation_scans_{4};
  int active_path_blocked_confirmation_scans_{3};
  int no_safe_path_confirmation_scans_{3};
  double rolling_pass_refresh_remaining_m_{4.0};
  double rolling_pass_rear_clearance_m_{0.60};
  int rolling_pass_completion_scans_{4};
  bool enable_detour_planning_{true};
  bool publish_markers_{true};
  double last_published_speed_cap_mps_{0.0};

  ActivePlan active_plan_;
  unsigned long next_plan_id_{1};
  int rejoin_stable_cycles_{0};
  int active_blocked_cycles_{0};
  int active_physical_blocked_cycles_{0};
  int no_safe_path_cycles_{0};
  int terminal_safe_yield_cycles_{0};
  int rolling_pass_clear_cycles_{0};
  double last_effective_detection_distance_m_{0.0};
  rclcpp::Time last_processed_scan_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_terminal_safe_yield_scan_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_no_safe_path_scan_time_{0, 0, RCL_STEADY_TIME};
  std::vector<Point2> raceline_reference_points_;
  std::vector<polyline_query::Segment> raceline_reference_segments_;
  bool raceline_reference_valid_{false};
  std::string last_reference_source_{"raceline"};
  std::size_t last_lattice_evaluated_transitions_{0};
  double last_lattice_compute_time_ms_{0.0};
  double last_lattice_clearance_grid_time_ms_{0.0};
  std::string last_lattice_reason_{"not run"};
  int last_static_analytic_candidates_{0};
  double last_static_analytic_time_ms_{0.0};
  double last_static_side_score_time_ms_{0.0};
  double last_static_total_time_ms_{0.0};
  double last_static_left_map_score_m_{
    -std::numeric_limits<double>::infinity()};
  double last_static_right_map_score_m_{
    -std::numeric_limits<double>::infinity()};
  std::string last_static_backend_{"not run"};
  unsigned long candidate_decision_id_{0};
  std::string candidate_decision_context_{"none"};
  int candidate_decision_selected_side_{0};
  CandidateSummary last_left_candidate_;
  CandidateSummary last_right_candidate_;
  ObstacleTrack obstacle_track_;
  MapClearanceField map_clearance_;

  void declareParameters()
  {
    declare_parameter<std::string>(
      "raw_path_topic", "/path_following_v2/raceline_local_path");
    declare_parameter<std::string>("raceline_reference_topic", "/raceline_path");
    declare_parameter<std::string>("final_path_topic", "/path_following_v2/local_path");
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>(
      "speed_cap_topic", "/path_following_v2/trajectory_speed_cap_mps");
    declare_parameter<std::string>(
      "heading_error_topic", "/path_following_v2/raceline_heading_error_rad");
    declare_parameter<std::string>("status_topic", "/path_following_v2/path_status");
    declare_parameter<std::string>("marker_topic", "/path_following_v2/detour_markers");
    declare_parameter<std::string>("reset_topic", "/path_following_v2/reset");
    declare_parameter<std::string>("map_topic", "/map");
    declare_parameter<std::string>("robot_frame", "base_link");

    declare_parameter<double>("control_rate_hz", 20.0);
    declare_parameter<double>("raw_path_timeout_sec", 0.30);
    declare_parameter<double>("scan_timeout_sec", 0.30);
    declare_parameter<double>("planning_heartbeat_timeout_sec", 1.0);
    declare_parameter<double>("transform_timeout_sec", 0.05);
    // The effective obstacle-trigger distance is bounded by the raw path,
    // scan reach, and requested planning distance, then reduced only when
    // necessary to leave room for a complete, curvature-valid return.
    declare_parameter<double>("scan_range_cap_m", 10.0);
    declare_parameter<double>("planning_distance_m", 10.0);
    // Planning may conservatively min-pool adjacent beams. Candidate
    // acceptance below always reprojects the latest scan at full resolution.
    declare_parameter<int>("planning_scan_pool_size", 1);

    // Common footprint definition.  The effective per-side envelope is
    // vehicle_width / 2 + lateral_safety_margin = 0.24 m by default.
    declare_parameter<double>("vehicle_width_m", 0.28);
    declare_parameter<double>("lateral_safety_margin_m", 0.10);
    declare_parameter<double>("planning_clearance_reserve_m", 0.06);
    declare_parameter<double>("raceline_splice_max_gap_m", 0.15);
    declare_parameter<double>("raceline_splice_max_heading_error_deg", 15.0);
    declare_parameter<double>("min_valid_beam_ratio", 0.35);
    declare_parameter<int>("blocked_min_points", 3);

    // Adjacent LaserScan returns are grouped before applying the competition
    // rule that an obstacle is smaller than 0.5 m x 0.5 m.
    declare_parameter<int>("cluster_max_beam_gap", 2);
    declare_parameter<double>("cluster_point_gap_m", 0.18);
    declare_parameter<double>("max_obstacle_size_m", 0.50);
    declare_parameter<double>("obstacle_size_tolerance_m", 0.15);
    declare_parameter<double>("critical_obstacle_distance_m", 0.80);
    declare_parameter<double>("obstacle_track_match_distance_m", 0.80);
    declare_parameter<double>("obstacle_velocity_filter_gain", 0.25);
    declare_parameter<double>("moving_obstacle_min_speed_mps", 0.55);
    declare_parameter<int>("moving_obstacle_confirmation_scans", 4);
    declare_parameter<int>("obstacle_track_drop_scans", 12);
    declare_parameter<bool>("require_map_clearance", true);
    declare_parameter<int>("map_occupied_threshold", 50);

    declare_parameter<double>("detour_longitudinal_buffer_m", 0.20);
    declare_parameter<double>("detour_post_obstacle_hold_m", 0.80);
    declare_parameter<double>("detour_extra_clearance_m", 0.0);
    declare_parameter<double>("detour_return_min_length_m", 2.50);
    declare_parameter<double>("detour_return_max_length_m", 5.00);
    declare_parameter<double>("candidate_lateral_step_m", 0.08);
    declare_parameter<int>("max_candidates_per_side", 6);
    declare_parameter<bool>("static_obstacle_fast_mode", false);
    declare_parameter<int>("static_analytic_candidates_per_side", 2);
    declare_parameter<double>("static_analytic_extra_clearance_m", 0.04);
    declare_parameter<bool>("static_first_valid_side", true);
    declare_parameter<double>("max_lateral_shift_m", 0.80);
    declare_parameter<double>("wheelbase_m", 0.33);
    declare_parameter<double>("steering_max_deg", 20.6);
    declare_parameter<double>("curvature_safety_factor", 0.95);

    // The global path emitted by the racing-line CSV publisher is the sole
    // Frenet reference. Consuming the transient-local topic avoids parsing the
    // same CSV twice with potentially different direction/closure semantics.
    declare_parameter<bool>("require_raceline_reference", true);
    declare_parameter<double>("raceline_reference_match_tolerance_m", 0.10);
    declare_parameter<bool>("enable_frenet_lattice_planner", true);
    declare_parameter<bool>("lattice_fallback_to_legacy_planner", true);
    declare_parameter<double>("lattice_station_step_m", 0.25);
    declare_parameter<double>("lattice_lateral_step_m", 0.05);
    declare_parameter<double>("lattice_max_abs_slope", 0.55);
    declare_parameter<int>("lattice_beam_width", 120);
    declare_parameter<int>("lattice_max_final_candidates", 6);
    declare_parameter<double>("lattice_max_compute_time_ms", 12.0);
    declare_parameter<double>("lattice_reference_weight", 4.0);
    declare_parameter<double>("lattice_slope_weight", 1.0);
    declare_parameter<double>("lattice_curvature_weight", 8.0);
    declare_parameter<double>("lattice_curvature_rate_weight", 2.0);
    declare_parameter<double>("lattice_continuity_weight", 10.0);
    declare_parameter<double>("lattice_clearance_weight", 2.0);
    declare_parameter<double>("lattice_preferred_clearance_m", 0.45);
    declare_parameter<double>("lattice_terminal_distance_m", 1.0);
    declare_parameter<double>("lattice_terminal_weight_multiplier", 5.0);
    declare_parameter<double>("lattice_rejoin_alignment_length_m", 0.75);
    declare_parameter<double>("side_clearance_tie_m", 0.03);
    declare_parameter<int>("corridor_smoothing_iterations", 12);
    declare_parameter<double>("corridor_smoothing_relaxation", 0.85);
    declare_parameter<double>("corridor_reference_weight", 1.0);
    declare_parameter<double>("corridor_continuity_weight", 10.0);
    declare_parameter<double>("corridor_slope_weight", 0.5);
    declare_parameter<double>("corridor_curvature_weight", 10.0);
    declare_parameter<double>("corridor_curvature_rate_weight", 2.0);
    // Both shared envelope parameters are declared so the common YAML wildcard
    // is explicit for this node. Planning uses only the maximum as its normal cap.
    declare_parameter<double>("command_speed_min_mps", 0.0);
    declare_parameter<double>("command_speed_max_mps", 2.0);
    // Temporary maneuver caps; these do not generate normal raceline speed.
    declare_parameter<double>("avoidance_speed_cap_mps", 1.0);
    declare_parameter<double>("recovery_speed_cap_mps", 1.0);
    declare_parameter<double>("replan_pending_speed_cap_mps", 0.4);
    declare_parameter<double>("maneuver_lateral_acceleration_limit_mps2", 2.0);
    // Keep safe waiting inside the primary planner.
    declare_parameter<bool>("enable_primary_safe_yield", true);
    declare_parameter<double>("yield_standoff_m", 1.0);
    declare_parameter<double>("yield_min_path_length_m", 0.50);
    declare_parameter<double>("yield_max_speed_mps", 1.0);
    declare_parameter<double>("yield_deceleration_mps2", 3.0);
    declare_parameter<double>("safe_yield_terminal_speed_mps", 0.05);

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
    declare_parameter<int>("rejoin_confirmation_scans", 4);
    declare_parameter<int>("active_path_blocked_confirmation_scans", 4);
    declare_parameter<int>("no_safe_path_confirmation_scans", 5);
    declare_parameter<double>("rolling_pass_refresh_remaining_m", 4.0);
    declare_parameter<double>("rolling_pass_rear_clearance_m", 0.60);
    declare_parameter<int>("rolling_pass_completion_scans", 4);
    declare_parameter<bool>("enable_detour_planning", true);
    declare_parameter<bool>("publish_markers", true);
  }

  void loadParameters()
  {
    raw_path_topic_ = get_parameter("raw_path_topic").as_string();
    raceline_reference_topic_ = get_parameter("raceline_reference_topic").as_string();
    final_path_topic_ = get_parameter("final_path_topic").as_string();
    scan_topic_ = get_parameter("scan_topic").as_string();
    speed_cap_topic_ = get_parameter("speed_cap_topic").as_string();
    heading_error_topic_ = get_parameter("heading_error_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    marker_topic_ = get_parameter("marker_topic").as_string();
    reset_topic_ = get_parameter("reset_topic").as_string();
    map_topic_ = get_parameter("map_topic").as_string();
    robot_frame_ = get_parameter("robot_frame").as_string();
    control_rate_hz_ = std::max(1.0, get_parameter("control_rate_hz").as_double());
    raw_path_timeout_sec_ = std::max(0.01, get_parameter("raw_path_timeout_sec").as_double());
    scan_timeout_sec_ = std::max(0.01, get_parameter("scan_timeout_sec").as_double());
    planning_heartbeat_timeout_sec_ = std::max(
      0.10, get_parameter("planning_heartbeat_timeout_sec").as_double());
    transform_timeout_sec_ = std::max(0.0, get_parameter("transform_timeout_sec").as_double());
    scan_range_cap_m_ = std::max(0.10, get_parameter("scan_range_cap_m").as_double());
    planning_distance_m_ = std::max(0.50, get_parameter("planning_distance_m").as_double());
    planning_scan_pool_size_ = std::clamp(
      static_cast<int>(get_parameter("planning_scan_pool_size").as_int()), 1, 8);
    vehicle_width_m_ = std::max(0.01, get_parameter("vehicle_width_m").as_double());
    lateral_safety_margin_m_ = std::max(
      0.0, get_parameter("lateral_safety_margin_m").as_double());
    safety_half_width_m_ = 0.5 * vehicle_width_m_ + lateral_safety_margin_m_;
    planning_clearance_reserve_m_ = std::max(
      0.0, get_parameter("planning_clearance_reserve_m").as_double());
    raceline_splice_max_gap_m_ = std::max(
      0.01, get_parameter("raceline_splice_max_gap_m").as_double());
    raceline_splice_max_heading_error_deg_ = std::clamp(
      get_parameter("raceline_splice_max_heading_error_deg").as_double(), 1.0, 90.0);
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
    obstacle_track_match_distance_m_ = std::max(
      0.20, get_parameter("obstacle_track_match_distance_m").as_double());
    obstacle_velocity_filter_gain_ = std::clamp(
      get_parameter("obstacle_velocity_filter_gain").as_double(), 0.01, 1.0);
    moving_obstacle_min_speed_mps_ = std::max(
      0.10, get_parameter("moving_obstacle_min_speed_mps").as_double());
    moving_obstacle_confirmation_scans_ = std::max(
      2, static_cast<int>(get_parameter("moving_obstacle_confirmation_scans").as_int()));
    obstacle_track_drop_scans_ = std::max(
      moving_obstacle_confirmation_scans_ + 1,
      static_cast<int>(get_parameter("obstacle_track_drop_scans").as_int()));
    require_map_clearance_ = get_parameter("require_map_clearance").as_bool();
    map_occupied_threshold_ = std::clamp(
      static_cast<int>(get_parameter("map_occupied_threshold").as_int()), 1, 100);
    detour_longitudinal_buffer_m_ = std::max(
      0.0, get_parameter("detour_longitudinal_buffer_m").as_double());
    detour_post_obstacle_hold_m_ = std::max(
      0.0, get_parameter("detour_post_obstacle_hold_m").as_double());
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
    static_obstacle_fast_mode_ =
      get_parameter("static_obstacle_fast_mode").as_bool();
    static_analytic_candidates_per_side_ = std::clamp(
      static_cast<int>(get_parameter("static_analytic_candidates_per_side").as_int()),
      1, 2);
    static_analytic_extra_clearance_m_ = std::max(
      0.0, get_parameter("static_analytic_extra_clearance_m").as_double());
    static_first_valid_side_ = get_parameter("static_first_valid_side").as_bool();
    max_lateral_shift_m_ = std::max(
      safety_half_width_m_, get_parameter("max_lateral_shift_m").as_double());
    wheelbase_m_ = std::max(0.01, get_parameter("wheelbase_m").as_double());
    steering_max_deg_ = std::max(0.1, get_parameter("steering_max_deg").as_double());
    curvature_safety_factor_ = std::clamp(
      get_parameter("curvature_safety_factor").as_double(), 0.1, 100.0);
    require_raceline_reference_ = get_parameter("require_raceline_reference").as_bool();
    raceline_reference_match_tolerance_m_ = std::max(
      0.01, get_parameter("raceline_reference_match_tolerance_m").as_double());
    enable_frenet_lattice_planner_ =
      get_parameter("enable_frenet_lattice_planner").as_bool();
    lattice_fallback_to_legacy_planner_ =
      get_parameter("lattice_fallback_to_legacy_planner").as_bool();
    lattice_station_step_m_ = std::max(
      0.10, get_parameter("lattice_station_step_m").as_double());
    lattice_lateral_step_m_ = std::max(
      0.02, get_parameter("lattice_lateral_step_m").as_double());
    lattice_max_abs_slope_ = std::max(
      0.05, get_parameter("lattice_max_abs_slope").as_double());
    lattice_beam_width_ = std::max(
      10, static_cast<int>(get_parameter("lattice_beam_width").as_int()));
    lattice_max_final_candidates_ = std::max(
      1, static_cast<int>(get_parameter("lattice_max_final_candidates").as_int()));
    lattice_max_compute_time_ms_ = std::max(
      1.0, get_parameter("lattice_max_compute_time_ms").as_double());
    lattice_reference_weight_ = std::max(
      0.0, get_parameter("lattice_reference_weight").as_double());
    lattice_slope_weight_ = std::max(
      0.0, get_parameter("lattice_slope_weight").as_double());
    lattice_curvature_weight_ = std::max(
      0.0, get_parameter("lattice_curvature_weight").as_double());
    lattice_curvature_rate_weight_ = std::max(
      0.0, get_parameter("lattice_curvature_rate_weight").as_double());
    lattice_continuity_weight_ = std::max(
      0.0, get_parameter("lattice_continuity_weight").as_double());
    lattice_clearance_weight_ = std::max(
      0.0, get_parameter("lattice_clearance_weight").as_double());
    lattice_preferred_clearance_m_ = std::max(
      planningClearance(), get_parameter("lattice_preferred_clearance_m").as_double());
    lattice_terminal_distance_m_ = std::max(
      0.0, get_parameter("lattice_terminal_distance_m").as_double());
    lattice_terminal_weight_multiplier_ = std::max(
      1.0, get_parameter("lattice_terminal_weight_multiplier").as_double());
    lattice_rejoin_alignment_length_m_ = std::max(
      2.0 * lattice_station_step_m_,
      get_parameter("lattice_rejoin_alignment_length_m").as_double());
    side_clearance_tie_m_ = std::max(
      0.0, get_parameter("side_clearance_tie_m").as_double());
    corridor_smoothing_iterations_ = std::max(
      0, static_cast<int>(get_parameter("corridor_smoothing_iterations").as_int()));
    corridor_smoothing_relaxation_ = std::clamp(
      get_parameter("corridor_smoothing_relaxation").as_double(), 0.05, 1.0);
    corridor_reference_weight_ = std::max(
      0.0, get_parameter("corridor_reference_weight").as_double());
    corridor_continuity_weight_ = std::max(
      0.0, get_parameter("corridor_continuity_weight").as_double());
    corridor_slope_weight_ = std::max(
      0.0, get_parameter("corridor_slope_weight").as_double());
    corridor_curvature_weight_ = std::max(
      0.0, get_parameter("corridor_curvature_weight").as_double());
    corridor_curvature_rate_weight_ = std::max(
      0.0, get_parameter("corridor_curvature_rate_weight").as_double());
    command_speed_max_mps_ = std::max(
      0.0, get_parameter("command_speed_max_mps").as_double());
    avoidance_speed_cap_mps_ = std::clamp(
      get_parameter("avoidance_speed_cap_mps").as_double(), 0.0, command_speed_max_mps_);
    recovery_speed_cap_mps_ = std::clamp(
      get_parameter("recovery_speed_cap_mps").as_double(), 0.0, command_speed_max_mps_);
    replan_pending_speed_cap_mps_ = std::clamp(
      get_parameter("replan_pending_speed_cap_mps").as_double(),
      0.0, command_speed_max_mps_);
    maneuver_lateral_acceleration_limit_mps2_ = std::max(
      0.1, get_parameter("maneuver_lateral_acceleration_limit_mps2").as_double());
    enable_primary_safe_yield_ = get_parameter("enable_primary_safe_yield").as_bool();
    yield_standoff_m_ = std::max(
      planningClearance(), get_parameter("yield_standoff_m").as_double());
    yield_min_path_length_m_ = std::max(
      0.30, get_parameter("yield_min_path_length_m").as_double());
    yield_max_speed_mps_ = std::clamp(
      get_parameter("yield_max_speed_mps").as_double(), 0.0, command_speed_max_mps_);
    yield_deceleration_mps2_ = std::max(
      0.1, get_parameter("yield_deceleration_mps2").as_double());
    safe_yield_terminal_speed_mps_ = std::max(
      0.0, get_parameter("safe_yield_terminal_speed_mps").as_double());
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
    rejoin_confirmation_scans_ = std::max(
      1, static_cast<int>(get_parameter("rejoin_confirmation_scans").as_int()));
    active_path_blocked_confirmation_scans_ = std::max(
      1, static_cast<int>(
        get_parameter("active_path_blocked_confirmation_scans").as_int()));
    no_safe_path_confirmation_scans_ = std::max(
      1, static_cast<int>(get_parameter("no_safe_path_confirmation_scans").as_int()));
    rolling_pass_refresh_remaining_m_ = std::max(
      1.0, get_parameter("rolling_pass_refresh_remaining_m").as_double());
    rolling_pass_rear_clearance_m_ = std::max(
      safety_half_width_m_, get_parameter("rolling_pass_rear_clearance_m").as_double());
    rolling_pass_completion_scans_ = std::max(
      2, static_cast<int>(get_parameter("rolling_pass_completion_scans").as_int()));
    enable_detour_planning_ = get_parameter("enable_detour_planning").as_bool();
    publish_markers_ = get_parameter("publish_markers").as_bool();
  }

  void racelineReferenceCallback(const nav_msgs::msg::Path::SharedPtr path)
  {
    if (!path || path->header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "received global racing line without a frame");
      return;
    }

    std::vector<Point2> points;
    points.reserve(path->poses.size());
    for (const auto & pose : path->poses) {
      const Point2 point{pose.pose.position.x, pose.pose.position.y};
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        RCLCPP_WARN(get_logger(), "received non-finite global racing-line point");
        return;
      }
      if (points.empty() || distance(points.back(), point) > 1e-6) {
        points.push_back(point);
      }
    }
    // The publisher owns loop-closure policy and represents a closed path by
    // appending an exact copy of the first point. Only remove that explicit
    // duplicate; do not reinterpret two distinct nearby endpoints here.
    const bool closed_loop = points.size() >= 3 &&
      distance(points.front(), points.back()) <= 1e-6;
    if (closed_loop) {
      points.pop_back();
    }
    if (points.size() < 3) {
      RCLCPP_WARN(get_logger(), "global racing-line path has fewer than three usable points");
      return;
    }

    bool changed = !raceline_reference_valid_ ||
      raceline_frame_ != path->header.frame_id ||
      raceline_closed_loop_ != closed_loop ||
      raceline_reference_points_.size() != points.size();
    if (!changed) {
      for (std::size_t index = 0; index < points.size(); ++index) {
        if (distance(raceline_reference_points_[index], points[index]) > 1e-6) {
          changed = true;
          break;
        }
      }
    }
    if (!changed) {
      return;
    }

    if (raceline_reference_valid_ && active_plan_.valid) {
      releaseActivePlan("global racing-line Frenet reference changed");
    }
    raceline_reference_points_ = std::move(points);
    raceline_reference_segments_ = polyline_query::prepareSegments(
      raceline_reference_points_, closed_loop);
    raceline_frame_ = path->header.frame_id;
    raceline_closed_loop_ = closed_loop;
    raceline_reference_valid_ = true;
    RCLCPP_INFO(
      get_logger(), "accepted %zu-point global racing-line Frenet reference from %s",
      raceline_reference_points_.size(), raceline_reference_topic_.c_str());
  }

  Projection projectToRacelineReference(const Point2 & point) const
  {
    Projection best;
    if (raceline_reference_points_.size() < 2 || raceline_reference_segments_.empty()) {
      return best;
    }
    const auto nearest = polyline_query::nearestPoint(
      point.x, point.y, raceline_reference_segments_, 0,
      raceline_reference_segments_.size(), true);
    if (!nearest.valid) {
      return best;
    }
    const auto & segment = raceline_reference_segments_[nearest.segment_index];
    best.valid = true;
    best.s = nearest.ratio * segment.length;
    best.d = (nearest.residual_x * (-segment.dy) +
      nearest.residual_y * segment.dx) / segment.length;
    best.yaw = segment.yaw;
    best.distance = nearest.distance;
    best.segment_index = nearest.segment_index;
    return best;
  }

  bool buildLocalRacelineReference(
    const Point2 & robot_position, const double target_length,
    PathModel & model, nav_msgs::msg::Path & path_message,
    Projection & robot_projection) const
  {
    model = PathModel{};
    path_message = nav_msgs::msg::Path{};
    robot_projection = projectToRacelineReference(robot_position);
    if (!raceline_reference_valid_ || !robot_projection.valid || target_length < 0.5) {
      return false;
    }

    const std::size_t size = raceline_reference_points_.size();
    const std::size_t segment_index = robot_projection.segment_index;
    const Point2 & a = raceline_reference_points_[segment_index];
    const Point2 & b = raceline_reference_points_[(segment_index + 1) % size];
    const double segment_length = distance(a, b);
    const double ratio = segment_length > 1e-9 ?
      std::clamp(robot_projection.s / segment_length, 0.0, 1.0) : 0.0;
    const Point2 anchor{
      a.x + ratio * (b.x - a.x),
      a.y + ratio * (b.y - a.y)};

    model.frame = raceline_frame_;
    model.points.push_back(anchor);
    model.s.push_back(0.0);
    std::size_t next_index = (segment_index + 1) % size;
    const std::size_t maximum_points = raceline_closed_loop_ ? size + 1 : size;
    for (std::size_t count = 0; count < maximum_points && model.length < target_length; ++count) {
      if (!raceline_closed_loop_ && next_index >= size) {
        break;
      }
      const Point2 point = raceline_reference_points_[next_index % size];
      const double segment = distance(model.points.back(), point);
      if (segment > 1e-6) {
        model.length += segment;
        model.points.push_back(point);
        model.s.push_back(model.length);
      }
      if (!raceline_closed_loop_ && next_index + 1 >= size) {
        break;
      }
      next_index = (next_index + 1) % size;
    }
    if (model.points.size() < 3 || model.length < std::min(1.0, target_length)) {
      return false;
    }
    model.segments = polyline_query::prepareSegments(model.points);

    path_message.header.frame_id = model.frame;
    path_message.header.stamp = now();
    path_message.poses.resize(model.points.size());
    for (std::size_t index = 0; index < model.points.size(); ++index) {
      const std::size_t first = index == 0 ? 0 : index - 1;
      const std::size_t second = index + 1 < model.points.size() ? index + 1 : index;
      auto & pose = path_message.poses[index];
      pose.header = path_message.header;
      pose.pose.position.x = model.points[index].x;
      pose.pose.position.y = model.points[index].y;
      pose.pose.orientation = yawToQuaternion(std::atan2(
        model.points[second].y - model.points[first].y,
        model.points[second].x - model.points[first].x));
    }
    return true;
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

  void distanceTransform1D(
    const std::vector<double> & input, std::vector<double> & output) const
  {
    const int count = static_cast<int>(input.size());
    std::vector<int> sites(static_cast<std::size_t>(count));
    std::vector<double> boundaries(static_cast<std::size_t>(count + 1));
    int envelope = 0;
    sites[0] = 0;
    boundaries[0] = -std::numeric_limits<double>::infinity();
    boundaries[1] = std::numeric_limits<double>::infinity();
    for (int query = 1; query < count; ++query) {
      double intersection = 0.0;
      do {
        const int site = sites[envelope];
        intersection =
          ((input[static_cast<std::size_t>(query)] + query * query) -
          (input[static_cast<std::size_t>(site)] + site * site)) /
          (2.0 * static_cast<double>(query - site));
        if (intersection <= boundaries[envelope]) {
          --envelope;
        } else {
          break;
        }
      } while (envelope >= 0);
      ++envelope;
      sites[envelope] = query;
      boundaries[envelope] = intersection;
      boundaries[envelope + 1] = std::numeric_limits<double>::infinity();
    }
    envelope = 0;
    output.resize(input.size());
    for (int query = 0; query < count; ++query) {
      while (boundaries[envelope + 1] < query) {
        ++envelope;
      }
      const double delta = static_cast<double>(query - sites[envelope]);
      output[static_cast<std::size_t>(query)] = delta * delta +
        input[static_cast<std::size_t>(sites[envelope])];
    }
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
  {
    MapClearanceField field;
    field.frame = map->header.frame_id;
    field.width = map->info.width;
    field.height = map->info.height;
    field.resolution = map->info.resolution;
    field.origin_x = map->info.origin.position.x;
    field.origin_y = map->info.origin.position.y;
    tf2::Quaternion orientation;
    tf2::fromMsg(map->info.origin.orientation, orientation);
    double roll = 0.0;
    double pitch = 0.0;
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, field.origin_yaw);
    field.origin_cos = std::cos(field.origin_yaw);
    field.origin_sin = std::sin(field.origin_yaw);
    const std::size_t cell_count = field.width * field.height;
    if (field.frame.empty() || field.width == 0 || field.height == 0 ||
      field.resolution <= 0.0 || map->data.size() != cell_count)
    {
      RCLCPP_WARN(get_logger(), "received malformed occupancy map; corridor constraint unavailable");
      map_clearance_ = MapClearanceField();
      return;
    }

    constexpr double far_squared = 1e12;
    std::vector<double> vertical(cell_count, far_squared);
    for (std::size_t index = 0; index < cell_count; ++index) {
      const int occupancy = static_cast<int>(map->data[index]);
      if (occupancy < 0 || occupancy >= map_occupied_threshold_) {
        vertical[index] = 0.0;
      }
    }
    if (std::all_of(vertical.begin(), vertical.end(), [](const double value) {
        return value >= far_squared;
      }))
    {
      RCLCPP_WARN(get_logger(), "occupancy map contains no boundary cells");
      map_clearance_ = MapClearanceField();
      return;
    }

    std::vector<double> input;
    std::vector<double> output;
    input.resize(field.height);
    for (std::size_t x = 0; x < field.width; ++x) {
      for (std::size_t y = 0; y < field.height; ++y) {
        input[y] = vertical[y * field.width + x];
      }
      distanceTransform1D(input, output);
      for (std::size_t y = 0; y < field.height; ++y) {
        vertical[y * field.width + x] = output[y];
      }
    }
    field.clearance_m.resize(cell_count);
    input.resize(field.width);
    const double cell_radius = std::sqrt(0.5) * field.resolution;
    for (std::size_t y = 0; y < field.height; ++y) {
      for (std::size_t x = 0; x < field.width; ++x) {
        input[x] = vertical[y * field.width + x];
      }
      distanceTransform1D(input, output);
      for (std::size_t x = 0; x < field.width; ++x) {
        const double center_distance = std::sqrt(output[x]) * field.resolution;
        field.clearance_m[y * field.width + x] = static_cast<float>(
          std::max(0.0, center_distance - cell_radius));
      }
    }
    field.valid = true;
    map_clearance_ = std::move(field);
    RCLCPP_INFO(
      get_logger(), "map corridor constraint ready: %zux%zu at %.3f m/cell",
      map_clearance_.width, map_clearance_.height, map_clearance_.resolution);
  }

  double mapClearanceAtUnchecked(const Point2 & point) const
  {
    const double translated_x = point.x - map_clearance_.origin_x;
    const double translated_y = point.y - map_clearance_.origin_y;
    const double local_x =
      map_clearance_.origin_cos * translated_x + map_clearance_.origin_sin * translated_y;
    const double local_y =
      -map_clearance_.origin_sin * translated_x + map_clearance_.origin_cos * translated_y;
    const int cell_x = static_cast<int>(std::floor(local_x / map_clearance_.resolution));
    const int cell_y = static_cast<int>(std::floor(local_y / map_clearance_.resolution));
    if (cell_x < 0 || cell_y < 0 ||
      cell_x >= static_cast<int>(map_clearance_.width) ||
      cell_y >= static_cast<int>(map_clearance_.height))
    {
      return -std::numeric_limits<double>::infinity();
    }
    const std::size_t index = static_cast<std::size_t>(cell_y) * map_clearance_.width +
      static_cast<std::size_t>(cell_x);
    return static_cast<double>(map_clearance_.clearance_m[index]);
  }

  double mapClearanceAt(const Point2 & point, const std::string & frame) const
  {
    if (!map_clearance_.valid || frame != map_clearance_.frame) {
      return -std::numeric_limits<double>::infinity();
    }
    return mapClearanceAtUnchecked(point);
  }

  void resetCallback(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (!message->data) {
      return;
    }
    releaseActivePlan("explicit planner reset");
    rejoin_stable_cycles_ = 0;
    active_blocked_cycles_ = 0;
    active_physical_blocked_cycles_ = 0;
    resetNoSafePathConfirmation();
    terminal_safe_yield_cycles_ = 0;
    last_terminal_safe_yield_scan_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
    rolling_pass_clear_cycles_ = 0;
    obstacle_track_ = ObstacleTrack();
    last_processed_scan_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
    candidate_decision_context_ = "reset";
    candidate_decision_selected_side_ = 0;
    last_left_candidate_ = CandidateSummary();
    last_right_candidate_ = CandidateSummary();
    RCLCPP_INFO(get_logger(), "planner state reset from %s", reset_topic_.c_str());
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
    model.segments = polyline_query::prepareSegments(model.points);
    return model.length >= std::min(1.0, planning_distance_m_);
  }

  double planningClearance() const
  {
    return safety_half_width_m_ + planning_clearance_reserve_m_;
  }

  double effectiveObstacleDetectionDistance(const PathModel & raw) const
  {
    // Do not trigger a detour so far ahead that the current raw path cannot
    // contain the obstacle model plus the minimum smooth return.  This keeps
    // the early trigger useful instead of publishing a predictable
    // NO_SAFE_PATH while the obstacle is still far away.
    const double complete_plan_room =
      raw.length - 2.0 * detour_longitudinal_buffer_m_ -
      max_obstacle_size_m_ - detour_post_obstacle_hold_m_ -
      detour_return_min_length_m_ -
      (enable_frenet_lattice_planner_ ? lattice_rejoin_alignment_length_m_ : 0.0) -
      0.20;
    return std::max(0.0, std::min({
        planning_distance_m_, scan_range_cap_m_, complete_plan_room}));
  }

  Projection projectToPath(const Point2 & point, const PathModel & path) const
  {
    Projection best;
    // Restrict Frenet association to the planning region.  This matters on a
    // small closed track where a later part of the raw window can be
    // spatially close to the current part of the lap.
    const double projection_limit = std::min(
      path.length, planning_distance_m_ + max_obstacle_size_m_);
    std::size_t projection_end = 0;
    for (; projection_end < path.segments.size(); ++projection_end) {
      if (path.s[projection_end] > projection_limit) {
        break;
      }
    }
    const auto nearest = polyline_query::nearestPoint(
      point.x, point.y, path.segments, 0, projection_end, true);
    if (nearest.valid) {
      const auto & segment = path.segments[nearest.segment_index];
      best.valid = true;
      best.distance = nearest.distance;
      best.s = path.s[nearest.segment_index] + nearest.ratio * segment.length;
      best.d = (segment.dx * nearest.residual_y -
        segment.dy * nearest.residual_x) / segment.length;  // positive is path-left
      best.yaw = segment.yaw;
      best.segment_index = nearest.segment_index;
    }
    // Preserve negative s for returns behind the path start.
    if (best.valid && best.segment_index == 0 && best.s <= 1e-9) {
      const Point2 & first = path.points[0];
      const Point2 & second = path.points[1];
      const double dx = second.x - first.x;
      const double dy = second.y - first.y;
      const double segment_length = std::hypot(dx, dy);
      if (segment_length > 1e-9) {
        const double px = point.x - first.x;
        const double py = point.y - first.y;
        const double signed_s = (px * dx + py * dy) / segment_length;
        if (signed_s < 0.0) {
          best.s = signed_s;
          best.d = (dx * py - dy * px) / segment_length;
          best.distance = std::abs(best.d);
        }
      }
    }
    return best;
  }

  bool lookupRobotPose(
    const std::string & frame, const builtin_interfaces::msg::Time & source_stamp,
    Point2 & position, double & yaw)
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        frame, robot_frame_, rclcpp::Time(source_stamp),
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
    std::vector<ScanHit> & hits, double & valid_beam_ratio,
    const std::size_t pool_size = 1)
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
        path.frame, scan.header.frame_id, rclcpp::Time(scan.header.stamp),
        tf2::durationFromSec(transform_timeout_sec_));
      tf2::fromMsg(transform.transform, path_from_scan);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 2000, "local trajectory scan TF failed: %s", exception.what());
      return false;
    }

    const double configured_min = std::isfinite(scan.range_min) ? scan.range_min : 0.0;
    const double sensor_max = std::isfinite(scan.range_max) && scan.range_max > 0.0 ?
      scan.range_max : scan_range_cap_m_;
    const double usable_max = std::min(sensor_max, scan_range_cap_m_);
    const auto pooled = path_following_v2::scan_pooling::minimumRangePool(
      scan.ranges,
      path_following_v2::scan_pooling::RangeLimits{configured_min, usable_max},
      pool_size);
    hits.reserve(pooled.beams.size());
    for (const auto & beam : pooled.beams) {
      const double angle = scan.angle_min +
        static_cast<double>(beam.source_index) * scan.angle_increment;
      const tf2::Vector3 transformed = path_from_scan * tf2::Vector3(
        beam.range_m * std::cos(angle), beam.range_m * std::sin(angle), 0.0);
      ScanHit hit;
      hit.point = Point2{transformed.x(), transformed.y()};
      hit.projection = projectToPath(hit.point, path);
      // Use the bin ordinal for pooled scans so adjacent bins remain adjacent
      // even when pool_size exceeds cluster_max_beam_gap_. Full-resolution
      // scans have pool_size == 1 and retain their exact raw beam indices.
      hit.beam_index = beam.bin_start_index / pooled.pool_size;
      hit.support_count = beam.support_count;
      hit.range = beam.range_m;
      if (hit.projection.valid) {
        hits.push_back(hit);
      }
    }
    valid_beam_ratio = pooled.validBeamRatio();
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
        cluster.interfering_points += static_cast<int>(hit.support_count);
      }
    }
    return clusters;
  }

  Point2 obstacleCenter(const ObstacleCluster & obstacle) const
  {
    Point2 center;
    if (obstacle.hits.empty()) {
      return center;
    }
    std::size_t support = 0;
    for (const auto & hit : obstacle.hits) {
      center.x += static_cast<double>(hit.support_count) * hit.point.x;
      center.y += static_cast<double>(hit.support_count) * hit.point.y;
      support += hit.support_count;
    }
    const double count = static_cast<double>(std::max<std::size_t>(1, support));
    center.x /= count;
    center.y /= count;
    return center;
  }

  std::size_t scanSupportCount(const std::vector<ScanHit> & hits) const
  {
    std::size_t count = 0;
    for (const auto & hit : hits) {
      count += hit.support_count;
    }
    return count;
  }

  std::pair<double, double> conservativeObstacleLateralExtent(
    const ObstacleCluster & obstacle) const
  {
    const double center = 0.5 * (obstacle.d_min + obstacle.d_max);
    const double observed_half_width = 0.5 * (obstacle.d_max - obstacle.d_min);
    const double modeled_half_width = std::max(
      observed_half_width, 0.5 * max_obstacle_size_m_);
    return {center - modeled_half_width, center + modeled_half_width};
  }

  bool trackedObstacleMatches(const ObstacleCluster & obstacle) const
  {
    return obstacle_track_.valid && !obstacle.hits.empty() &&
           distance(obstacleCenter(obstacle), obstacle_track_.center) <=
           obstacle_track_match_distance_m_;
  }

  void beginObstacleTrack(
    const ObstacleCluster & obstacle, const PathModel & path,
    const rclcpp::Time & stamp)
  {
    obstacle_track_ = ObstacleTrack();
    obstacle_track_.valid = true;
    obstacle_track_.center = obstacleCenter(obstacle);
    obstacle_track_.updated_at = stamp;
    obstacle_track_.observations = 1;
    const Projection projection = projectToPath(obstacle_track_.center, path);
    obstacle_track_.longitudinal_speed = projection.valid ? 0.0 :
      std::numeric_limits<double>::quiet_NaN();
  }

  void updateObstacleTrack(
    const std::vector<ObstacleCluster> & clusters,
    const ObstacleCluster * selected_obstacle, const PathModel & path,
    const rclcpp::Time & stamp, const bool new_scan)
  {
    if (!new_scan) {
      return;
    }

    const ObstacleCluster * observation = nullptr;
    if (obstacle_track_.valid) {
      const double elapsed = std::clamp(
        (stamp - obstacle_track_.updated_at).seconds(), 0.0, 0.50);
      const Point2 predicted{
        obstacle_track_.center.x + elapsed * obstacle_track_.velocity.x,
        obstacle_track_.center.y + elapsed * obstacle_track_.velocity.y};
      double best_separation = std::numeric_limits<double>::infinity();
      const double gate = obstacle_track_match_distance_m_ +
        elapsed * std::hypot(obstacle_track_.velocity.x, obstacle_track_.velocity.y);
      for (const auto & cluster : clusters) {
        if (scanSupportCount(cluster.hits) <
          static_cast<std::size_t>(blocked_min_points_))
        {
          continue;
        }
        const double separation = distance(predicted, obstacleCenter(cluster));
        if (separation <= gate && separation < best_separation) {
          observation = &cluster;
          best_separation = separation;
        }
      }
    }

    if (!observation && selected_obstacle &&
      (!active_plan_.valid || !active_plan_.rolling_pass ||
      obstacle_track_.missing_scans >= obstacle_track_drop_scans_))
    {
      beginObstacleTrack(*selected_obstacle, path, stamp);
      return;
    }

    if (!observation) {
      if (obstacle_track_.valid) {
        ++obstacle_track_.missing_scans;
        if (!active_plan_.rolling_pass &&
          obstacle_track_.missing_scans >= obstacle_track_drop_scans_)
        {
          obstacle_track_ = ObstacleTrack();
        }
      }
      return;
    }

    const Point2 center = obstacleCenter(*observation);
    const double elapsed = (stamp - obstacle_track_.updated_at).seconds();
    if (elapsed <= 1e-3 || elapsed > 0.50) {
      beginObstacleTrack(*observation, path, stamp);
      return;
    }
    const Point2 measured_velocity{
      (center.x - obstacle_track_.center.x) / elapsed,
      (center.y - obstacle_track_.center.y) / elapsed};
    const double measured_speed = std::hypot(measured_velocity.x, measured_velocity.y);
    if (measured_speed <= 4.0) {
      obstacle_track_.velocity.x =
        (1.0 - obstacle_velocity_filter_gain_) * obstacle_track_.velocity.x +
        obstacle_velocity_filter_gain_ * measured_velocity.x;
      obstacle_track_.velocity.y =
        (1.0 - obstacle_velocity_filter_gain_) * obstacle_track_.velocity.y +
        obstacle_velocity_filter_gain_ * measured_velocity.y;
    }
    obstacle_track_.center = center;
    obstacle_track_.updated_at = stamp;
    obstacle_track_.missing_scans = 0;
    ++obstacle_track_.observations;

    const Projection projection = projectToPath(center, path);
    if (projection.valid) {
      obstacle_track_.longitudinal_speed =
        obstacle_track_.velocity.x * std::cos(projection.yaw) +
        obstacle_track_.velocity.y * std::sin(projection.yaw);
      if (obstacle_track_.longitudinal_speed >= moving_obstacle_min_speed_mps_) {
        obstacle_track_.moving_evidence = std::min(
          obstacle_track_.moving_evidence + 1, moving_obstacle_confirmation_scans_);
      } else {
        obstacle_track_.moving_evidence = std::max(
          0, obstacle_track_.moving_evidence - 1);
      }
      if (obstacle_track_.moving_evidence >= moving_obstacle_confirmation_scans_) {
        obstacle_track_.moving = true;
      }
    }
  }

  double trackedObstacleCurrentS(const PathModel & path) const
  {
    if (!obstacle_track_.valid) {
      return std::numeric_limits<double>::infinity();
    }
    const Projection projection = projectToPath(obstacle_track_.center, path);
    return projection.valid ? projection.s : std::numeric_limits<double>::infinity();
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

  Point2 pathNormal(const PathModel & path, const std::size_t index) const
  {
    const std::size_t first = index == 0 ? 0 : index - 1;
    const std::size_t second = index + 1 < path.points.size() ? index + 1 : index;
    const double dx = path.points[second].x - path.points[first].x;
    const double dy = path.points[second].y - path.points[first].y;
    const double norm = std::hypot(dx, dy);
    if (norm <= 1e-9) {
      return Point2{};
    }
    return Point2{-dy / norm, dx / norm};
  }

  std::vector<ScanHit> reprojectHits(
    const std::vector<ScanHit> & hits, const PathModel & reference) const
  {
    std::vector<ScanHit> projected;
    projected.reserve(hits.size());
    for (const auto & hit : hits) {
      ScanHit copy = hit;
      copy.projection = projectToPath(copy.point, reference);
      if (copy.projection.valid) {
        projected.push_back(std::move(copy));
      }
    }
    return projected;
  }

  ObstacleCluster reprojectObstacle(
    const ObstacleCluster & obstacle, const PathModel & reference) const
  {
    ObstacleCluster projected;
    projected.interfering_points = obstacle.interfering_points;
    for (const auto & hit : obstacle.hits) {
      ScanHit copy = hit;
      copy.projection = projectToPath(copy.point, reference);
      if (!copy.projection.valid) {
        continue;
      }
      projected.hits.push_back(copy);
      projected.s_min = std::min(projected.s_min, copy.projection.s);
      projected.s_max = std::max(projected.s_max, copy.projection.s);
      projected.d_min = std::min(projected.d_min, copy.projection.d);
      projected.d_max = std::max(projected.d_max, copy.projection.d);
      projected.minimum_range = std::min(projected.minimum_range, copy.range);
    }
    return projected;
  }

  bool racelineReferenceMatchesRaw(
    const PathModel & reference, const PathModel & raw) const
  {
    if (reference.frame != raw.frame || reference.points.empty() || raw.points.empty()) {
      return false;
    }
    constexpr std::size_t maximum_checks = 16;
    const std::size_t checks = std::min(maximum_checks, reference.points.size());
    for (std::size_t check = 0; check < checks; ++check) {
      const std::size_t index = checks == 1 ? 0 :
        check * (reference.points.size() - 1) / (checks - 1);
      const Projection projection = projectToPath(reference.points[index], raw);
      if (!projection.valid ||
        projection.distance > raceline_reference_match_tolerance_m_)
      {
        return false;
      }
    }
    return true;
  }

  RacelinePlanningContext prepareRacelinePlanningContext(
    const PathModel & raw, const std::vector<ScanHit> & raw_hits,
    const Point2 & robot_position, const double robot_yaw,
    const ObstacleCluster * raw_obstacle) const
  {
    RacelinePlanningContext context;
    if (!enable_frenet_lattice_planner_) {
      context.reason = "racing-line Frenet lattice is disabled";
      return context;
    }
    if (!raceline_reference_valid_) {
      context.reason = "global racing-line reference is unavailable";
      return context;
    }
    if (raw.frame != raceline_frame_) {
      context.reason = "raceline_frame does not match the raw path frame";
      return context;
    }

    Projection global_projection = projectToRacelineReference(robot_position);
    const Projection raw_projection = projectToPath(robot_position, raw);
    if (!global_projection.valid || !raw_projection.valid) {
      context.reason = "could not compare global and local racing-line directions";
      return context;
    }
    const double target_length = std::min(planning_distance_m_, raw.length);
    if (!buildLocalRacelineReference(
        robot_position, target_length, context.reference,
        context.reference_message, global_projection))
    {
      context.reason = "could not construct a forward global racing-line window";
      return context;
    }
    context.robot_projection = projectToPath(robot_position, context.reference);
    if (!context.robot_projection.valid) {
      context.reason = "robot pose cannot be projected onto the racing-line reference";
      return context;
    }
    if (!racelineReferenceMatchesRaw(context.reference, raw)) {
      context.reason =
        "global racing-line reference does not match the local raw racing line";
      return context;
    }
    context.robot_heading_error = angleDifference(
      robot_yaw, context.robot_projection.yaw);
    if (std::abs(angleDifference(raw_projection.yaw, context.robot_projection.yaw)) >
      M_PI / 2.0)
    {
      context.reason = "global racing-line direction is opposite to the local raw path";
      return context;
    }
    if (std::abs(context.robot_heading_error) > M_PI / 2.0) {
      context.reason = "vehicle heading is opposite to the forward planning direction";
      return context;
    }

    context.hits = reprojectHits(raw_hits, context.reference);
    const auto trusted_clusters = clusterScanHits(context.hits, context.reference.length);
    context.trusted_points.reserve(context.hits.size());
    for (const auto & cluster : trusted_clusters) {
      if (scanSupportCount(cluster.hits) <
        static_cast<std::size_t>(blocked_min_points_))
      {
        continue;
      }
      for (const auto & hit : cluster.hits) {
        context.trusted_points.push_back(hit.point);
      }
    }
    if (raw_obstacle) {
      context.obstacle = reprojectObstacle(*raw_obstacle, context.reference);
      context.obstacle_valid =
        scanSupportCount(context.obstacle.hits) >=
        static_cast<std::size_t>(blocked_min_points_) &&
        std::isfinite(context.obstacle.s_min) && std::isfinite(context.obstacle.d_min);
      if (!context.obstacle_valid) {
        context.reason = "raw obstacle could not be associated with the racing-line reference";
        return context;
      }
    }

    context.valid = true;
    context.reason = "racing-line Frenet planning context is ready";
    return context;
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
        candidate.reason = "planning reference contains a zero-length tangent";
        return false;
      }
      tx /= tangent_norm;
      ty /= tangent_norm;
      shifted.push_back(Point2{
        raw.points[i].x - ty * offsets[i],
        raw.points[i].y + tx * offsets[i]});
    }

    // Make the plan start at the actual localized car pose. The remainder is
    // generated in the selected Frenet frame, so it converges from the current
    // lateral position instead of assuming the car starts on the reference.
    shifted.front() = robot_position;

    if (require_map_clearance_ &&
      (!map_clearance_.valid || map_clearance_.frame != raw.frame))
    {
      candidate.reason = "occupancy-map corridor constraint is unavailable";
      return false;
    }
    double map_pass_clearance = std::numeric_limits<double>::infinity();
    const std::size_t map_end = std::min(
      candidate.rejoin_index, shifted.size() - static_cast<std::size_t>(1));
    for (std::size_t index = 0; index <= map_end; ++index) {
      const double clearance = mapClearanceAt(shifted[index], raw.frame);
      const bool passing_section = candidate.side != 0 &&
        index >= candidate.pass_start_index && index <= candidate.pass_end_index;
      const double required_clearance = passing_section ?
        planningClearance() : safety_half_width_m_;
      if (clearance <= required_clearance) {
        candidate.reason = "candidate leaves the occupancy-map free corridor";
        return false;
      }
      if (index >= candidate.pass_start_index && index <= candidate.pass_end_index) {
        map_pass_clearance = std::min(map_pass_clearance, clearance);
      }
    }
    candidate.min_clearance = std::min(candidate.min_clearance, map_pass_clearance);

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
    // final modified segment into the raceline rejoin point, but it does
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
    // Check curvature across the planner-to-raceline seam.
    const std::size_t splice_index = std::min(
      candidate.rejoin_index, shifted.size() - 1);
    if (splice_index > 0 && splice_index + 1 < shifted.size()) {
      const double a = distance(shifted[splice_index - 1], shifted[splice_index]);
      const double b = distance(shifted[splice_index], shifted[splice_index + 1]);
      const double c = distance(shifted[splice_index - 1], shifted[splice_index + 1]);
      if (a > 1e-6 && b > 1e-6 && c > 1e-6) {
        const double cross = std::abs(
          (shifted[splice_index].x - shifted[splice_index - 1].x) *
          (shifted[splice_index + 1].y - shifted[splice_index - 1].y) -
          (shifted[splice_index].y - shifted[splice_index - 1].y) *
          (shifted[splice_index + 1].x - shifted[splice_index - 1].x));
        candidate.max_curvature = std::max(
          candidate.max_curvature, 2.0 * cross / (a * b * c));
      }
    }
    if (candidate.max_curvature > curvature_limit) {
      candidate.reason = "candidate curvature exceeds steering limit";
      return false;
    }

    // Use connected LiDAR evidence as in the final guard, but add the planning
    // reserve so the selected trajectory does not merely scrape the physical
    // safety envelope. One isolated beam still cannot reject a whole path.
    int connected = 0;
    std::size_t previous_beam = 0;
    Point2 previous_point;
    bool previous_interfered = false;
    const std::size_t collision_end = std::min(
      shifted.size() - 1, candidate.rejoin_index + static_cast<std::size_t>(3));
    const auto shifted_segments = polyline_query::prepareSegments(shifted);
    for (const auto & hit : all_hits) {
      if (hit.projection.s < -0.20 || hit.projection.s > candidate.rejoin_s + 0.50) {
        continue;
      }
      const bool interfered = polyline_query::anyWithinDistance(
        hit.point.x, hit.point.y, planningClearance(),
        shifted_segments, 0, collision_end);

      // Score only the actual obstacle-passing section.  Departure and return
      // are shared smoothness requirements; including them here previously
      // biased the result toward the smaller offset rather than the wider gap.
      const std::size_t pass_begin = std::min(candidate.pass_start_index, collision_end);
      const std::size_t pass_end = std::min(candidate.pass_end_index + 1, collision_end);
      const auto pass_nearest = polyline_query::nearestPoint(
        hit.point.x, hit.point.y, shifted_segments, pass_begin, pass_end);
      candidate.min_clearance = std::min(
        candidate.min_clearance, pass_nearest.distance);

      if (interfered) {
        const bool connected_to_previous = previous_interfered &&
          hit.beam_index <= previous_beam + static_cast<std::size_t>(cluster_max_beam_gap_) &&
          distance(hit.point, previous_point) <= cluster_point_gap_m_;
        const int support = static_cast<int>(hit.support_count);
        connected = connected_to_previous ? connected + support : support;
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

  double activePlanOffsetAt(const Point2 & center, const Point2 & normal, const int side) const
  {
    if (!active_plan_.valid || (side != 0 && active_plan_.side != side) ||
      active_plan_.path.poses.empty())
    {
      return std::numeric_limits<double>::quiet_NaN();
    }

    const std::size_t begin = active_plan_.progress_index > 5 ?
      active_plan_.progress_index - 5 : 0;
    const std::size_t end = std::min(
      active_plan_.rejoin_index + static_cast<std::size_t>(1),
      active_plan_.path.poses.size());
    double best_distance = std::numeric_limits<double>::infinity();
    double best_offset = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t index = begin; index < end; ++index) {
      const auto & position = active_plan_.path.poses[index].pose.position;
      const Point2 point{position.x, position.y};
      const double separation = distance(center, point);
      if (separation < best_distance) {
        best_distance = separation;
        best_offset = (point.x - center.x) * normal.x +
          (point.y - center.y) * normal.y;
      }
    }
    return best_distance <= 0.75 ?
      best_offset : std::numeric_limits<double>::quiet_NaN();
  }

  path_following_v2::corridor::Result smoothLatticeOffsets(
    const path_following_v2::lattice::Problem & lattice_problem,
    const path_following_v2::lattice::Solver & lattice_solver,
    const std::vector<double> & initial_offsets) const
  {
    path_following_v2::corridor::Problem problem;
    problem.s.reserve(lattice_problem.stations.size());
    problem.reference_offsets.reserve(lattice_problem.stations.size());
    problem.lower_offsets.reserve(lattice_problem.stations.size());
    problem.upper_offsets.reserve(lattice_problem.stations.size());
    problem.continuity_offsets.reserve(lattice_problem.stations.size());
    problem.initial_offsets = initial_offsets;

    for (std::size_t station_index = 0;
      station_index < lattice_problem.stations.size(); ++station_index)
    {
      const auto & station = lattice_problem.stations[station_index];
      problem.s.push_back(station.s);
      problem.reference_offsets.push_back(station.reference_offset);
      problem.continuity_offsets.push_back(station.continuity_offset);

      double lower = station.lower_offset;
      double upper = station.upper_offset;
      if (station_index == 0 || station_index + 1 == lattice_problem.stations.size()) {
        lower = initial_offsets[station_index];
        upper = initial_offsets[station_index];
      } else if (!station.sample_clearances.empty()) {
        int selected_sample = 0;
        double selected_error = std::numeric_limits<double>::infinity();
        for (int sample = 0; sample < lattice_solver.sampleCount(); ++sample) {
          const double offset = lattice_solver.sampleOffset(station, sample);
          const double error = std::abs(offset - initial_offsets[station_index]);
          if (error < selected_error) {
            selected_error = error;
            selected_sample = sample;
          }
        }
        const auto sample_safe = [&station, &lattice_solver, this](const int sample) {
            const double offset = lattice_solver.sampleOffset(station, sample);
            if (offset < station.lower_offset - 1e-9 ||
              offset > station.upper_offset + 1e-9 ||
              std::abs(offset) > max_lateral_shift_m_ + 1e-9)
            {
              return false;
            }
            const double clearance =
              station.sample_clearances[static_cast<std::size_t>(sample)];
            return clearance == std::numeric_limits<double>::infinity() ||
                   (std::isfinite(clearance) && clearance > planningClearance());
          };
        int first_safe = selected_sample;
        int last_safe = selected_sample;
        if (sample_safe(selected_sample)) {
          while (first_safe > 0 && sample_safe(first_safe - 1)) {
            --first_safe;
          }
          while (last_safe + 1 < lattice_solver.sampleCount() &&
            sample_safe(last_safe + 1))
          {
            ++last_safe;
          }
          lower = std::max(lower, lattice_solver.sampleOffset(station, first_safe));
          upper = std::min(upper, lattice_solver.sampleOffset(station, last_safe));
        } else {
          lower = initial_offsets[station_index];
          upper = initial_offsets[station_index];
        }
      }
      problem.lower_offsets.push_back(lower);
      problem.upper_offsets.push_back(upper);
    }

    path_following_v2::corridor::Config config;
    config.max_iterations = corridor_smoothing_iterations_;
    config.relaxation = corridor_smoothing_relaxation_;
    config.reference_weight = corridor_reference_weight_;
    config.continuity_weight = corridor_continuity_weight_;
    config.slope_weight = corridor_slope_weight_;
    config.curvature_weight = corridor_curvature_weight_;
    config.curvature_rate_weight = corridor_curvature_rate_weight_;
    return path_following_v2::corridor::Solver(config).solve(problem);
  }

  std::vector<double> interpolateLatticeOffsets(
    const PathModel & reference,
    const path_following_v2::lattice::Problem & problem,
    const std::vector<double> & station_offsets) const
  {
    const std::size_t count = problem.stations.size();
    std::vector<double> slopes(count, 0.0);
    std::vector<double> secants(count - 1, 0.0);
    for (std::size_t index = 0; index + 1 < count; ++index) {
      const double ds = problem.stations[index + 1].s - problem.stations[index].s;
      secants[index] = (station_offsets[index + 1] - station_offsets[index]) /
        std::max(ds, 1e-6);
    }

    // Shape-preserving cubic interpolation smooths the discrete lattice
    // without overshooting its sampled lateral corridor.
    slopes.front() = problem.start_slope;
    if (std::abs(secants.front()) <= 1e-9 ||
      slopes.front() * secants.front() <= 0.0)
    {
      slopes.front() = 0.0;
    } else {
      slopes.front() = std::copysign(
        std::min(std::abs(slopes.front()), 3.0 * std::abs(secants.front())),
        slopes.front());
    }
    for (std::size_t index = 1; index + 1 < count; ++index) {
      if (secants[index - 1] * secants[index] <= 0.0) {
        slopes[index] = 0.0;
        continue;
      }
      const double previous_ds =
        problem.stations[index].s - problem.stations[index - 1].s;
      const double next_ds =
        problem.stations[index + 1].s - problem.stations[index].s;
      const double first_weight = 2.0 * next_ds + previous_ds;
      const double second_weight = next_ds + 2.0 * previous_ds;
      slopes[index] = (first_weight + second_weight) /
        (first_weight / secants[index - 1] + second_weight / secants[index]);
    }
    slopes.back() = secants.back();

    std::vector<double> dense_offsets(reference.points.size(), 0.0);
    std::size_t interval = 0;
    for (std::size_t index = 0; index < reference.points.size(); ++index) {
      const double s = reference.s[index];
      if (s > problem.stations.back().s + 1e-9) {
        // The global racing line is the Frenet reference, so its desired
        // lateral offset is exactly zero outside the locally modified section.
        dense_offsets[index] = 0.0;
        continue;
      }
      while (interval + 2 < count && s > problem.stations[interval + 1].s) {
        ++interval;
      }
      const double s0 = problem.stations[interval].s;
      const double s1 = problem.stations[interval + 1].s;
      const double length = std::max(s1 - s0, 1e-6);
      const double u = std::clamp((s - s0) / length, 0.0, 1.0);
      const double u2 = u * u;
      const double u3 = u2 * u;
      dense_offsets[index] =
        (2.0 * u3 - 3.0 * u2 + 1.0) * station_offsets[interval] +
        (u3 - 2.0 * u2 + u) * length * slopes[interval] +
        (-2.0 * u3 + 3.0 * u2) * station_offsets[interval + 1] +
        (u3 - u2) * length * slopes[interval + 1];
      dense_offsets[index] = std::clamp(
        dense_offsets[index], -max_lateral_shift_m_, max_lateral_shift_m_);
    }
    return dense_offsets;
  }

  path_following_v2::lattice::Config latticeConfig() const
  {
    path_following_v2::lattice::Config config;
    config.lateral_step_m = lattice_lateral_step_m_;
    config.max_lateral_shift_m = max_lateral_shift_m_;
    config.max_abs_slope = lattice_max_abs_slope_;
    config.curvature_limit_inv_m = curvature_safety_factor_ *
      std::tan(steering_max_deg_ * M_PI / 180.0) / wheelbase_m_;
    config.collision_clearance_m = planningClearance();
    config.preferred_clearance_m = lattice_preferred_clearance_m_;
    config.reference_weight = lattice_reference_weight_;
    config.slope_weight = lattice_slope_weight_;
    config.curvature_weight = lattice_curvature_weight_;
    config.curvature_rate_weight = lattice_curvature_rate_weight_;
    config.continuity_weight = lattice_continuity_weight_;
    config.clearance_weight = lattice_clearance_weight_;
    config.terminal_weight_multiplier = lattice_terminal_weight_multiplier_;
    config.terminal_distance_m = lattice_terminal_distance_m_;
    config.beam_width = lattice_beam_width_;
    config.max_solutions = lattice_max_final_candidates_;
    config.max_compute_time_ms = lattice_max_compute_time_ms_;
    return config;
  }

  Candidate bestLatticeCandidate(
    const RacelinePlanningContext & context,
    const Point2 & robot_position, const double robot_yaw, const int side,
    const bool avoidance, const bool force_open_end = false)
  {
    Candidate best;
    best.side = side;
    best.trajectory_mode = avoidance ? "AVOIDING" : "RECOVERING_TO_RACELINE";
    best.planning_reference = "racingline_frenet_corridor";
    best.start_lateral_offset = context.robot_projection.d;
    if (!context.valid || (avoidance && !context.obstacle_valid)) {
      best.reason = context.reason;
      return best;
    }
    if (require_map_clearance_ &&
      (!map_clearance_.valid || map_clearance_.frame != context.reference.frame))
    {
      best.reason = "occupancy-map corridor constraint is unavailable";
      return best;
    }

    const PathModel & reference = context.reference;
    double pass_start_s = 0.0;
    double pass_end_s = 0.0;
    double return_end_s = 0.0;
    double plan_end_s = reference.length;
    double required_offset = 0.0;
    if (avoidance) {
      best.rolling_pass = !static_obstacle_fast_mode_ &&
        (force_open_end ||
        (trackedObstacleMatches(context.obstacle) && obstacle_track_.moving));
      pass_start_s = std::max(
        0.0, context.obstacle.s_min - detour_longitudinal_buffer_m_);
      if (best.rolling_pass) {
        // Keep an open-ended pass until the moving target is behind.
        plan_end_s = reference.length;
        pass_end_s = plan_end_s;
        return_end_s = plan_end_s;
      } else {
        const double conservative_obstacle_end = std::max(
          context.obstacle.s_max,
          context.obstacle.s_min + max_obstacle_size_m_);
        pass_end_s = conservative_obstacle_end + detour_longitudinal_buffer_m_ +
          detour_post_obstacle_hold_m_;
        const double available_after_pass = reference.length - 0.20 - pass_end_s;
        const double required_after_pass =
          detour_return_min_length_m_ + lattice_rejoin_alignment_length_m_;
        if (available_after_pass < required_after_pass) {
          best.reason =
            "racing-line window is too short for the smooth return and aligned handoff";
          return best;
        }
        const double return_length = std::min(
          detour_return_max_length_m_,
          available_after_pass - lattice_rejoin_alignment_length_m_);
        return_end_s = pass_end_s + return_length;
        plan_end_s = return_end_s + lattice_rejoin_alignment_length_m_;
      }
      const auto lateral_extent = conservativeObstacleLateralExtent(context.obstacle);
      required_offset = side > 0 ?
        lateral_extent.second + planningClearance() + detour_extra_clearance_m_ :
        lateral_extent.first - planningClearance() - detour_extra_clearance_m_;
      if (std::abs(required_offset) > max_lateral_shift_m_ + 1e-9) {
        best.reason = "required racing-line offset exceeds max_lateral_shift_m";
        return best;
      }
    } else {
      plan_end_s = std::clamp(
        recovery_min_rejoin_length_m_ +
        recovery_length_gain_ * std::abs(context.robot_projection.d),
        recovery_min_rejoin_length_m_, recovery_max_rejoin_length_m_);
      if (plan_end_s > reference.length - 0.20) {
        best.reason = "racing-line window is too short for lateral recovery";
        return best;
      }
      pass_end_s = 0.5 * plan_end_s;
    }

    const std::size_t plan_end_index = indexAtOrAfter(reference, plan_end_s);
    std::vector<std::size_t> station_indices{0};
    double next_station_s = lattice_station_step_m_;
    for (std::size_t index = 1; index < plan_end_index; ++index) {
      if (reference.s[index] + 1e-9 >= next_station_s) {
        station_indices.push_back(index);
        next_station_s = reference.s[index] + lattice_station_step_m_;
      }
    }
    if (station_indices.back() != plan_end_index) {
      station_indices.push_back(plan_end_index);
    }
    if (station_indices.size() < 3) {
      best.reason = "racing-line window produced fewer than three lattice stations";
      return best;
    }

    path_following_v2::lattice::Problem problem;
    problem.start_offset = context.robot_projection.d;
    problem.start_slope = std::clamp(
      std::tan(std::clamp(context.robot_heading_error, -M_PI / 4.0, M_PI / 4.0)),
      -lattice_max_abs_slope_, lattice_max_abs_slope_);

    const auto config = latticeConfig();
    const path_following_v2::lattice::Solver solver(config);
    const auto clearance_grid_started_at = std::chrono::steady_clock::now();
    problem.stations.reserve(station_indices.size());
    for (const std::size_t index : station_indices) {
      const Point2 normal = pathNormal(reference, index);
      path_following_v2::lattice::Station station;
      station.s = reference.s[index];
      station.x = reference.points[index].x;
      station.y = reference.points[index].y;
      station.normal_x = normal.x;
      station.normal_y = normal.y;
      // The static global racing line is the Frenet reference and desired
      // nominal trajectory, so every unmodified station targets d=0.
      constexpr double raceline_offset = 0.0;
      station.reference_offset = raceline_offset;
      station.continuity_offset = activePlanOffsetAt(
        reference.points[index], normal, side);
      if (avoidance) {
        // Give the forward beam search a smooth, side-specific guide so it
        // begins moving before the hard passing corridor.  The search still
        // chooses nearby samples using clearance and curvature costs.
        if (station.s < pass_start_s) {
          const double blend = quinticSmoothstep(
            station.s / std::max(pass_start_s, 1e-6));
          station.reference_offset =
            raceline_offset + blend * (required_offset - raceline_offset);
        } else if (station.s <= pass_end_s) {
          station.reference_offset = required_offset;
        } else if (station.s < return_end_s) {
          const double blend = quinticSmoothstep(
            (station.s - pass_end_s) /
            std::max(return_end_s - pass_end_s, 1e-6));
          station.reference_offset =
            required_offset + blend * (raceline_offset - required_offset);
        } else {
          station.reference_offset = raceline_offset;
        }
      }
      station.lower_offset = -max_lateral_shift_m_;
      station.upper_offset = max_lateral_shift_m_;
      const bool passing_section = avoidance &&
        station.s + 1e-9 >= pass_start_s && station.s <= pass_end_s + 1e-9;
      if (passing_section)
      {
        if (side > 0) {
          station.lower_offset = std::max(station.lower_offset, required_offset);
        } else {
          station.upper_offset = std::min(station.upper_offset, required_offset);
        }
      }
      const bool avoidance_alignment =
        avoidance && !best.rolling_pass && station.s + 1e-9 >= return_end_s;
      const bool recovery_alignment = !avoidance &&
        station.s + 1e-9 >= plan_end_s - lattice_rejoin_alignment_length_m_;
      if (avoidance_alignment || recovery_alignment) {
        // Pin the terminal interval to align position and tangent.
        station.lower_offset = raceline_offset;
        station.upper_offset = raceline_offset;
      }
      if (index == plan_end_index) {
        station.lower_offset = station.reference_offset;
        station.upper_offset = station.reference_offset;
      }
      station.sample_clearances.resize(
        static_cast<std::size_t>(solver.sampleCount()),
        std::numeric_limits<double>::infinity());
      for (int sample = 0; sample < solver.sampleCount(); ++sample) {
        const double offset = solver.sampleOffset(station, sample);
        if (!solver.sampleGeometryAllowed(station, offset)) {
          continue;
        }
        const Point2 shifted{
          station.x + station.normal_x * offset,
          station.y + station.normal_y * offset};
        auto & sample_clearance =
          station.sample_clearances[static_cast<std::size_t>(sample)];
        if (require_map_clearance_) {
          const double map_clearance = mapClearanceAtUnchecked(shifted);
          if (!std::isfinite(map_clearance)) {
            // Unknown and out-of-map samples are forbidden. The map frame was
            // validated once before entering this hot loop.
            sample_clearance = 0.0;
            continue;
          } else {
            const double required_map_clearance = passing_section ?
              planningClearance() : safety_half_width_m_;
            // Normalize the physical-only departure/return allowance into the
            // lattice's planning-clearance comparison domain.
            const double normalized_map_clearance = map_clearance +
              (planningClearance() - required_map_clearance);
            sample_clearance = std::min(sample_clearance, normalized_map_clearance);
          }
        }
        if (sample_clearance <= planningClearance()) {
          continue;
        }
        sample_clearance =
          path_following_v2::lattice_clearance::refineWithTrustedPoints(
          shifted.x, shifted.y, sample_clearance,
          lattice_preferred_clearance_m_, planningClearance(),
          context.trusted_points);
      }
      problem.stations.push_back(std::move(station));
    }
    last_lattice_clearance_grid_time_ms_ +=
      std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - clearance_grid_started_at).count();

    const auto result = solver.solve(problem);
    last_lattice_evaluated_transitions_ += result.evaluated_transitions;
    last_lattice_compute_time_ms_ += result.compute_time_ms;
    const std::string result_summary =
      std::string(side > 0 ? "left: " : (side < 0 ? "right: " : "recovery: ")) +
      result.reason;
    last_lattice_reason_ = last_lattice_reason_ == "not run" ?
      result_summary : last_lattice_reason_ + "; " + result_summary;
    if (!result.valid) {
      best.reason = result.reason;
      return best;
    }

    std::string last_validation_reason = "no lattice solution passed final validation";
    for (const auto & solution : result.solutions) {
      const auto smoothing = smoothLatticeOffsets(problem, solver, solution.offsets);
      const auto evaluate_station_offsets = [&](
          const std::vector<double> & station_offsets,
          const std::string & planning_reference) {
          Candidate candidate;
          candidate.side = side;
          candidate.trajectory_mode = best.trajectory_mode;
          candidate.planning_reference = planning_reference;
          candidate.start_lateral_offset = context.robot_projection.d;
          candidate.objective_cost = solution.cost;
          candidate.objective_domain = selection::ObjectiveDomain::LATTICE;
          candidate.rolling_pass = best.rolling_pass;
          candidate.pass_start_index = indexAtOrAfter(reference, pass_start_s);
          candidate.pass_end_index = indexAtOrAfter(reference, pass_end_s);
          candidate.rejoin_index = plan_end_index;
          candidate.rejoin_s = reference.s[plan_end_index];
          auto dense_offsets = interpolateLatticeOffsets(
            reference, problem, station_offsets);
          if (avoidance) {
            for (std::size_t index = candidate.pass_start_index;
              index <= candidate.pass_end_index && index < dense_offsets.size(); ++index)
            {
              dense_offsets[index] = side > 0 ?
                std::max(dense_offsets[index], required_offset) :
                std::min(dense_offsets[index], required_offset);
            }
          }

          double peak_displacement = 0.0;
          for (std::size_t index = 0; index <= candidate.rejoin_index; ++index) {
            const double displacement = dense_offsets[index];
            if (std::abs(displacement) > std::abs(peak_displacement)) {
              peak_displacement = displacement;
            }
          }
          candidate.peak_offset = peak_displacement;
          candidate.valid = fillCandidatePath(
            candidate, reference, context.reference_message,
            robot_position, robot_yaw, dense_offsets, context.hits);
          return candidate;
        };

      Candidate candidate = smoothing.valid ?
        evaluate_station_offsets(smoothing.offsets, "racingline_frenet_corridor") :
        evaluate_station_offsets(solution.offsets, "racingline_frenet_lattice");
      if (!candidate.valid && smoothing.valid) {
        last_validation_reason = "corridor: " + candidate.reason;
        candidate = evaluate_station_offsets(solution.offsets, "racingline_frenet_lattice");
      }
      if (!candidate.valid) {
        last_validation_reason += "; lattice: " + candidate.reason;
        continue;
      }
      candidate.valid = true;
      candidate.reason = avoidance ?
        "bounded racing-line corridor detour passed final validation" :
        "bounded racing-line corridor recovery passed final validation";
      // Solver results are cost-ordered, so later valid solutions cannot win.
      best = std::move(candidate);
      break;
    }
    if (!best.valid) {
      best.reason = "lattice solutions failed final validation; last: " +
        last_validation_reason;
    }
    return best;
  }

  Candidate buildDetourCandidateAtOffset(
    const PathModel & raw, const nav_msgs::msg::Path & raw_message,
    const ObstacleCluster & obstacle, const std::vector<ScanHit> & all_hits,
    const Point2 & robot_position, const double robot_yaw,
    const double car_lateral_offset, const double car_heading_error,
    const int side, const double peak_offset,
    const bool force_open_end = false) const
  {
    Candidate candidate;
    candidate.side = side;
    candidate.trajectory_mode = "AVOIDING";
    candidate.peak_offset = peak_offset;
    candidate.start_lateral_offset = car_lateral_offset;
    candidate.rolling_pass = !static_obstacle_fast_mode_ &&
      (force_open_end ||
      (trackedObstacleMatches(obstacle) && obstacle_track_.moving));

    const double departure_end = std::max(
      0.25, obstacle.s_min - detour_longitudinal_buffer_m_);
    double plateau_end = raw.length;
    double return_end = plateau_end;
    if (!candidate.rolling_pass) {
      const double conservative_obstacle_end = std::max(
        obstacle.s_max, obstacle.s_min + max_obstacle_size_m_);
      plateau_end = conservative_obstacle_end + detour_longitudinal_buffer_m_ +
        detour_post_obstacle_hold_m_;
      const double available_return_length = raw.length - 0.20 - plateau_end;
      if (available_return_length < detour_return_min_length_m_) {
        candidate.reason = "raw path is too short for minimum smooth return";
        return candidate;
      }
      // Use the longest return that fits the known raceline.
      const double return_length = std::min(
        detour_return_max_length_m_, available_return_length);
      return_end = plateau_end + return_length;
    }
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
    const int side, const bool force_open_end = false) const
  {
    Candidate best;
    best.side = side;
    best.trajectory_mode = "AVOIDING";
    const auto lateral_extent = conservativeObstacleLateralExtent(obstacle);
    const double required_offset = side > 0 ?
      lateral_extent.second + planningClearance() + detour_extra_clearance_m_ :
      lateral_extent.first - planningClearance() - detour_extra_clearance_m_;
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
        static_cast<double>(side) * magnitude, force_open_end);
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

  bool staticAnalyticRequiredOffset(
    const ObstacleCluster & obstacle, const int side,
    double & required_offset) const
  {
    const auto lateral_extent = conservativeObstacleLateralExtent(obstacle);
    const double extra_clearance =
      detour_extra_clearance_m_ + static_analytic_extra_clearance_m_;
    required_offset = side > 0 ?
      lateral_extent.second + planningClearance() + extra_clearance :
      lateral_extent.first - planningClearance() - extra_clearance;
    return side * required_offset > 0.0 &&
           std::abs(required_offset) <= max_lateral_shift_m_ + 1e-9;
  }

  Candidate firstValidStaticAnalyticCandidateForSide(
    const PathModel & raw, const nav_msgs::msg::Path & raw_message,
    const ObstacleCluster & obstacle, const std::vector<ScanHit> & all_hits,
    const Point2 & robot_position, const double robot_yaw,
    const double car_lateral_offset, const double car_heading_error,
    const int side, int & evaluated) const
  {
    Candidate result;
    result.side = side;
    result.trajectory_mode = "AVOIDING";
    result.planning_reference = "racingline_static_analytic";
    evaluated = 0;

    double required_offset = 0.0;
    if (!staticAnalyticRequiredOffset(obstacle, side, required_offset)) {
      result.reason = "static analytic required offset is outside the requested side/bounds";
      return result;
    }

    const double required_magnitude = std::abs(required_offset);
    std::string last_reason = "no static analytic offset evaluated";
    for (int sample = 0; sample < static_analytic_candidates_per_side_; ++sample) {
      const double magnitude = required_magnitude +
        static_cast<double>(sample) * candidate_lateral_step_m_;
      if (magnitude > max_lateral_shift_m_ + 1e-9) {
        break;
      }
      ++evaluated;
      Candidate candidate = buildDetourCandidateAtOffset(
        raw, raw_message, obstacle, all_hits, robot_position, robot_yaw,
        car_lateral_offset, car_heading_error, side,
        static_cast<double>(side) * magnitude, false);
      candidate.planning_reference = "racingline_static_analytic";
      last_reason = candidate.reason;
      if (candidate.valid) {
        candidate.reason =
          "first-valid static analytic detour passed full planning validation";
        return candidate;
      }
    }

    result.reason = "no valid static analytic path among " +
      std::to_string(evaluated) + " bounded offsets; last: " + last_reason;
    return result;
  }

  double staticSideMapScore(
    const PathModel & raw, const ObstacleCluster & obstacle,
    const int side) const
  {
    double required_offset = 0.0;
    if (!staticAnalyticRequiredOffset(obstacle, side, required_offset)) {
      return -std::numeric_limits<double>::infinity();
    }

    // This score only orders the two sides. Candidate construction below
    // still checks every dense path point against the map and LiDAR envelope.
    if (!map_clearance_.valid || map_clearance_.frame != raw.frame) {
      return -std::abs(required_offset);
    }
    const double conservative_obstacle_end = std::max(
      obstacle.s_max, obstacle.s_min + max_obstacle_size_m_);
    const double score_start_s = std::clamp(obstacle.s_min, 0.0, raw.length);
    const double score_end_s = std::clamp(
      conservative_obstacle_end + detour_longitudinal_buffer_m_ +
      detour_post_obstacle_hold_m_, score_start_s, raw.length);
    constexpr int score_samples = 9;
    double minimum_clearance = std::numeric_limits<double>::infinity();
    for (int sample = 0; sample < score_samples; ++sample) {
      const double ratio = static_cast<double>(sample) /
        static_cast<double>(score_samples - 1);
      const double target_s = score_start_s + ratio * (score_end_s - score_start_s);
      const std::size_t index = indexAtOrAfter(raw, target_s);
      const Point2 normal = pathNormal(raw, index);
      const Point2 shifted{
        raw.points[index].x + normal.x * required_offset,
        raw.points[index].y + normal.y * required_offset};
      minimum_clearance = std::min(
        minimum_clearance, mapClearanceAtUnchecked(shifted));
    }
    // Break sub-millimetre map-score ties in favour of the smaller shift.
    return minimum_clearance - 1e-4 * std::abs(required_offset);
  }

  void resetStaticFastSearchDiagnostics()
  {
    last_lattice_evaluated_transitions_ = 0;
    last_lattice_compute_time_ms_ = 0.0;
    last_lattice_clearance_grid_time_ms_ = 0.0;
    last_lattice_reason_ = "not run";
    last_static_analytic_candidates_ = 0;
    last_static_analytic_time_ms_ = 0.0;
    last_static_side_score_time_ms_ = 0.0;
    last_static_total_time_ms_ = 0.0;
    last_static_left_map_score_m_ = -std::numeric_limits<double>::infinity();
    last_static_right_map_score_m_ = -std::numeric_limits<double>::infinity();
    last_static_backend_ = "not run";
  }

  template<typename ContextProvider>
  Candidate bestStaticObstacleCandidateForSide(
    ContextProvider && get_context,
    const PathModel & raw, const nav_msgs::msg::Path & raw_message,
    const ObstacleCluster & obstacle, const std::vector<ScanHit> & all_hits,
    const Point2 & robot_position, const double robot_yaw,
    const double car_lateral_offset, const double car_heading_error,
    const int side)
  {
    const auto total_started_at = std::chrono::steady_clock::now();
    const auto analytic_started_at = std::chrono::steady_clock::now();
    int evaluated = 0;
    Candidate analytic = firstValidStaticAnalyticCandidateForSide(
      raw, raw_message, obstacle, all_hits, robot_position, robot_yaw,
      car_lateral_offset, car_heading_error, side, evaluated);
    last_static_analytic_candidates_ += evaluated;
    last_static_analytic_time_ms_ += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - analytic_started_at).count();
    if (analytic.valid) {
      last_reference_source_ = "racingline_static_analytic";
      last_static_backend_ = "analytic";
      last_static_total_time_ms_ += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - total_started_at).count();
      return analytic;
    }

    Candidate lattice;
    lattice.side = side;
    lattice.trajectory_mode = "AVOIDING";
    lattice.planning_reference = "racingline_frenet_corridor";
    if (enable_frenet_lattice_planner_) {
      const RacelinePlanningContext & context = get_context();
      if (context.valid && context.obstacle_valid) {
        last_reference_source_ = "racingline_static_bounded_lattice";
        // Exactly one bounded, full-return lattice attempt. Static fast mode
        // never asks for the open-ended or moving-target variants.
        lattice = bestLatticeCandidate(
          context, robot_position, robot_yaw, side, true, false);
      } else {
        lattice.reason = context.reason;
      }
    } else {
      lattice.reason = "bounded Frenet lattice is disabled";
    }

    if (lattice.valid) {
      lattice.reason = "static analytic path unavailable; " + lattice.reason;
      last_static_backend_ = "bounded_lattice";
    } else {
      lattice.reason = "analytic: " + analytic.reason +
        "; bounded full-return lattice: " + lattice.reason;
      last_reference_source_ = "racingline_static_fast_unavailable";
      last_static_backend_ = "none";
    }
    last_static_total_time_ms_ += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - total_started_at).count();
    return lattice;
  }

  template<typename ContextProvider>
  const Candidate * selectStaticObstacleCandidate(
    ContextProvider && get_context,
    const PathModel & raw, const nav_msgs::msg::Path & raw_message,
    const ObstacleCluster & obstacle, const std::vector<ScanHit> & all_hits,
    const Point2 & robot_position, const double robot_yaw,
    const double car_lateral_offset, const double car_heading_error,
    Candidate & left, Candidate & right)
  {
    const auto score_started_at = std::chrono::steady_clock::now();
    last_static_left_map_score_m_ = staticSideMapScore(raw, obstacle, +1);
    last_static_right_map_score_m_ = staticSideMapScore(raw, obstacle, -1);
    const double score_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - score_started_at).count();
    last_static_side_score_time_ms_ += score_time_ms;
    last_static_total_time_ms_ += score_time_ms;
    const int first_side = last_static_left_map_score_m_ >=
      last_static_right_map_score_m_ ? +1 : -1;
    const int second_side = -first_side;

    const auto evaluate = [&](const int side) -> Candidate & {
        Candidate & candidate = side > 0 ? left : right;
        candidate = bestStaticObstacleCandidateForSide(
          get_context, raw, raw_message, obstacle, all_hits,
          robot_position, robot_yaw, car_lateral_offset,
          car_heading_error, side);
        return candidate;
      };

    Candidate & first = evaluate(first_side);
    if (static_first_valid_side_ && first.valid) {
      return &first;
    }
    Candidate & second = evaluate(second_side);
    if (static_first_valid_side_) {
      return second.valid ? &second : nullptr;
    }
    return chooseCandidate(left, right);
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

  Candidate buildRollingContinuationCandidate(
    const PathModel & raw, const nav_msgs::msg::Path & raw_message,
    const std::vector<ScanHit> & all_hits, const Point2 & robot_position,
    const double robot_yaw, const double car_lateral_offset, const int side) const
  {
    Candidate candidate;
    candidate.side = side;
    candidate.trajectory_mode = "AVOIDING";
    candidate.planning_reference = "raceline_rolling_continuation";
    candidate.start_lateral_offset = car_lateral_offset;
    candidate.peak_offset = car_lateral_offset;
    candidate.rolling_pass = true;
    candidate.pass_start_index = 0;
    candidate.pass_end_index = raw.points.size() - 1;
    candidate.rejoin_index = raw.points.size() - 1;
    candidate.rejoin_s = raw.length;
    if (side * car_lateral_offset <= safety_half_width_m_) {
      candidate.reason = "vehicle has not reached the committed passing corridor";
      return candidate;
    }
    std::vector<double> offsets(raw.points.size(), car_lateral_offset);
    if (!fillCandidatePath(
        candidate, raw, raw_message, robot_position, robot_yaw, offsets, all_hits))
    {
      return candidate;
    }
    candidate.valid = true;
    candidate.reason = "map-bounded rolling pass continuation is valid";
    return candidate;
  }

  Candidate bestAvailableDetourCandidateForSide(
    const RacelinePlanningContext & context,
    const PathModel & raw, const nav_msgs::msg::Path & raw_message,
    const ObstacleCluster & obstacle, const std::vector<ScanHit> & all_hits,
    const Point2 & robot_position, const double robot_yaw,
    const double car_lateral_offset, const double car_heading_error,
    const int side, const bool allow_legacy_fallback = true)
  {
    if (enable_frenet_lattice_planner_ && context.valid && context.obstacle_valid) {
      last_reference_source_ = "racingline_frenet_corridor";
      Candidate lattice = bestLatticeCandidate(
        context, robot_position, robot_yaw, side, true);
      if (lattice.valid) {
        return lattice;
      }
      Candidate extended_lattice = bestLatticeCandidate(
        context, robot_position, robot_yaw, side, true, true);
      if (extended_lattice.valid) {
        extended_lattice.reason =
          "full-return candidate unavailable; open-ended racing-line pass is valid";
        return extended_lattice;
      }
      if (!lattice_fallback_to_legacy_planner_ || !allow_legacy_fallback) {
        extended_lattice.reason = "full-return: " + lattice.reason +
          "; open-ended: " + extended_lattice.reason;
        return extended_lattice;
      }
      Candidate legacy = bestDetourCandidateForSide(
        raw, raw_message, obstacle, all_hits, robot_position, robot_yaw,
        car_lateral_offset, car_heading_error, side);
      last_reference_source_ = "racingline_legacy_fallback";
      if (legacy.valid) {
        return legacy;
      }
      Candidate extended_legacy = bestDetourCandidateForSide(
        raw, raw_message, obstacle, all_hits, robot_position, robot_yaw,
        car_lateral_offset, car_heading_error, side, true);
      if (extended_legacy.valid) {
        extended_legacy.reason =
          "full-return candidates unavailable; open-ended raceline pass is valid";
        return extended_legacy;
      }
      extended_legacy.reason = "lattice: " + lattice.reason +
        "; extended lattice: " + extended_lattice.reason +
        "; legacy: " + legacy.reason +
        "; extended legacy: " + extended_legacy.reason;
      return extended_legacy;
    }
    if (enable_frenet_lattice_planner_ && require_raceline_reference_) {
      Candidate unavailable;
      unavailable.side = side;
      unavailable.trajectory_mode = "AVOIDING";
      unavailable.reason = context.reason;
      last_reference_source_ = "racingline_reference_unavailable";
      return unavailable;
    }
    last_reference_source_ = "racingline_legacy";
    Candidate legacy = bestDetourCandidateForSide(
      raw, raw_message, obstacle, all_hits, robot_position, robot_yaw,
      car_lateral_offset, car_heading_error, side);
    if (legacy.valid) {
      return legacy;
    }
    return bestDetourCandidateForSide(
      raw, raw_message, obstacle, all_hits, robot_position, robot_yaw,
      car_lateral_offset, car_heading_error, side, true);
  }

  Candidate bestAvailableRecoveryCandidate(
    const RacelinePlanningContext & context,
    const PathModel & raw, const nav_msgs::msg::Path & raw_message,
    const std::vector<ScanHit> & all_hits, const Point2 & robot_position,
    const double robot_yaw, const double car_lateral_offset,
    const double car_heading_error, const bool allow_legacy_fallback = true)
  {
    if (enable_frenet_lattice_planner_ && context.valid) {
      last_reference_source_ = "racingline_frenet_corridor";
      Candidate lattice = bestLatticeCandidate(
        context, robot_position, robot_yaw, 0, false);
      if (lattice.valid || static_obstacle_fast_mode_ ||
        !lattice_fallback_to_legacy_planner_ ||
        !allow_legacy_fallback)
      {
        return lattice;
      }
      Candidate legacy = buildRecoveryCandidate(
        raw, raw_message, all_hits, robot_position, robot_yaw,
        car_lateral_offset, car_heading_error);
      last_reference_source_ = "racingline_legacy_fallback";
      if (!legacy.valid) {
        legacy.reason = "lattice: " + lattice.reason + "; legacy: " + legacy.reason;
      }
      return legacy;
    }
    if (enable_frenet_lattice_planner_ && require_raceline_reference_) {
      Candidate unavailable;
      unavailable.side = 0;
      unavailable.trajectory_mode = "RECOVERING_TO_RACELINE";
      unavailable.reason = context.reason;
      last_reference_source_ = "racingline_reference_unavailable";
      return unavailable;
    }
    if (static_obstacle_fast_mode_) {
      Candidate unavailable;
      unavailable.side = 0;
      unavailable.trajectory_mode = "RECOVERING_TO_RACELINE";
      unavailable.reason = enable_frenet_lattice_planner_ ? context.reason :
        "static-obstacle fast mode requires the bounded Frenet recovery lattice";
      last_reference_source_ = "racingline_static_recovery_unavailable";
      return unavailable;
    }
    last_reference_source_ = "racingline_legacy";
    return buildRecoveryCandidate(
      raw, raw_message, all_hits, robot_position, robot_yaw,
      car_lateral_offset, car_heading_error);
  }

  const Candidate * chooseCandidate(const Candidate & left, const Candidate & right) const
  {
    selection::Metrics left_metrics;
    left_metrics.valid = left.valid;
    left_metrics.minimum_clearance = left.min_clearance;
    left_metrics.objective_cost = left.objective_cost;
    left_metrics.maximum_curvature = left.max_curvature;
    left_metrics.peak_offset = left.peak_offset;
    left_metrics.objective_domain = left.objective_domain;

    selection::Metrics right_metrics;
    right_metrics.valid = right.valid;
    right_metrics.minimum_clearance = right.min_clearance;
    right_metrics.objective_cost = right.objective_cost;
    right_metrics.maximum_curvature = right.max_curvature;
    right_metrics.peak_offset = right.peak_offset;
    right_metrics.objective_domain = right.objective_domain;

    switch (selection::chooseSaferCandidate(
        left_metrics, right_metrics, side_clearance_tie_m_))
    {
      case selection::Choice::LEFT:
        return &left;
      case selection::Choice::RIGHT:
        return &right;
      case selection::Choice::NONE:
      default:
        return nullptr;
    }
  }

  CandidateSummary summarizeCandidate(const Candidate & candidate) const
  {
    CandidateSummary summary;
    summary.evaluated = candidate.side != 0;
    summary.valid = candidate.valid;
    summary.reason = candidate.reason;
    summary.minimum_clearance = candidate.min_clearance;
    summary.objective_cost = candidate.objective_cost;
    summary.objective_domain = candidate.objective_domain;
    summary.maximum_curvature = candidate.max_curvature;
    summary.peak_offset = candidate.peak_offset;
    summary.maximum_connected_interference =
      candidate.maximum_connected_interference;
    return summary;
  }

  void recordCandidateDecision(
    const Candidate & left, const Candidate & right,
    const Candidate * selected, const std::string & context)
  {
    ++candidate_decision_id_;
    candidate_decision_context_ = context;
    candidate_decision_selected_side_ = selected ? selected->side : 0;
    last_left_candidate_ = summarizeCandidate(left);
    last_right_candidate_ = summarizeCandidate(right);
  }

  void activatePlan(const Candidate & candidate, const std::string & reason)
  {
    active_plan_ = ActivePlan();
    active_plan_.valid = true;
    active_plan_.id = next_plan_id_++;
    active_plan_.side = candidate.side;
    active_plan_.source_mode = candidate.trajectory_mode;
    active_plan_.planning_reference = candidate.planning_reference;
    active_plan_.objective_domain = candidate.objective_domain;
    active_plan_.reason = reason;
    active_plan_.path = candidate.path;
    active_plan_.pass_start_index = candidate.pass_start_index;
    active_plan_.pass_end_index = candidate.pass_end_index;
    active_plan_.rejoin_index = candidate.rejoin_index;
    active_plan_.minimum_clearance = candidate.min_clearance;
    active_plan_.maximum_curvature = candidate.max_curvature;
    active_plan_.peak_offset = candidate.peak_offset;
    active_plan_.start_lateral_offset = candidate.start_lateral_offset;
    active_plan_.rolling_pass = candidate.rolling_pass;
    // Publishing the first accepted path closes the direction decision. Any
    // later material replan must stay on this side until the pass anchor has
    // been crossed; scan noise cannot reopen the decision mid-maneuver.
    active_plan_.side_committed = candidate.side != 0;
    active_plan_.phase = candidate.side == 0 ?
      ManeuverPhase::RECOVERING : ManeuverPhase::DEPARTING;
    updateActivePhase();
    active_plan_.created_at = steady_clock_.now();
    rejoin_stable_cycles_ = 0;
    active_blocked_cycles_ = 0;
    active_physical_blocked_cycles_ = 0;
    resetNoSafePathConfirmation();
    terminal_safe_yield_cycles_ = 0;
    last_terminal_safe_yield_scan_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
    rolling_pass_clear_cycles_ = 0;
    RCLCPP_INFO(
      get_logger(),
      "activated local plan %lu: mode=%s side=%s rejoin_index=%zu reason=%s",
      active_plan_.id, active_plan_.source_mode.c_str(),
      active_plan_.side > 0 ? "LEFT" : (active_plan_.side < 0 ? "RIGHT" : "NONE"),
      active_plan_.rejoin_index, reason.c_str());
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
    active_physical_blocked_cycles_ = 0;
    resetNoSafePathConfirmation();
    terminal_safe_yield_cycles_ = 0;
    last_terminal_safe_yield_scan_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
    rolling_pass_clear_cycles_ = 0;
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
    const nav_msgs::msg::Path & raw_path,
    const path_splice::ActivePathHandoffMode handoff_mode =
    path_splice::ActivePathHandoffMode::STRICT_SPLICE) const
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

    if (handoff_mode == path_splice::ActivePathHandoffMode::FRESH_RAW_PATH &&
      raw_path.header.frame_id == output.header.frame_id)
    {
      // The active-plan release still waits for multi-scan convergence, but
      // once the car is geometrically back on the raceline the old modified
      // tail may be shorter than the follower lookahead. Keep the confirmation
      // state while handing control geometry to the fresh forward raw window.
      output.poses.reserve(raw_path.poses.size());
      for (const auto & pose : raw_path.poses) {
        if (!append_pose(pose)) {
          break;
        }
      }
      return output;
    }

    const std::size_t start = active_plan_.progress_index > 0 ?
      active_plan_.progress_index - 1 : 0;
    std::size_t stored_tail_start = active_plan_.path.poses.size();

    // Keep only the still-relevant, locally modified part of the stored plan.
    // The stale raw-raceline tail saved when the plan was created is replaced
    // below by the newest raw 10 m window.
    if (start <= active_plan_.rejoin_index) {
      const std::size_t local_end = std::min(
        active_plan_.rejoin_index, active_plan_.path.poses.size() - 1);
      stored_tail_start = local_end + 1;
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

    // Once the local curve reaches the raceline, splice in the current raw path.
    // If the car has already passed the rejoin index during the short release
    // confirmation, publish the fresh raw path directly. Otherwise locate the
    // matching rejoin point in the current raw window and append only the
    // raceline ahead of it.
    // Splice only when gap, heading, and curvature are continuous.
    std::size_t raw_start = output.poses.empty() ? 0 : raw_path.poses.size();
    if (output.poses.size() >= 2 && raw_path.poses.size() >= 2) {
      const auto & previous_position =
        output.poses[output.poses.size() - 2].pose.position;
      const auto & anchor_position = output.poses.back().pose.position;
      const path_splice::Point2 previous{
        previous_position.x, previous_position.y};
      const path_splice::Point2 anchor{
        anchor_position.x, anchor_position.y};
      std::vector<path_splice::Point2> raw_points;
      raw_points.reserve(raw_path.poses.size());
      for (const auto & pose : raw_path.poses) {
        raw_points.push_back(path_splice::Point2{
          pose.pose.position.x, pose.pose.position.y});
      }
      const double curvature_limit = curvature_safety_factor_ *
        std::tan(steering_max_deg_ * M_PI / 180.0) / wheelbase_m_;
      const auto match = path_splice::findContinuousTailStart(
        previous, anchor, raw_points,
        raceline_splice_max_gap_m_,
        raceline_splice_max_heading_error_deg_ * M_PI / 180.0,
        curvature_limit);
      if (match.valid) {
        raw_start = match.raw_start_index;
      }
    }

    if (!output.poses.empty() && raw_start >= raw_path.poses.size()) {
      // Preserve the validated stored tail until a live splice is available.
      for (std::size_t i = stored_tail_start; i < active_plan_.path.poses.size(); ++i) {
        if (!append_pose(active_plan_.path.poses[i])) {
          break;
        }
      }
      return output;
    }

    for (std::size_t i = raw_start; i < raw_path.poses.size(); ++i) {
      if (!append_pose(raw_path.poses[i])) {
        break;
      }
    }
    return output;
  }

  SafeYieldPlan buildSafeYieldPlan(
    const nav_msgs::msg::Path & source_path,
    const std::vector<ScanHit> & hits,
    const Point2 & robot_position, const double robot_yaw) const
  {
    SafeYieldPlan yield;
    if (!enable_primary_safe_yield_) {
      yield.reason = "primary safe-yield state is disabled";
      return yield;
    }
    if (source_path.header.frame_id.empty() || source_path.poses.size() < 3) {
      yield.reason = "source path is too short for a safe-yield prefix";
      return yield;
    }

    std::size_t nearest = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < source_path.poses.size(); ++index) {
      const auto & position = source_path.poses[index].pose.position;
      const double separation = std::hypot(
        position.x - robot_position.x, position.y - robot_position.y);
      if (separation < nearest_distance) {
        nearest_distance = separation;
        nearest = index;
      }
    }
    if (nearest + 2 >= source_path.poses.size()) {
      yield.reason = "source path ends before a safe-yield prefix can be formed";
      return yield;
    }

    std::vector<Point2> points;
    points.reserve(source_path.poses.size() - nearest);
    points.push_back(robot_position);
    for (std::size_t index = nearest + 1; index < source_path.poses.size(); ++index) {
      const auto & position = source_path.poses[index].pose.position;
      const Point2 point{position.x, position.y};
      if (distance(points.back(), point) > 1e-5) {
        points.push_back(point);
      }
    }
    if (points.size() < 3) {
      yield.reason = "source path has too few distinct forward points";
      return yield;
    }

    const auto clusters = clusterScanHits(hits, planning_distance_m_);
    const auto segments = polyline_query::prepareSegments(points);
    double traversed = 0.0;
    for (std::size_t segment_index = 0; segment_index < segments.size(); ++segment_index) {
      const auto & segment = segments[segment_index];
      const double dx = segment.dx;
      const double dy = segment.dy;
      const double segment_length = std::hypot(dx, dy);
      if (segment_length <= 1e-6) {
        continue;
      }
      for (const auto & cluster : clusters) {
        int interfering = 0;
        double first_contact = segment_length;
        for (const auto & hit : cluster.hits) {
          if (hit.projection.s < -0.10 ||
            !polyline_query::anyWithinDistance(
              hit.point.x, hit.point.y, planningClearance(),
              segments, segment_index, segment_index + 1))
          {
            continue;
          }
          interfering += static_cast<int>(hit.support_count);
          const double along = std::clamp(
            ((hit.point.x - segment.ax) * dx + (hit.point.y - segment.ay) * dy) /
            (segment_length * segment_length), 0.0, 1.0) * segment_length;
          first_contact = std::min(first_contact, along);
        }
        if (interfering >= blocked_min_points_) {
          yield.collision_distance = std::min(
            yield.collision_distance, traversed + first_contact);
        }
      }
      traversed += segment_length;
    }
    if (!std::isfinite(yield.collision_distance)) {
      yield.reason = "no confirmed connected interference was found on the source path";
      return yield;
    }

    const double stop_length = yield.collision_distance - yield_standoff_m_;
    if (yield.collision_distance <= planningClearance()) {
      yield.reason = "confirmed interference is inside the controlled-yield safety boundary";
      return yield;
    }

    // Retain a short endpoint so steering remains defined at zero speed.
    const double path_target_length = std::max(0.05, stop_length);

    yield.path.header = source_path.header;
    yield.path.header.stamp = now();
    geometry_msgs::msg::PoseStamped first_pose;
    first_pose.header = yield.path.header;
    first_pose.pose.position.x = robot_position.x;
    first_pose.pose.position.y = robot_position.y;
    first_pose.pose.orientation = yawToQuaternion(robot_yaw);
    yield.path.poses.push_back(first_pose);

    traversed = 0.0;
    for (std::size_t segment = 0; segment + 1 < points.size(); ++segment) {
      const Point2 & first = points[segment];
      const Point2 & second = points[segment + 1];
      const double segment_length = distance(first, second);
      if (segment_length <= 1e-6) {
        continue;
      }
      const double remaining = path_target_length - traversed;
      if (remaining <= 1e-6) {
        break;
      }
      const double used = std::min(segment_length, remaining);
      const double ratio = used / segment_length;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = yield.path.header;
      pose.pose.position.x = first.x + ratio * (second.x - first.x);
      pose.pose.position.y = first.y + ratio * (second.y - first.y);
      pose.pose.orientation = yawToQuaternion(std::atan2(
        second.y - first.y, second.x - first.x));
      yield.path.poses.push_back(pose);
      traversed += used;
      if (used + 1e-9 < segment_length) {
        break;
      }
    }
    if (yield.path.poses.size() < 2 || traversed < 0.04) {
      yield.reason = "safe-yield prefix could not preserve a forward endpoint";
      yield.path.poses.clear();
      return yield;
    }

    yield.path_length = traversed;
    const double braking_room = std::max(
      0.0, yield.path_length - yield_min_path_length_m_);
    yield.speed_cap = std::min(
      yield_max_speed_mps_,
      std::sqrt(2.0 * yield_deceleration_mps2_ * braking_room));
    yield.valid = true;
    yield.reason = "temporarily occupied pass corridor has a collision-free yielding prefix";
    return yield;
  }

  void resetNoSafePathConfirmation()
  {
    no_safe_path_cycles_ = 0;
    last_no_safe_path_scan_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
  }

  bool noSafePathFailureConfirmed(
    const bool confirmable, const rclcpp::Time & scan_time)
  {
    const auto update = active_path_safety::updateDistinctScanConfirmation(
      {no_safe_path_cycles_, last_no_safe_path_scan_time_.nanoseconds()},
      scan_time.nanoseconds(), confirmable, no_safe_path_confirmation_scans_);
    no_safe_path_cycles_ = update.state.consecutive_scans;
    if (update.consumed) {
      last_no_safe_path_scan_time_ = scan_time;
    }
    return update.confirmed;
  }

  bool terminalSafeYieldFailureConfirmed(
    const SafeYieldPlan & safe_yield, const rclcpp::Time & scan_time)
  {
    const bool terminal = !std::isfinite(safe_yield.speed_cap) ||
      safe_yield.speed_cap <= std::max(0.0, safe_yield_terminal_speed_mps_);
    const auto update = active_path_safety::updateDistinctScanConfirmation(
      {terminal_safe_yield_cycles_, last_terminal_safe_yield_scan_time_.nanoseconds()},
      scan_time.nanoseconds(), terminal, no_safe_path_confirmation_scans_);
    terminal_safe_yield_cycles_ = update.state.consecutive_scans;
    if (update.consumed) {
      last_terminal_safe_yield_scan_time_ = scan_time;
    }
    return update.confirmed;
  }

  void updateActivePhase()
  {
    if (!active_plan_.valid) {
      return;
    }
    if (active_plan_.side == 0) {
      active_plan_.phase = ManeuverPhase::RECOVERING;
      active_plan_.side_committed = false;
    } else if (active_plan_.progress_index < active_plan_.pass_start_index) {
      active_plan_.phase = ManeuverPhase::DEPARTING;
      active_plan_.side_committed = true;
    } else if (active_plan_.progress_index <= active_plan_.pass_end_index) {
      active_plan_.phase = ManeuverPhase::PASSING;
      active_plan_.side_committed = true;
    } else {
      active_plan_.phase = ManeuverPhase::RETURNING;
      active_plan_.side_committed = false;
    }
  }

  std::string activeTrajectoryMode() const
  {
    if (!active_plan_.valid) {
      return "NONE";
    }
    switch (active_plan_.phase) {
      case ManeuverPhase::DEPARTING:
        return "AVOIDANCE_DEPARTING";
      case ManeuverPhase::PASSING:
        return "AVOIDANCE_PASSING";
      case ManeuverPhase::RETURNING:
        return "AVOIDANCE_RETURNING";
      case ManeuverPhase::RECOVERING:
        return "RECOVERING_TO_RACELINE";
      case ManeuverPhase::OPEN:
      default:
        return "NONE";
    }
  }

  double remainingActivePathLength() const
  {
    if (!active_plan_.valid || active_plan_.path.poses.size() < 2) {
      return 0.0;
    }
    const std::size_t begin = std::min(
      active_plan_.progress_index, active_plan_.path.poses.size() - 1);
    const std::size_t end = std::min(
      active_plan_.rejoin_index, active_plan_.path.poses.size() - 1);
    double length = 0.0;
    for (std::size_t index = begin; index < end; ++index) {
      const auto & first = active_plan_.path.poses[index].pose.position;
      const auto & second = active_plan_.path.poses[index + 1].pose.position;
      length += std::hypot(second.x - first.x, second.y - first.y);
    }
    return length;
  }

  bool pathBlockedByConnectedHits(
    const nav_msgs::msg::Path & path, const std::vector<ScanHit> & hits,
    const double clearance_threshold) const
  {
    if (path.poses.size() < 2) {
      return true;
    }
    std::vector<Point2> path_points;
    path_points.reserve(path.poses.size());
    for (const auto & pose : path.poses) {
      path_points.push_back(Point2{
        pose.pose.position.x, pose.pose.position.y});
    }
    const auto path_segments = polyline_query::prepareSegments(path_points);
    int connected = 0;
    int maximum_connected = 0;
    std::size_t previous_beam = 0;
    Point2 previous_point;
    bool previous_interfered = false;
    for (const auto & hit : hits) {
      // Ignore returns behind the forward-only trajectory.
      if (hit.projection.s < -0.10) {
        continue;
      }
      const bool interfered = polyline_query::anyWithinDistance(
        hit.point.x, hit.point.y, clearance_threshold, path_segments);
      if (interfered) {
        const bool connected_to_previous = previous_interfered &&
          hit.beam_index <= previous_beam + static_cast<std::size_t>(cluster_max_beam_gap_) &&
          distance(hit.point, previous_point) <= cluster_point_gap_m_;
        const int support = static_cast<int>(hit.support_count);
        connected = connected_to_previous ? connected + support : support;
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

  bool activePathBlocked(
    const std::vector<ScanHit> & hits, const double clearance_threshold,
    const nav_msgs::msg::Path & executable_path) const
  {
    if (!active_plan_.valid || executable_path.poses.size() < 2) {
      return true;
    }
    // Validate the exact path that will be published for this control cycle.
    return pathBlockedByConnectedHits(
      executable_path, hits, clearance_threshold);
  }

  double remainingActiveMaximumCurvature() const
  {
    if (!active_plan_.valid || active_plan_.path.poses.size() < 3) {
      return 0.0;
    }
    const std::size_t begin = std::max<std::size_t>(
      1, active_plan_.progress_index > 0 ? active_plan_.progress_index - 1 : 1);
    const std::size_t end = std::min(
      active_plan_.rejoin_index,
      active_plan_.path.poses.size() - static_cast<std::size_t>(1));
    double maximum_curvature = 0.0;
    for (std::size_t index = begin; index < end; ++index) {
      const auto & previous_position = active_plan_.path.poses[index - 1].pose.position;
      const auto & current_position = active_plan_.path.poses[index].pose.position;
      const auto & next_position = active_plan_.path.poses[index + 1].pose.position;
      const Point2 previous{previous_position.x, previous_position.y};
      const Point2 current{current_position.x, current_position.y};
      const Point2 next{next_position.x, next_position.y};
      const double a = distance(previous, current);
      const double b = distance(current, next);
      const double c = distance(previous, next);
      if (a <= 1e-6 || b <= 1e-6 || c <= 1e-6) {
        continue;
      }
      const double cross = std::abs(
        (current.x - previous.x) * (next.y - previous.y) -
        (current.y - previous.y) * (next.x - previous.x));
      maximum_curvature = std::max(maximum_curvature, 2.0 * cross / (a * b * c));
    }
    return maximum_curvature;
  }

  double activeSpeedCap() const
  {
    const double configured_ceiling = active_plan_.side == 0 ?
      recovery_speed_cap_mps_ : avoidance_speed_cap_mps_;
    return maneuver_speed::curvatureLimitedSpeed(
      configured_ceiling, command_speed_max_mps_,
      maneuver_lateral_acceleration_limit_mps2_, remainingActiveMaximumCurvature());
  }

  double activeReplanPendingSpeedCap() const
  {
    return active_plan_.valid ?
      std::min(replan_pending_speed_cap_mps_, activeSpeedCap()) :
      replan_pending_speed_cap_mps_;
  }

  std_msgs::msg::Float64 publishSpeedCap(const double cap)
  {
    std_msgs::msg::Float64 message;
    message.data = std::max(0.0, cap);
    last_published_speed_cap_mps_ = message.data;
    speed_cap_pub_->publish(message);
    return message;
  }

  nav_msgs::msg::Path publishPath(nav_msgs::msg::Path path)
  {
    path.header.stamp = now();
    for (auto & pose : path.poses) {
      pose.header = path.header;
    }
    final_path_pub_->publish(path);
    return path;
  }

  diagnostic_msgs::msg::DiagnosticArray publishStatus(
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
    add(
      "planning_clearance_reserve_m",
      std::to_string(planning_clearance_reserve_m_));
    add("planning_clearance_m", std::to_string(planningClearance()));
    add("planning_distance_m", std::to_string(planning_distance_m_));
    add("scan_range_cap_m", std::to_string(scan_range_cap_m_));
    add(
      "planning_reference",
      candidate ? candidate->planning_reference :
      (active_plan_.valid ? active_plan_.planning_reference : last_reference_source_));
    add("raceline_reference_loaded", raceline_reference_valid_ ? "true" : "false");
    add("raceline_reference_point_count", std::to_string(raceline_reference_points_.size()));
    add(
      "raceline_reference_match_tolerance_m",
      std::to_string(raceline_reference_match_tolerance_m_));
    add(
      "lattice_evaluated_transitions",
      std::to_string(last_lattice_evaluated_transitions_));
    add(
      "lattice_clearance_grid_time_ms",
      std::to_string(last_lattice_clearance_grid_time_ms_));
    add("lattice_compute_time_ms", std::to_string(last_lattice_compute_time_ms_));
    add("lattice_result", last_lattice_reason_);
    add("static_obstacle_fast_mode", static_obstacle_fast_mode_ ? "true" : "false");
    add(
      "static_analytic_candidates_evaluated",
      std::to_string(last_static_analytic_candidates_));
    add("static_analytic_time_ms", std::to_string(last_static_analytic_time_ms_));
    add("static_side_score_time_ms", std::to_string(last_static_side_score_time_ms_));
    add("static_planning_total_time_ms", std::to_string(last_static_total_time_ms_));
    add("static_left_map_score_m", std::to_string(last_static_left_map_score_m_));
    add("static_right_map_score_m", std::to_string(last_static_right_map_score_m_));
    add("static_planning_backend", last_static_backend_);
    add(
      "effective_obstacle_detection_distance_m",
      std::to_string(last_effective_detection_distance_m_));
    add("speed_cap_mps", std::to_string(last_published_speed_cap_mps_));
    add(
      "safe_yield_terminal_speed_mps",
      std::to_string(safe_yield_terminal_speed_mps_));
    add(
      "remaining_maximum_curvature_inv_m",
      active_plan_.valid ? std::to_string(remainingActiveMaximumCurvature()) : "0.0");
    add(
      "maneuver_lateral_acceleration_limit_mps2",
      std::to_string(maneuver_lateral_acceleration_limit_mps2_));
    add("local_plan_active", active_plan_.valid ? "true" : "false");
    add(
      "rolling_pass_active",
      active_plan_.valid && active_plan_.rolling_pass ? "true" : "false");
    add("obstacle_track_valid", obstacle_track_.valid ? "true" : "false");
    add("obstacle_track_moving", obstacle_track_.moving ? "true" : "false");
    add(
      "obstacle_track_longitudinal_speed_mps",
      std::to_string(obstacle_track_.longitudinal_speed));
    add("obstacle_track_observations", std::to_string(obstacle_track_.observations));
    add("obstacle_track_missing_scans", std::to_string(obstacle_track_.missing_scans));
    add("rolling_pass_clear_cycles", std::to_string(rolling_pass_clear_cycles_));
    add(
      "side_committed",
      active_plan_.valid && active_plan_.side != 0 ?
      (active_plan_.side_committed ? "true" : "false") : "n/a");
    add("plan_id", active_plan_.valid ? std::to_string(active_plan_.id) : "0");
    add(
      "plan_age_sec",
      active_plan_.valid ?
      std::to_string((steady_clock_.now() - active_plan_.created_at).seconds()) : "0.0");
    add("maneuver_phase", active_plan_.valid ? activeTrajectoryMode() : "OPEN");
    add(
      "plan_progress_index",
      active_plan_.valid ? std::to_string(active_plan_.progress_index) : "0");
    add(
      "plan_rejoin_index",
      active_plan_.valid ? std::to_string(active_plan_.rejoin_index) : "0");
    add("rejoin_stable_cycles", std::to_string(rejoin_stable_cycles_));
    add(
      "raceline_handoff_started",
      active_plan_.valid && active_plan_.raceline_handoff_started ? "true" : "false");
    add("active_blocked_cycles", std::to_string(active_blocked_cycles_));
    add("active_margin_blocked_cycles", std::to_string(active_blocked_cycles_));
    add(
      "active_physical_blocked_cycles",
      std::to_string(active_physical_blocked_cycles_));
    add("no_safe_path_cycles", std::to_string(no_safe_path_cycles_));
    add(
      "terminal_safe_yield_cycles",
      std::to_string(terminal_safe_yield_cycles_));
    add("candidate_decision_id", std::to_string(candidate_decision_id_));
    add("candidate_decision_context", candidate_decision_context_);
    add(
      "candidate_selected_side",
      candidate_decision_selected_side_ > 0 ? "LEFT" :
      (candidate_decision_selected_side_ < 0 ? "RIGHT" : "NONE"));
    const auto add_candidate_summary = [&add](
        const std::string & prefix, const CandidateSummary & summary) {
        add(prefix + "_evaluated", summary.evaluated ? "true" : "false");
        add(prefix + "_valid", summary.valid ? "true" : "false");
        add(prefix + "_reason", summary.reason);
        add(prefix + "_minimum_clearance_m", std::to_string(summary.minimum_clearance));
        add(prefix + "_objective_cost", std::to_string(summary.objective_cost));
        add(
          prefix + "_objective_domain",
          summary.objective_domain == selection::ObjectiveDomain::LATTICE ?
          "LATTICE" : "NONE");
        add(prefix + "_maximum_curvature_inv_m", std::to_string(summary.maximum_curvature));
        add(prefix + "_peak_offset_m", std::to_string(summary.peak_offset));
        add(
          prefix + "_maximum_connected_interference",
          std::to_string(summary.maximum_connected_interference));
      };
    add_candidate_summary("left_candidate", last_left_candidate_);
    add_candidate_summary("right_candidate", last_right_candidate_);
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
      add("planner_objective_cost", std::to_string(candidate->objective_cost));
    }
    array.status.push_back(status);
    status_pub_->publish(array);
    return array;
  }

  void startPlanningHeartbeat(
    nav_msgs::msg::Path path, const double speed_cap,
    const std::string & reason, const int side,
    const double raw_path_age, const double scan_age,
    const double valid_beam_ratio, const ObstacleCluster * obstacle = nullptr)
  {
    stopPlanningHeartbeat();

    // Publish the conservative hold immediately before starting expensive
    // candidate search. The dedicated heartbeat callback then refreshes this
    // coherent cap/path/status bundle while planning is still in progress.
    auto cap_message = publishSpeedCap(speed_cap);
    auto path_message = publishPath(std::move(path));
    auto status_message = publishStatus(
      "READY", reason, "PLANNING_HOLD", side,
      raw_path_age, scan_age, valid_beam_ratio, obstacle);

    std::lock_guard<std::mutex> lock(planning_heartbeat_mutex_);
    planning_heartbeat_.started_at = steady_clock_.now();
    planning_heartbeat_.path = std::move(path_message);
    planning_heartbeat_.speed_cap = cap_message;
    planning_heartbeat_.status = std::move(status_message);
    planning_heartbeat_.active = true;
  }

  void stopPlanningHeartbeat()
  {
    std::lock_guard<std::mutex> lock(planning_heartbeat_mutex_);
    planning_heartbeat_.active = false;
  }

  bool finishPlanningHeartbeat()
  {
    std::lock_guard<std::mutex> lock(planning_heartbeat_mutex_);
    const bool valid = planning_heartbeat_.active &&
      (steady_clock_.now() - planning_heartbeat_.started_at).seconds() <=
      planning_heartbeat_timeout_sec_;
    planning_heartbeat_.active = false;
    return valid;
  }

  static void updateDiagnosticValue(
    diagnostic_msgs::msg::DiagnosticStatus & status,
    const std::string & key, const std::string & value)
  {
    for (auto & pair : status.values) {
      if (pair.key == key) {
        pair.value = value;
        return;
      }
    }
  }

  InputFreshness currentInputFreshness()
  {
    rclcpp::Time raw_time(0, 0, RCL_STEADY_TIME);
    rclcpp::Time current_scan_time(0, 0, RCL_STEADY_TIME);
    InputFreshness freshness;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      freshness.raw_received = raw_path_received_ && latest_raw_path_;
      freshness.scan_received = scan_received_ && latest_scan_;
      raw_time = raw_path_time_;
      current_scan_time = scan_time_;
    }
    const auto current_time = steady_clock_.now();
    if (freshness.raw_received) {
      freshness.raw_age = (current_time - raw_time).seconds();
    }
    if (freshness.scan_received) {
      freshness.scan_age = (current_time - current_scan_time).seconds();
    }
    return freshness;
  }

  void failPlanningHeartbeatLocked(
    const std::string & state, const std::string & reason)
  {
    planning_heartbeat_.active = false;
    std_msgs::msg::Float64 stop;
    stop.data = 0.0;
    speed_cap_pub_->publish(stop);

    auto failure_status = planning_heartbeat_.status;
    failure_status.header.stamp = now();
    for (auto & status : failure_status.status) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = state + ": " + reason;
      updateDiagnosticValue(status, "state", state);
      updateDiagnosticValue(status, "reason", reason);
      updateDiagnosticValue(status, "trajectory_mode", "NONE");
      updateDiagnosticValue(status, "speed_cap_mps", "0.0");
    }
    status_pub_->publish(failure_status);
  }

  void planningHeartbeatLoop()
  {
    std::lock_guard<std::mutex> lock(planning_heartbeat_mutex_);
    if (!planning_heartbeat_.active) {
      return;
    }

    const InputFreshness freshness = currentInputFreshness();
    updateDiagnosticValue(
      planning_heartbeat_.status.status.front(), "raw_path_age_sec",
      std::to_string(freshness.raw_age));
    updateDiagnosticValue(
      planning_heartbeat_.status.status.front(), "scan_age_sec",
      std::to_string(freshness.scan_age));
    if (!freshness.scan_received || freshness.scan_age > scan_timeout_sec_) {
      failPlanningHeartbeatLocked(
        "SCAN_INVALID", freshness.scan_received ?
        "LaserScan became stale during planning" :
        "LaserScan is unavailable during planning");
      return;
    }
    if (!freshness.raw_received || freshness.raw_age > raw_path_timeout_sec_) {
      failPlanningHeartbeatLocked(
        "INPUT_INVALID", freshness.raw_received ?
        "raw raceline path became stale during planning" :
        "raw raceline path is unavailable during planning");
      return;
    }

    const double hold_age =
      (steady_clock_.now() - planning_heartbeat_.started_at).seconds();
    if (hold_age > planning_heartbeat_timeout_sec_) {
      failPlanningHeartbeatLocked(
        "PLANNING_TIMEOUT", "obstacle-plan computation exceeded heartbeat window");
      RCLCPP_WARN(
        get_logger(),
        "planning heartbeat expired after %.3f s; holding zero speed",
        hold_age);
      return;
    }

    planning_heartbeat_.path.header.stamp = now();
    for (auto & pose : planning_heartbeat_.path.poses) {
      pose.header = planning_heartbeat_.path.header;
    }
    planning_heartbeat_.status.header.stamp = now();

    // Cap first ensures a refreshed path can never briefly carry an older,
    // faster maneuver ceiling. Status is last so arbitration sees a coherent
    // output bundle.
    speed_cap_pub_->publish(planning_heartbeat_.speed_cap);
    final_path_pub_->publish(planning_heartbeat_.path);
    status_pub_->publish(planning_heartbeat_.status);
  }

  bool finishPlanningAttempt(
    const rclcpp::Time & started_at, const bool heartbeat_started)
  {
    const bool heartbeat_valid = !heartbeat_started || finishPlanningHeartbeat();
    const InputFreshness freshness = currentInputFreshness();
    if (!freshness.scan_received || freshness.scan_age > scan_timeout_sec_) {
      failPrimary(
        "SCAN_INVALID", freshness.scan_received ?
        "LaserScan became stale during planning" :
        "LaserScan is unavailable during planning",
        freshness.raw_age, freshness.scan_age);
      return false;
    }
    if (!freshness.raw_received || freshness.raw_age > raw_path_timeout_sec_) {
      failPrimary(
        "INPUT_INVALID", freshness.raw_received ?
        "raw raceline path became stale during planning" :
        "raw raceline path is unavailable during planning",
        freshness.raw_age, freshness.scan_age);
      return false;
    }
    if ((steady_clock_.now() - started_at).seconds() > planning_heartbeat_timeout_sec_) {
      failPrimary(
        "PLANNING_TIMEOUT", "obstacle-plan computation exceeded heartbeat window",
        freshness.raw_age, freshness.scan_age);
      return false;
    }
    return heartbeat_valid;
  }

  LatestCandidateValidation validateCandidateOnLatestScan(
    const Candidate & candidate, const PathModel & raw)
  {
    LatestCandidateValidation validation;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan;
    rclcpp::Time latest_scan_time(0, 0, RCL_STEADY_TIME);
    bool scan_received = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_scan = latest_scan_;
      latest_scan_time = scan_time_;
      scan_received = scan_received_;
    }
    validation.scan_time = latest_scan_time;
    const double scan_age = scan_received ?
      (steady_clock_.now() - latest_scan_time).seconds() :
      std::numeric_limits<double>::infinity();
    if (!scan_received || !latest_scan || scan_age > scan_timeout_sec_) {
      validation.failure_state = "SCAN_INVALID";
      validation.failure_reason = scan_received ?
        "LaserScan became stale before candidate acceptance" :
        "LaserScan is unavailable before candidate acceptance";
      return validation;
    }

    if (!scanHitsInPathFrame(
        *latest_scan, raw, validation.hits, validation.valid_beam_ratio))
    {
      validation.failure_state = "TF_UNAVAILABLE";
      validation.failure_reason =
        "latest LaserScan cannot be transformed before candidate acceptance";
      return validation;
    }
    if (validation.valid_beam_ratio < min_valid_beam_ratio_) {
      validation.failure_state = "SCAN_INVALID";
      validation.failure_reason = "latest LaserScan has insufficient valid beams";
      return validation;
    }
    if (pathBlockedByConnectedHits(
        candidate.path, validation.hits, safety_half_width_m_))
    {
      if (!lookupRobotPose(
          raw.frame, latest_scan->header.stamp,
          validation.robot_position, validation.robot_yaw))
      {
        validation.failure_state = "TF_UNAVAILABLE";
        validation.failure_reason =
          "latest LaserScan robot pose is unavailable before active-path fallback";
        return validation;
      }
      validation.outcome = active_path_safety::CandidateValidationOutcome::COLLISION;
      validation.failure_state = "PLAN_INVALIDATED";
      validation.failure_reason = "fresh LaserScan blocks the newly generated candidate";
      return validation;
    }
    validation.outcome = active_path_safety::CandidateValidationOutcome::CLEAR;
    validation.failure_state.clear();
    validation.failure_reason.clear();
    return validation;
  }

  active_path_safety::ActiveCandidateDisposition handleActiveCandidateValidation(
    const LatestCandidateValidation & validation,
    const nav_msgs::msg::Path & held_active_path,
    const bool held_path_ended_without_rejoin,
    const ObstacleCluster * obstacle = nullptr)
  {
    const bool held_path_physically_clear =
      validation.outcome == active_path_safety::CandidateValidationOutcome::COLLISION &&
      !pathBlockedByConnectedHits(
        held_active_path, validation.hits, safety_half_width_m_);
    const auto disposition = active_path_safety::activeCandidateDisposition(
      validation.outcome, held_path_physically_clear,
      held_path_ended_without_rejoin);
    if (disposition ==
      active_path_safety::ActiveCandidateDisposition::ACTIVATE_REPLACEMENT)
    {
      return disposition;
    }
    if (disposition ==
      active_path_safety::ActiveCandidateDisposition::CONTINUE_FAILURE_HANDLING)
    {
      return disposition;
    }

    const auto freshness = currentInputFreshness();
    if (disposition ==
      active_path_safety::ActiveCandidateDisposition::RETAIN_ACTIVE_PATH)
    {
      resetNoSafePathConfirmation();
      terminal_safe_yield_cycles_ = 0;
      last_terminal_safe_yield_scan_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
      active_physical_blocked_cycles_ = 0;
      publishSpeedCap(activeReplanPendingSpeedCap());
      publishPath(held_active_path);
      publishStatus(
        "READY",
        "fresh LaserScan rejected the replacement candidate; retaining the "
        "physically clear active path at reduced speed",
        "REPLAN_PENDING", active_plan_.side,
        freshness.raw_age, freshness.scan_age,
        validation.valid_beam_ratio, obstacle);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "replacement candidate rejected by fresh LaserScan; retaining active plan %lu",
        active_plan_.id);
      return disposition;
    }

    failPrimary(
      validation.failure_state, validation.failure_reason,
      freshness.raw_age, freshness.scan_age,
      validation.valid_beam_ratio, obstacle);
    return disposition;
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
    stopPlanningHeartbeat();
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
    std::vector<ScanHit> full_resolution_hits;
    double valid_beam_ratio = 0.0;
    if (!scanHitsInPathFrame(
        *scan, raw, full_resolution_hits, valid_beam_ratio))
    {
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
    if (!lookupRobotPose(raw.frame, scan->header.stamp, robot_position, robot_yaw)) {
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
      geometry_msgs::msg::Vector3Stamped heading_error_message;
      heading_error_message.header.stamp = scan->header.stamp;
      heading_error_message.header.frame_id = raw.frame;
      heading_error_message.vector.x = car_heading_error;
      heading_error_message.vector.y = robot_on_raw.distance;
      heading_error_message.vector.z = robot_on_raw.s;
      heading_error_pub_->publish(heading_error_message);
    }

    last_effective_detection_distance_m_ = effectiveObstacleDetectionDistance(raw);
    const auto clusters = clusterScanHits(
      full_resolution_hits, last_effective_detection_distance_m_);
    ObstacleCluster obstacle;
    const bool obstacle_found = selectNearestObstacle(clusters, obstacle);
    const double allowed_extent = max_obstacle_size_m_ + obstacle_size_tolerance_m_;
    const bool obstacle_size_valid = !obstacle_found ||
      ((obstacle.s_max - obstacle.s_min <= allowed_extent) &&
      (obstacle.d_max - obstacle.d_min <= allowed_extent));
    if (static_obstacle_fast_mode_) {
      // The static profile deliberately has no velocity estimation or
      // rolling/open-ended pass state. Moving returns are treated as ordinary
      // instantaneous obstacles by the same collision validators.
      if (obstacle_track_.valid) {
        obstacle_track_ = ObstacleTrack();
      }
      rolling_pass_clear_cycles_ = 0;
    } else {
      updateObstacleTrack(
        clusters, obstacle_found ? &obstacle : nullptr, raw,
        current_scan_time, new_scan);
    }
    std::vector<ScanHit> pooled_planning_hits;
    bool planning_hits_prepared = false;
    const auto getPlanningHits = [&]() -> const std::vector<ScanHit> & {
        if (planning_scan_pool_size_ <= 1) {
          return full_resolution_hits;
        }
        if (!planning_hits_prepared) {
          // Reuse already transformed/projected endpoints. This avoids a
          // second TF lookup and a second trigonometric/projection pass.
          pooled_planning_hits =
            path_following_v2::scan_pooling::minimumRangePoolProjectedHits(
              full_resolution_hits,
              static_cast<std::size_t>(planning_scan_pool_size_));
          planning_hits_prepared = true;
        }
        return pooled_planning_hits;
      };
    RacelinePlanningContext raceline_context;
    bool raceline_context_prepared = false;
    const auto getRacelineContext = [&]() -> const RacelinePlanningContext & {
        if (!raceline_context_prepared) {
          last_lattice_evaluated_transitions_ = 0;
          last_lattice_compute_time_ms_ = 0.0;
          last_lattice_clearance_grid_time_ms_ = 0.0;
          last_lattice_reason_ = "not run";
          raceline_context = prepareRacelinePlanningContext(
            raw, getPlanningHits(), robot_position, robot_yaw,
            obstacle_found ? &obstacle : nullptr);
          raceline_context_prepared = true;
        }
        return raceline_context;
      };
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
      updateActivePhase();
      const double plan_age = (steady_clock_.now() - active_plan_.created_at).seconds();
      const double heading_error = std::abs(car_heading_error);
      const bool rejoin_geometry_reached =
        plan_age >= minimum_plan_hold_sec_ &&
        active_plan_.progress_index >= active_plan_.rejoin_index &&
        std::abs(robot_on_raw.d) <= recovery_exit_lateral_error_m_ &&
        heading_error <= recovery_exit_heading_error_deg_ * M_PI / 180.0;
      const auto handoff_mode = path_splice::activePathHandoffMode(
        active_plan_.raceline_handoff_started, rejoin_geometry_reached,
        active_plan_.progress_index, active_plan_.rejoin_index);
      active_plan_.raceline_handoff_started =
        handoff_mode == path_splice::ActivePathHandoffMode::FRESH_RAW_PATH;
      if (new_scan) {
        rejoin_stable_cycles_ = rejoin_geometry_reached ?
          rejoin_stable_cycles_ + 1 : 0;
      }
      if (rejoin_stable_cycles_ >= rejoin_confirmation_scans_) {
        releaseActivePlan("rejoin anchor passed and lateral/heading convergence confirmed");
      }

      if (active_plan_.valid) {
        const bool end_without_rejoin = path_splice::storedPathEndedWithoutRejoin(
          active_plan_.raceline_handoff_started,
          active_plan_.progress_index + 10 >= active_plan_.path.poses.size(),
          rejoin_geometry_reached);
        const auto held_active_path = continuousActivePath(
          *raw_message, handoff_mode);
        LatestCandidateValidation rejected_candidate_validation;
        bool candidate_rejected_on_latest_scan = false;
        const double tracked_obstacle_s = static_obstacle_fast_mode_ ?
          std::numeric_limits<double>::infinity() : trackedObstacleCurrentS(raw);
        const bool rolling_target_cleared = !static_obstacle_fast_mode_ &&
          active_plan_.rolling_pass &&
          !obstacle_found &&
          (tracked_obstacle_s <= -rolling_pass_rear_clearance_m_ ||
          obstacle_track_.missing_scans >= rolling_pass_completion_scans_) &&
          active_plan_.progress_index >= active_plan_.pass_start_index;
        if (new_scan) {
          rolling_pass_clear_cycles_ = rolling_target_cleared ?
            rolling_pass_clear_cycles_ + 1 : 0;
        }
        if (!static_obstacle_fast_mode_ && active_plan_.rolling_pass && !obstacle_found &&
          rolling_pass_clear_cycles_ >= rolling_pass_completion_scans_)
        {
          const auto planning_started_at = steady_clock_.now();
          startPlanningHeartbeat(
            held_active_path, activeReplanPendingSpeedCap(),
            "computing smooth recovery while retaining the current clear path",
            active_plan_.side, raw_age, scan_age, valid_beam_ratio);
          Candidate recovery = bestAvailableRecoveryCandidate(
            getRacelineContext(), raw, *raw_message, getPlanningHits(),
            robot_position, robot_yaw, robot_on_raw.d, car_heading_error);
          if (!finishPlanningAttempt(planning_started_at, true)) {
            return;
          }
          if (recovery.valid) {
            auto validation = validateCandidateOnLatestScan(recovery, raw);
            const auto disposition = handleActiveCandidateValidation(
              validation, held_active_path, end_without_rejoin);
            if (disposition ==
              active_path_safety::ActiveCandidateDisposition::RETAIN_ACTIVE_PATH ||
              disposition == active_path_safety::ActiveCandidateDisposition::FAIL_PRIMARY)
            {
              return;
            }
            if (disposition ==
              active_path_safety::ActiveCandidateDisposition::CONTINUE_FAILURE_HANDLING)
            {
              rejected_candidate_validation = std::move(validation);
              candidate_rejected_on_latest_scan = true;
              recovery.valid = false;
            }
          }
          if (recovery.valid) {
            activatePlan(recovery, "moving target passed with confirmed rear clearance");
            obstacle_track_ = ObstacleTrack();
            publishPath(continuousActivePath(*raw_message));
            publishSpeedCap(activeSpeedCap());
            publishStatus(
              "READY", "rolling pass completed; smooth raceline recovery accepted",
              activeTrajectoryMode(), active_plan_.side,
              raw_age, scan_age, valid_beam_ratio, nullptr, &recovery);
            return;
          }
        }

        const bool margin_blocked_now = activePathBlocked(
          full_resolution_hits, planningClearance(), held_active_path);
        const bool physical_blocked_now = activePathBlocked(
          full_resolution_hits, safety_half_width_m_, held_active_path);
        if (new_scan) {
          active_blocked_cycles_ = margin_blocked_now ? active_blocked_cycles_ + 1 : 0;
          active_physical_blocked_cycles_ = physical_blocked_now ?
            active_physical_blocked_cycles_ + 1 : 0;
        }
        const bool stale_plan = plan_age >= maximum_plan_hold_sec_;
        const bool excessive_deviation = deviation > plan_deviation_replan_m_;
        const bool blocked_confirmed =
          active_blocked_cycles_ >= active_path_blocked_confirmation_scans_;
        const bool physical_blocked_confirmed =
          active_physical_blocked_cycles_ >= active_path_blocked_confirmation_scans_;
        const bool rolling_upgrade_requested = !static_obstacle_fast_mode_ && obstacle_found &&
          trackedObstacleMatches(obstacle) && obstacle_track_.moving &&
          !active_plan_.rolling_pass;
        const bool rolling_refresh_requested = !static_obstacle_fast_mode_ &&
          active_plan_.rolling_pass &&
          obstacle_found && trackedObstacleMatches(obstacle) &&
          remainingActivePathLength() <= rolling_pass_refresh_remaining_m_;
        const bool rolling_continuation_requested = !static_obstacle_fast_mode_ &&
          active_plan_.rolling_pass &&
          !obstacle_found &&
          remainingActivePathLength() <= rolling_pass_refresh_remaining_m_;
        const bool update_requested = blocked_confirmed || stale_plan ||
          excessive_deviation || end_without_rejoin ||
          rolling_upgrade_requested || rolling_refresh_requested ||
          rolling_continuation_requested || candidate_rejected_on_latest_scan;

        if (!update_requested) {
          resetNoSafePathConfirmation();
          terminal_safe_yield_cycles_ = 0;
          last_terminal_safe_yield_scan_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
          publishPath(held_active_path);
          if (margin_blocked_now) {
            publishSpeedCap(activeReplanPendingSpeedCap());
            publishStatus(
              "READY",
              physical_blocked_now ?
              "physical active-path blockage awaiting fresh-scan confirmation" :
              "planning reserve encroachment awaiting fresh-scan confirmation",
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
        const auto planning_started_at = steady_clock_.now();
        bool heartbeat_started = false;
        const bool latest_held_path_blocked = candidate_rejected_on_latest_scan &&
          pathBlockedByConnectedHits(
          held_active_path, rejected_candidate_validation.hits, safety_half_width_m_);
        const auto & planning_safety_hits = candidate_rejected_on_latest_scan ?
          rejected_candidate_validation.hits : full_resolution_hits;
        const auto & planning_safety_robot_position = candidate_rejected_on_latest_scan ?
          rejected_candidate_validation.robot_position : robot_position;
        const double planning_safety_robot_yaw = candidate_rejected_on_latest_scan ?
          rejected_candidate_validation.robot_yaw : robot_yaw;
        if (!physical_blocked_now && !latest_held_path_blocked) {
          startPlanningHeartbeat(
            held_active_path, activeReplanPendingSpeedCap(),
            "updating obstacle plan while retaining the physically clear active path",
            active_plan_.side, raw_age, scan_age, valid_beam_ratio,
            obstacle_found ? &obstacle : nullptr);
          heartbeat_started = true;
        } else {
          const auto planning_yield = buildSafeYieldPlan(
            held_active_path, planning_safety_hits,
            planning_safety_robot_position, planning_safety_robot_yaw);
          if (planning_yield.valid) {
            startPlanningHeartbeat(
              planning_yield.path,
              std::min(planning_yield.speed_cap, activeReplanPendingSpeedCap()),
              "updating obstacle plan while following a collision-free yield prefix",
              active_plan_.side, raw_age, scan_age, valid_beam_ratio,
              obstacle_found ? &obstacle : nullptr);
            heartbeat_started = true;
          }
        }
        if (obstacle_found && enable_detour_planning_ && obstacle_size_valid) {
          if (static_obstacle_fast_mode_) {
            resetStaticFastSearchDiagnostics();
          }
          const bool preserve_legacy_backend =
            active_plan_.objective_domain == selection::ObjectiveDomain::NONE;
          const auto active_side_candidate = [&](const int side) {
              if (static_obstacle_fast_mode_) {
                return bestStaticObstacleCandidateForSide(
                  getRacelineContext, raw, *raw_message, obstacle, getPlanningHits(),
                  robot_position, robot_yaw, robot_on_raw.d,
                  car_heading_error, side);
              }
              if (preserve_legacy_backend) {
                last_reference_source_ = "racingline_legacy_continuous_replan";
                Candidate candidate = bestDetourCandidateForSide(
                  raw, *raw_message, obstacle, getPlanningHits(),
                  robot_position, robot_yaw, robot_on_raw.d,
                  car_heading_error, side);
                if (!candidate.valid) {
                  candidate = bestDetourCandidateForSide(
                    raw, *raw_message, obstacle, getPlanningHits(),
                    robot_position, robot_yaw, robot_on_raw.d,
                    car_heading_error, side, true);
                }
                return candidate;
              }
              return bestAvailableDetourCandidateForSide(
                getRacelineContext(), raw, *raw_message, obstacle, getPlanningHits(),
                robot_position, robot_yaw, robot_on_raw.d, car_heading_error,
                side, false);
            };
          // Reopen side selection only after the current pass anchor is behind.
          const bool side_decision_closed = active_plan_.side_committed;
          if (side_decision_closed && active_plan_.side > 0) {
            left = active_side_candidate(+1);
            selected = left.valid ? &left : nullptr;
          } else if (side_decision_closed && active_plan_.side < 0) {
            right = active_side_candidate(-1);
            selected = right.valid ? &right : nullptr;
          } else if (static_obstacle_fast_mode_) {
            selected = selectStaticObstacleCandidate(
              getRacelineContext, raw, *raw_message, obstacle, getPlanningHits(),
              robot_position, robot_yaw, robot_on_raw.d,
              car_heading_error, left, right);
          } else {
            left = active_side_candidate(+1);
            right = active_side_candidate(-1);
            selected = chooseCandidate(left, right);
          }
          recordCandidateDecision(
            left, right, selected, "active obstacle-path replan");
          publishMarkers(
            obstacle, left, right,
            selected ? selected->side : active_plan_.side, raw.frame);
          update_reason = side_decision_closed ?
            "same-side active path update around observed obstacle" :
            "next obstacle selected from the wider verified corridor";
          if (rolling_upgrade_requested) {
            update_reason = "moving target confirmed; committed side extended for a rolling pass";
          } else if (rolling_refresh_requested) {
            update_reason = "rolling pass corridor refreshed on the committed side";
          }
        } else if (!obstacle_found) {
          clearMarkers(raw.frame);
          if (!static_obstacle_fast_mode_ && active_plan_.rolling_pass) {
            replacement = buildRollingContinuationCandidate(
              raw, *raw_message, getPlanningHits(), robot_position, robot_yaw,
              robot_on_raw.d, active_plan_.side);
            selected = replacement.valid ? &replacement : nullptr;
            update_reason = "rolling pass horizon continued on the committed side";
          } else if (active_plan_.side == 0 ||
            active_plan_.phase == ManeuverPhase::RETURNING)
          {
            replacement = bestAvailableRecoveryCandidate(
              getRacelineContext(), raw, *raw_message, getPlanningHits(),
              robot_position, robot_yaw, robot_on_raw.d, car_heading_error, false);
            selected = replacement.valid ? &replacement : nullptr;
            update_reason = "active return path update toward raceline";
          } else {
            update_reason = "avoidance direction remains closed until the pass anchor";
          }
        }
        if (!finishPlanningAttempt(planning_started_at, heartbeat_started)) {
          return;
        }
        if (selected) {
          auto validation = validateCandidateOnLatestScan(*selected, raw);
          const auto disposition = handleActiveCandidateValidation(
            validation, held_active_path, end_without_rejoin,
            obstacle_found ? &obstacle : nullptr);
          if (disposition ==
            active_path_safety::ActiveCandidateDisposition::RETAIN_ACTIVE_PATH ||
            disposition == active_path_safety::ActiveCandidateDisposition::FAIL_PRIMARY)
          {
            return;
          }
          if (disposition ==
            active_path_safety::ActiveCandidateDisposition::CONTINUE_FAILURE_HANDLING)
          {
            rejected_candidate_validation = std::move(validation);
            candidate_rejected_on_latest_scan = true;
            selected = nullptr;
          }
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

        // Yield on a safe prefix while retrying the committed side.
        SafeYieldPlan safe_yield;
        const auto & failure_safety_hits = candidate_rejected_on_latest_scan ?
          rejected_candidate_validation.hits : full_resolution_hits;
        const auto & failure_robot_position = candidate_rejected_on_latest_scan ?
          rejected_candidate_validation.robot_position : robot_position;
        const double failure_robot_yaw = candidate_rejected_on_latest_scan ?
          rejected_candidate_validation.robot_yaw : robot_yaw;
        if (blocked_confirmed || candidate_rejected_on_latest_scan) {
          safe_yield = buildSafeYieldPlan(
            held_active_path, failure_safety_hits,
            failure_robot_position, failure_robot_yaw);
        }
        const auto & failure_scan_time = candidate_rejected_on_latest_scan ?
          rejected_candidate_validation.scan_time : current_scan_time;
        const auto failure_valid_beam_ratio = candidate_rejected_on_latest_scan ?
          rejected_candidate_validation.valid_beam_ratio : valid_beam_ratio;
        if (safe_yield.valid) {
          resetNoSafePathConfirmation();
          if (terminalSafeYieldFailureConfirmed(safe_yield, failure_scan_time))
          {
            failPrimary(
              "NO_SAFE_PATH_CONFIRMED",
              "active plan has only a terminal safe-yield prefix; transferring to Reactive",
              raw_age, scan_age, failure_valid_beam_ratio,
              obstacle_found ? &obstacle : nullptr);
            return;
          }
          publishPath(safe_yield.path);
          publishSpeedCap(safe_yield.speed_cap);
          publishStatus(
            "READY",
            candidate_rejected_on_latest_scan ?
            "replacement candidate rejected; following the held path's safe-yield prefix" :
            "committed pass corridor is temporarily occupied; holding a safe following gap",
            "FOLLOWING_OBSTACLE", active_plan_.side,
            raw_age, scan_age, failure_valid_beam_ratio,
            obstacle_found ? &obstacle : nullptr);
          return;
        }
        terminal_safe_yield_cycles_ = 0;
        last_terminal_safe_yield_scan_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);

        const auto failure_disposition =
          active_path_safety::replacementFailureDisposition(
            physical_blocked_confirmed, end_without_rejoin);
        const bool update_failure_confirmable =
          failure_disposition ==
          active_path_safety::ReplacementFailureDisposition::CONFIRM_PRIMARY_FAILURE;
        if (noSafePathFailureConfirmed(
            update_failure_confirmable, failure_scan_time))
        {
          failPrimary(
            "NO_SAFE_PATH_CONFIRMED",
            "active plan remained blocked and no valid replacement was found",
            raw_age, scan_age, failure_valid_beam_ratio,
            obstacle_found ? &obstacle : nullptr);
          return;
        }

        publishSpeedCap(activeReplanPendingSpeedCap());
        publishPath(held_active_path);
        publishStatus(
          "READY",
          candidate_rejected_on_latest_scan ?
          "replacement candidate rejected; active-path failure awaiting confirmation" :
          blocked_confirmed && !physical_blocked_confirmed ?
          "planning reserve encroached; holding physically clear active path at reduced speed" :
          "active-plan update pending multi-scan confirmation",
          "REPLAN_PENDING", active_plan_.side,
          raw_age, scan_age, failure_valid_beam_ratio,
          obstacle_found ? &obstacle : nullptr);
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
      const auto planning_started_at = steady_clock_.now();
      bool heartbeat_started = false;
      if (obstacle_found && enable_detour_planning_ && obstacle_size_valid) {
        const auto planning_yield = buildSafeYieldPlan(
          *raw_message, full_resolution_hits, robot_position, robot_yaw);
        if (planning_yield.valid) {
          startPlanningHeartbeat(
            planning_yield.path,
            std::min(planning_yield.speed_cap, replan_pending_speed_cap_mps_),
            "computing avoidance while following a collision-free raceline prefix",
            0, raw_age, scan_age, valid_beam_ratio, &obstacle);
          heartbeat_started = true;
        }
        if (static_obstacle_fast_mode_) {
          resetStaticFastSearchDiagnostics();
          selected = selectStaticObstacleCandidate(
            getRacelineContext, raw, *raw_message, obstacle, getPlanningHits(),
            robot_position, robot_yaw, robot_on_raw.d,
            car_heading_error, left, right);
        } else {
          left = bestAvailableDetourCandidateForSide(
            getRacelineContext(), raw, *raw_message, obstacle, getPlanningHits(),
            robot_position, robot_yaw, robot_on_raw.d, car_heading_error, +1);
          right = bestAvailableDetourCandidateForSide(
            getRacelineContext(), raw, *raw_message, obstacle, getPlanningHits(),
            robot_position, robot_yaw, robot_on_raw.d, car_heading_error, -1);
          selected = chooseCandidate(left, right);
        }
        recordCandidateDecision(
          left, right, selected, "new obstacle-path decision");
        publishMarkers(obstacle, left, right, selected ? selected->side : 0, raw.frame);
        failure_reason = "no valid local passage (left: " + left.reason +
          "; right: " + right.reason + ")";
      } else if (obstacle_found && !enable_detour_planning_) {
        failure_reason = "raw raceline is blocked and local planning is disabled";
      } else if (obstacle_found && !obstacle_size_valid) {
        failure_reason = "interfering cluster exceeds configured obstacle-size rule";
      } else {
        clearMarkers(raw.frame);
        startPlanningHeartbeat(
          *raw_message, replan_pending_speed_cap_mps_,
          "computing raceline recovery while retaining the clear raw path",
          0, raw_age, scan_age, valid_beam_ratio);
        heartbeat_started = true;
        recovery = bestAvailableRecoveryCandidate(
          getRacelineContext(), raw, *raw_message, getPlanningHits(),
          robot_position, robot_yaw,
          robot_on_raw.d, car_heading_error);
        selected = recovery.valid ? &recovery : nullptr;
        failure_reason = "no valid raceline-recovery trajectory: " + recovery.reason;
      }
      if (!finishPlanningAttempt(planning_started_at, heartbeat_started)) {
        return;
      }
      if (selected) {
        const auto validation = validateCandidateOnLatestScan(*selected, raw);
        if (validation.outcome !=
          active_path_safety::CandidateValidationOutcome::CLEAR)
        {
          const auto freshness = currentInputFreshness();
          failPrimary(
            validation.failure_state, validation.failure_reason,
            freshness.raw_age, freshness.scan_age,
            validation.valid_beam_ratio,
            obstacle_found ? &obstacle : nullptr, selected);
          return;
        }
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


      // Stop before the obstacle when both passing sides are unavailable.
      const SafeYieldPlan safe_yield = buildSafeYieldPlan(
        *raw_message, full_resolution_hits, robot_position, robot_yaw);
      if (obstacle_found && safe_yield.valid) {
        resetNoSafePathConfirmation();
        if (terminalSafeYieldFailureConfirmed(safe_yield, current_scan_time))
        {
          failPrimary(
            "NO_SAFE_PATH_CONFIRMED",
            failure_reason + "; terminal safe-yield persisted",
            raw_age, scan_age, valid_beam_ratio, &obstacle);
          return;
        }
        publishPath(safe_yield.path);
        publishSpeedCap(safe_yield.speed_cap);
        publishStatus(
          "READY",
          "no safe overtake side is open yet; maintaining a controlled following gap",
          "FOLLOWING_OBSTACLE", 0,
          raw_age, scan_age, valid_beam_ratio, &obstacle);
        return;
      }
      terminal_safe_yield_cycles_ = 0;
      last_terminal_safe_yield_scan_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);

      if (noSafePathFailureConfirmed(true, current_scan_time)) {
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

    resetNoSafePathConfirmation();
    terminal_safe_yield_cycles_ = 0;
    last_terminal_safe_yield_scan_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
    active_blocked_cycles_ = 0;
    active_physical_blocked_cycles_ = 0;
    publishPath(*raw_message);
    publishSpeedCap(command_speed_max_mps_);
    publishStatus(
      "READY", "raw local raceline is clear and car is within recovery threshold",
      "RACELINE", 0, raw_age, scan_age, valid_beam_ratio);
    clearMarkers(raw.frame);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocalTrajectoryPlannerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
