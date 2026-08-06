#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

struct RacelineWaypoint
{
  int index{0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double curvature{0.0};
  double curvature_abs{0.0};
};

class PathGeneratorNode : public rclcpp::Node
{
public:
  PathGeneratorNode()
  : Node("path_generator"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("raceline_waypoints_topic", "/raceline_waypoints");
    declare_parameter<std::string>("local_path_topic", "/path_following_v2/local_path");
    declare_parameter<std::string>("rule_speed_index_topic", "/path_following_v2/rule_speed_index");
    declare_parameter<std::string>("status_topic", "/path_following_v2/path_status");
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<std::string>("robot_frame", "ego_racecar/base_link");

    declare_parameter<double>("publish_rate_hz", 20.0);
    declare_parameter<double>("tf_timeout_sec", 0.05);
    declare_parameter<int>("local_path_horizon_points", 80);

    // Rule-based speed preview. The steering pure-pursuit lookahead lives in path_following_v2_node.
    // Keep the old parameter names declared for compatibility, but use the clearer names below.
    declare_parameter<double>("speed_min", 1.0);
    declare_parameter<double>("speed_max", 10.0);
    declare_parameter<double>("rule_min_speed_mps", 1.0);
    declare_parameter<double>("rule_max_speed_mps", 6.0);
    declare_parameter<double>("rule_speed_curvature_gain", 2.0);
    declare_parameter<int>("rule_speed_curvature_lookahead_points", 3);
    declare_parameter<double>("curvature_speed_gain", 2.0);          // deprecated alias
    declare_parameter<int>("curvature_lookahead_points", 3);         // deprecated alias

    raceline_waypoints_topic_ = get_parameter("raceline_waypoints_topic").as_string();
    local_path_topic_ = get_parameter("local_path_topic").as_string();
    rule_speed_index_topic_ = get_parameter("rule_speed_index_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    global_frame_ = get_parameter("global_frame").as_string();
    robot_frame_ = get_parameter("robot_frame").as_string();

    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();
    local_path_horizon_points_ = get_parameter("local_path_horizon_points").as_int();

    speed_min_ = get_parameter("speed_min").as_double();
    speed_max_ = get_parameter("speed_max").as_double();
    rule_min_speed_mps_ = get_parameter("rule_min_speed_mps").as_double();
    rule_max_speed_mps_ = get_parameter("rule_max_speed_mps").as_double();
    rule_speed_curvature_gain_ = get_parameter("rule_speed_curvature_gain").as_double();
    rule_speed_curvature_lookahead_points_ = get_parameter("rule_speed_curvature_lookahead_points").as_int();

    if (local_path_horizon_points_ < 2) {
      RCLCPP_WARN(get_logger(), "local_path_horizon_points must be >= 2; forcing to 2.");
      local_path_horizon_points_ = 2;
    }

    if (speed_min_ > speed_max_) {
      RCLCPP_WARN(get_logger(), "speed_min > speed_max; swapping them.");
      std::swap(speed_min_, speed_max_);
    }
    rule_min_speed_mps_ = std::clamp(rule_min_speed_mps_, speed_min_, speed_max_);
    rule_max_speed_mps_ = std::clamp(rule_max_speed_mps_, rule_min_speed_mps_, speed_max_);

    raceline_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      raceline_waypoints_topic_,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&PathGeneratorNode::racelineCallback, this, std::placeholders::_1));

    local_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      local_path_topic_, rclcpp::QoS(1).reliable().transient_local());

    rule_speed_index_pub_ = create_publisher<std_msgs::msg::Float64>(rule_speed_index_topic_, 10);
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(status_topic_, 10);

    const auto period_ms = std::chrono::milliseconds(
      static_cast<int>(1000.0 / std::max(1.0, publish_rate_hz_)));
    timer_ = create_wall_timer(period_ms, std::bind(&PathGeneratorNode::publishLoop, this));

    RCLCPP_INFO(get_logger(), "path_generator started as clean raceline local-path generator");
    RCLCPP_INFO(get_logger(), "  raceline_waypoints_topic: %s", raceline_waypoints_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  local_path_topic: %s", local_path_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  rule_speed_index_topic: %s", rule_speed_index_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  status_topic: %s", status_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  physical speed range: %.2f to %.2f m/s", speed_min_, speed_max_);
    RCLCPP_INFO(get_logger(), "  rule speed range: %.2f to %.2f m/s", rule_min_speed_mps_, rule_max_speed_mps_);
    RCLCPP_INFO(get_logger(), "  local_path_horizon_points: %d", local_path_horizon_points_);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr raceline_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rule_speed_index_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<RacelineWaypoint> raceline_;
  bool raceline_valid_{false};
  bool raceline_received_{false};

  std::string raceline_waypoints_topic_;
  std::string local_path_topic_;
  std::string rule_speed_index_topic_;
  std::string status_topic_;
  std::string global_frame_;
  std::string robot_frame_;

  double publish_rate_hz_{20.0};
  double tf_timeout_sec_{0.05};
  double speed_min_{1.0};
  double speed_max_{10.0};
  double rule_min_speed_mps_{1.0};
  double rule_max_speed_mps_{6.0};
  double rule_speed_curvature_gain_{2.0};

  int local_path_horizon_points_{80};
  int rule_speed_curvature_lookahead_points_{3};

  void racelineCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    constexpr std::size_t fields = 6;
    raceline_received_ = true;

    if (msg->data.empty() || msg->data.size() % fields != 0) {
      RCLCPP_WARN(
        get_logger(),
        "Received invalid raceline waypoint data size: %zu. Expected rows of [index,x,y,yaw,curvature,curvature_abs].",
        msg->data.size());
      raceline_valid_ = false;
      return;
    }

    std::vector<RacelineWaypoint> parsed;
    parsed.reserve(msg->data.size() / fields);

    for (std::size_t i = 0; i + fields - 1 < msg->data.size(); i += fields) {
      RacelineWaypoint wp;
      wp.index = static_cast<int>(std::lround(msg->data[i + 0]));
      wp.x = msg->data[i + 1];
      wp.y = msg->data[i + 2];
      wp.yaw = msg->data[i + 3];
      wp.curvature = msg->data[i + 4];
      wp.curvature_abs = msg->data[i + 5];
      parsed.push_back(wp);
    }

    raceline_ = std::move(parsed);
    raceline_valid_ = !raceline_.empty();

    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Received %zu raceline waypoints.",
      raceline_.size());
  }

  bool lookupRobotPose(tf2::Transform & tf_map_to_base)
  {
    try {
      const auto tf_msg = tf_buffer_.lookupTransform(
        global_frame_,
        robot_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));

      tf2::fromMsg(tf_msg.transform, tf_map_to_base);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF lookup failed (%s -> %s): %s",
        global_frame_.c_str(),
        robot_frame_.c_str(),
        ex.what());
      return false;
    }
  }

  int findNearestRacelineIndex(const tf2::Transform & tf_map_to_base) const
  {
    if (raceline_.empty()) {
      return 0;
    }

    const double rx = tf_map_to_base.getOrigin().x();
    const double ry = tf_map_to_base.getOrigin().y();

    int best_idx = 0;
    double best_dist2 = std::numeric_limits<double>::max();

    for (std::size_t i = 0; i < raceline_.size(); ++i) {
      const double dx = raceline_[i].x - rx;
      const double dy = raceline_[i].y - ry;
      const double d2 = dx * dx + dy * dy;

      if (d2 < best_dist2) {
        best_dist2 = d2;
        best_idx = static_cast<int>(i);
      }
    }

    return best_idx;
  }

  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) const
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(q);
  }

  nav_msgs::msg::Path buildLocalRacelinePath(int nearest_idx) const
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = global_frame_;

    if (raceline_.empty()) {
      return path;
    }

    const int n = static_cast<int>(raceline_.size());
    const int horizon = std::min(local_path_horizon_points_, n);
    path.poses.reserve(static_cast<std::size_t>(horizon));

    for (int step = 0; step < horizon; ++step) {
      const int idx = ((nearest_idx + step) % n + n) % n;
      const auto & wp = raceline_[idx];

      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = wp.x;
      pose.pose.position.y = wp.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = yawToQuaternion(wp.yaw);
      path.poses.push_back(pose);
    }

    return path;
  }

  double computeRuleSpeedIndex(int nearest_idx) const
  {
    if (raceline_.empty()) {
      return 0.0;
    }

    const int n = static_cast<int>(raceline_.size());
    const int preview_points = std::max(0, rule_speed_curvature_lookahead_points_);

    // Match rl_training: use the maximum curvature_abs from nearest_idx to nearest_idx + N.
    double curvature_abs = 0.0;
    for (int offset = 0; offset <= preview_points; ++offset) {
      const int idx = ((nearest_idx + offset) % n + n) % n;
      curvature_abs = std::max(curvature_abs, std::max(0.0, raceline_[idx].curvature_abs));
    }

    // Match rl_training curvature_based_speed():
    //   speed = rule_max / (1 + curvature_gain * curvature_abs), clipped to rule range.
    // Then convert the physical rule speed into the final normalized speed index range.
    const double raw_rule_speed_mps =
      rule_max_speed_mps_ / (1.0 + rule_speed_curvature_gain_ * curvature_abs);
    const double rule_speed_mps =
      std::clamp(raw_rule_speed_mps, rule_min_speed_mps_, rule_max_speed_mps_);

    if (speed_max_ <= speed_min_) {
      return 0.0;
    }
    const double speed_index = (rule_speed_mps - speed_min_) / (speed_max_ - speed_min_);
    return std::clamp(speed_index, 0.0, 1.0);
  }

  void publishStatus(
    const std::string & state, const std::string & reason,
    const int nearest_idx = -1, const std::size_t local_path_points = 0)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "path_following_v2/path_generator";
    status.hardware_id = "raceline_path_generator";
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
    add("raceline_received", raceline_received_ ? "true" : "false");
    add("raceline_valid", raceline_valid_ ? "true" : "false");
    add("raceline_points", std::to_string(raceline_.size()));
    add("local_path_points", std::to_string(local_path_points));
    add("nearest_index", std::to_string(nearest_idx));
    array.status.push_back(status);
    status_pub_->publish(array);
  }

  void publishLoop()
  {
    if (!raceline_valid_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No valid raceline waypoints yet.");
      publishStatus(
        raceline_received_ ? "RACELINE_INVALID" : "WAITING_RACELINE",
        raceline_received_ ? "received raceline data are invalid" :
        "no raceline waypoints received");
      return;
    }

    tf2::Transform tf_map_to_base;
    if (!lookupRobotPose(tf_map_to_base)) {
      // Never publish a plausible-looking path from index zero when the
      // localization-dependent map -> base transform is unavailable.
      publishStatus("TF_UNAVAILABLE", "map-to-robot transform lookup failed");
      return;
    }
    const int nearest_idx = findNearestRacelineIndex(tf_map_to_base);

    const auto local_path = buildLocalRacelinePath(nearest_idx);
    if (local_path.poses.size() < 2) {
      publishStatus(
        "LOCAL_PATH_INVALID", "generated local path has fewer than two poses",
        nearest_idx, local_path.poses.size());
      return;
    }
    local_path_pub_->publish(local_path);

    std_msgs::msg::Float64 speed_index_msg;
    speed_index_msg.data = computeRuleSpeedIndex(nearest_idx);
    rule_speed_index_pub_->publish(speed_index_msg);
    publishStatus(
      "READY", "valid local raceline path and rule speed published",
      nearest_idx, local_path.poses.size());

    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "local raceline: nearest=%d, horizon=%d, rule_speed_index=%.3f, current_curv_abs=%.3f",
      nearest_idx,
      local_path_horizon_points_,
      speed_index_msg.data,
      raceline_[nearest_idx].curvature_abs);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
