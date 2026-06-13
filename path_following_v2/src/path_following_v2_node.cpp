#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class PathFollowingV2Node : public rclcpp::Node
{
public:
  PathFollowingV2Node()
  : Node("path_following_v2"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("local_path_topic", "/path_following_v2/local_path");
    declare_parameter<std::string>("rule_speed_index_topic", "/path_following_v2/rule_speed_index");
    declare_parameter<std::string>("rl_speed_residual_topic", "/rl_speed_inference/speed_residual_mps");
    declare_parameter<std::string>("drive_topic", "/drive");
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<std::string>("robot_frame", "ego_racecar/base_link");

    declare_parameter<double>("control_rate_hz", 20.0);
    declare_parameter<double>("path_timeout_sec", 1.0);
    declare_parameter<double>("rule_speed_index_timeout_sec", 0.5);
    declare_parameter<double>("rl_residual_timeout_sec", 0.5);
    declare_parameter<double>("tf_timeout_sec", 0.1);

    // speed_mode = 0: rule-based speed only.
    // speed_mode = 1: rule-based speed + fresh RL residual, otherwise rule-based speed only.
    declare_parameter<int>("speed_mode", 0);
    declare_parameter<double>("speed_min", 1.0);
    declare_parameter<double>("speed_max", 10.0);
    declare_parameter<double>("max_speed_delta_per_step_mps", 0.2);

    declare_parameter<double>("fixed_steering_lookahead_m", 0.60);
    declare_parameter<bool>("use_speed_dependent_steering_lookahead", true);
    declare_parameter<double>("steering_min_lookahead_m", 0.6);
    declare_parameter<double>("steering_max_lookahead_m", 1.6);
    declare_parameter<double>("steering_lookahead_speed_gain", 0.25);
    // Deprecated aliases kept to avoid breaking older YAML files immediately.
    declare_parameter<double>("fixed_lookahead_distance", 0.60);
    declare_parameter<bool>("use_speed_dependent_lookahead", true);
    declare_parameter<double>("min_lookahead", 0.6);
    declare_parameter<double>("max_lookahead", 1.6);
    declare_parameter<double>("lookahead_speed_gain", 0.25);

    declare_parameter<double>("wheelbase", 0.33);
    declare_parameter<double>("steering_max_deg", 20.6);
    declare_parameter<double>("min_forward_point_x", 0.05);
    declare_parameter<bool>("stop_if_no_path", true);

    declare_parameter<bool>("publish_markers", true);
    declare_parameter<std::string>("marker_frame", "ego_racecar/base_link");
    declare_parameter<std::string>("lookahead_marker_topic", "/path_following_v2/lookahead_marker");
    declare_parameter<std::string>("steering_marker_topic", "/path_following_v2/steering_marker");

    local_path_topic_ = get_parameter("local_path_topic").as_string();
    rule_speed_index_topic_ = get_parameter("rule_speed_index_topic").as_string();
    rl_speed_residual_topic_ = get_parameter("rl_speed_residual_topic").as_string();
    drive_topic_ = get_parameter("drive_topic").as_string();
    global_frame_ = get_parameter("global_frame").as_string();
    robot_frame_ = get_parameter("robot_frame").as_string();

    control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    path_timeout_sec_ = get_parameter("path_timeout_sec").as_double();
    rule_speed_index_timeout_sec_ = get_parameter("rule_speed_index_timeout_sec").as_double();
    rl_residual_timeout_sec_ = get_parameter("rl_residual_timeout_sec").as_double();
    tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();

    speed_mode_ = get_parameter("speed_mode").as_int();
    speed_min_ = get_parameter("speed_min").as_double();
    speed_max_ = get_parameter("speed_max").as_double();
    max_speed_delta_per_step_mps_ = get_parameter("max_speed_delta_per_step_mps").as_double();

    fixed_steering_lookahead_m_ = get_parameter("fixed_steering_lookahead_m").as_double();
    use_speed_dependent_steering_lookahead_ = get_parameter("use_speed_dependent_steering_lookahead").as_bool();
    steering_min_lookahead_m_ = get_parameter("steering_min_lookahead_m").as_double();
    steering_max_lookahead_m_ = get_parameter("steering_max_lookahead_m").as_double();
    steering_lookahead_speed_gain_ = get_parameter("steering_lookahead_speed_gain").as_double();

    wheelbase_ = get_parameter("wheelbase").as_double();
    steering_max_deg_ = get_parameter("steering_max_deg").as_double();
    min_forward_point_x_ = get_parameter("min_forward_point_x").as_double();
    stop_if_no_path_ = get_parameter("stop_if_no_path").as_bool();

    publish_markers_ = get_parameter("publish_markers").as_bool();
    marker_frame_ = get_parameter("marker_frame").as_string();
    lookahead_marker_topic_ = get_parameter("lookahead_marker_topic").as_string();
    steering_marker_topic_ = get_parameter("steering_marker_topic").as_string();

    if (speed_min_ > speed_max_) {
      RCLCPP_WARN(get_logger(), "speed_min > speed_max; swapping them.");
      std::swap(speed_min_, speed_max_);
    }
    if (speed_mode_ != 0 && speed_mode_ != 1) {
      RCLCPP_WARN(get_logger(), "Unsupported speed_mode=%d; forcing rule-based mode 0.", speed_mode_);
      speed_mode_ = 0;
    }
    max_speed_delta_per_step_mps_ = std::max(0.0, max_speed_delta_per_step_mps_);
    last_commanded_speed_ = speed_min_;

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      local_path_topic_,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&PathFollowingV2Node::pathCallback, this, std::placeholders::_1));

    rule_speed_index_sub_ = create_subscription<std_msgs::msg::Float64>(
      rule_speed_index_topic_, 10,
      std::bind(&PathFollowingV2Node::ruleSpeedIndexCallback, this, std::placeholders::_1));

    rl_speed_residual_sub_ = create_subscription<std_msgs::msg::Float64>(
      rl_speed_residual_topic_, 10,
      std::bind(&PathFollowingV2Node::rlSpeedResidualCallback, this, std::placeholders::_1));

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);

    if (publish_markers_) {
      lookahead_marker_pub_ =
        create_publisher<visualization_msgs::msg::Marker>(lookahead_marker_topic_, 10);
      steering_marker_pub_ =
        create_publisher<visualization_msgs::msg::Marker>(steering_marker_topic_, 10);
    }

    const auto period_ms = std::chrono::milliseconds(
      static_cast<int>(1000.0 / std::max(1.0, control_rate_hz_)));
    timer_ = create_wall_timer(period_ms, std::bind(&PathFollowingV2Node::controlLoop, this));

    RCLCPP_INFO(get_logger(), "path_following_v2 started as pure-pursuit follower");
    RCLCPP_INFO(get_logger(), "  local_path_topic: %s", local_path_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  rule_speed_index_topic: %s", rule_speed_index_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  rl_speed_residual_topic: %s", rl_speed_residual_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  drive_topic: %s", drive_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  speed_mode: %d (%s)", speed_mode_, speed_mode_ == 0 ? "rule_based" : "rule_plus_rl_residual");
    RCLCPP_INFO(get_logger(), "  final speed range: %.2f to %.2f m/s", speed_min_, speed_max_);
    RCLCPP_INFO(get_logger(), "  max_speed_delta_per_step_mps: %.2f", max_speed_delta_per_step_mps_);
    RCLCPP_INFO(
      get_logger(),
      "  use_speed_dependent_steering_lookahead: %s",
      use_speed_dependent_steering_lookahead_ ? "true" : "false");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rule_speed_index_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rl_speed_residual_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr steering_marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  nav_msgs::msg::Path latest_path_;
  bool path_valid_{false};
  bool rule_speed_index_valid_{false};
  bool rl_speed_residual_valid_{false};
  bool rl_residual_was_fresh_{false};
  double latest_rule_speed_index_{0.0};
  double latest_rl_speed_residual_mps_{0.0};
  double last_commanded_speed_{0.0};
  rclcpp::Time last_path_receive_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_rule_speed_index_receive_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_rl_speed_residual_receive_time_{0, 0, RCL_STEADY_TIME};

  std::string local_path_topic_;
  std::string rule_speed_index_topic_;
  std::string rl_speed_residual_topic_;
  std::string drive_topic_;
  std::string global_frame_;
  std::string robot_frame_;
  std::string marker_frame_;
  std::string lookahead_marker_topic_;
  std::string steering_marker_topic_;

  double control_rate_hz_{20.0};
  double path_timeout_sec_{1.0};
  double rule_speed_index_timeout_sec_{0.5};
  double rl_residual_timeout_sec_{0.5};
  double tf_timeout_sec_{0.1};

  int speed_mode_{0};
  double speed_min_{1.0};
  double speed_max_{10.0};
  double max_speed_delta_per_step_mps_{0.2};

  double fixed_steering_lookahead_m_{0.60};
  bool use_speed_dependent_steering_lookahead_{true};
  double steering_min_lookahead_m_{0.6};
  double steering_max_lookahead_m_{1.6};
  double steering_lookahead_speed_gain_{0.25};

  double wheelbase_{0.33};
  double steering_max_deg_{20.6};
  double min_forward_point_x_{0.05};

  bool stop_if_no_path_{true};
  bool publish_markers_{true};

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty local raceline path.");
      path_valid_ = false;
      return;
    }

    latest_path_ = *msg;
    path_valid_ = true;
    last_path_receive_time_ = steady_clock_.now();

    RCLCPP_INFO_THROTTLE(
      get_logger(), steady_clock_, 3000,
      "Received local raceline path with %zu poses in frame '%s'.",
      latest_path_.poses.size(), latest_path_.header.frame_id.c_str());
  }

  void ruleSpeedIndexCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    latest_rule_speed_index_ = std::clamp(msg->data, 0.0, 1.0);
    rule_speed_index_valid_ = true;
    last_rule_speed_index_receive_time_ = steady_clock_.now();
  }

  void rlSpeedResidualCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    latest_rl_speed_residual_mps_ = msg->data;
    rl_speed_residual_valid_ = true;
    last_rl_speed_residual_receive_time_ = steady_clock_.now();
  }

  bool pathFresh()
  {
    if (!path_valid_) {
      return false;
    }
    return (steady_clock_.now() - last_path_receive_time_).seconds() <= path_timeout_sec_;
  }

  bool ruleSpeedIndexFresh()
  {
    if (!rule_speed_index_valid_) {
      return false;
    }
    return (steady_clock_.now() - last_rule_speed_index_receive_time_).seconds() <= rule_speed_index_timeout_sec_;
  }

  bool rlResidualFresh()
  {
    if (!rl_speed_residual_valid_) {
      return false;
    }
    return (steady_clock_.now() - last_rl_speed_residual_receive_time_).seconds() <= rl_residual_timeout_sec_;
  }

  double speedFromIndex(double speed_index) const
  {
    const double idx = std::clamp(speed_index, 0.0, 1.0);
    return speed_min_ + idx * (speed_max_ - speed_min_);
  }

  double rateLimitSpeed(double requested_speed) const
  {
    requested_speed = std::clamp(requested_speed, speed_min_, speed_max_);
    if (max_speed_delta_per_step_mps_ <= 0.0) {
      return requested_speed;
    }
    const double lower = last_commanded_speed_ - max_speed_delta_per_step_mps_;
    const double upper = last_commanded_speed_ + max_speed_delta_per_step_mps_;
    return std::clamp(requested_speed, lower, upper);
  }

  double computeSpeedDependentSteeringLookahead(double speed) const
  {
    const double lookahead = steering_min_lookahead_m_ + steering_lookahead_speed_gain_ * std::max(0.0, speed);
    return std::clamp(lookahead, steering_min_lookahead_m_, steering_max_lookahead_m_);
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
        get_logger(), steady_clock_, 2000,
        "TF lookup failed (%s -> %s): %s",
        global_frame_.c_str(), robot_frame_.c_str(), ex.what());
      return false;
    }
  }

  bool findLookaheadPointInBaseFrame(
    const tf2::Transform & tf_map_to_base,
    const double active_lookahead_distance,
    geometry_msgs::msg::Point & lookahead_point_base)
  {
    const tf2::Transform tf_base_to_map = tf_map_to_base.inverse();

    bool found = false;
    double best_dist_error = std::numeric_limits<double>::max();

    for (const auto & pose_stamped : latest_path_.poses) {
      tf2::Vector3 p_map(
        pose_stamped.pose.position.x,
        pose_stamped.pose.position.y,
        0.0);

      const tf2::Vector3 p_base = tf_base_to_map * p_map;
      const double x = p_base.x();
      const double y = p_base.y();

      if (x < min_forward_point_x_) {
        continue;
      }

      const double dist = std::hypot(x, y);
      if (dist < active_lookahead_distance) {
        continue;
      }

      const double dist_error = std::abs(dist - active_lookahead_distance);
      if (dist_error < best_dist_error) {
        best_dist_error = dist_error;
        lookahead_point_base.x = x;
        lookahead_point_base.y = y;
        lookahead_point_base.z = 0.0;
        found = true;
      }
    }

    return found;
  }

  void publishDrive(double speed, double steering_angle)
  {
    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.header.stamp = now();
    msg.drive.speed = speed;
    msg.drive.steering_angle = steering_angle;
    drive_pub_->publish(msg);
  }

  void publishStop()
  {
    publishDrive(0.0, 0.0);
    last_commanded_speed_ = 0.0;
  }

  void publishMarkers(
    const geometry_msgs::msg::Point & lookahead_point,
    double steering_angle,
    bool valid)
  {
    if (!publish_markers_) {
      return;
    }

    const auto stamp = now();

    visualization_msgs::msg::Marker lookahead_marker;
    lookahead_marker.header.frame_id = marker_frame_;
    lookahead_marker.header.stamp = stamp;
    lookahead_marker.ns = "path_following_v2";
    lookahead_marker.id = 0;
    lookahead_marker.type = visualization_msgs::msg::Marker::SPHERE;
    lookahead_marker.action = valid ?
      visualization_msgs::msg::Marker::ADD :
      visualization_msgs::msg::Marker::DELETE;
    lookahead_marker.pose.position = lookahead_point;
    lookahead_marker.pose.orientation.w = 1.0;
    lookahead_marker.scale.x = 0.12;
    lookahead_marker.scale.y = 0.12;
    lookahead_marker.scale.z = 0.12;
    lookahead_marker.color.a = 1.0;
    lookahead_marker.color.r = 0.0;
    lookahead_marker.color.g = 1.0;
    lookahead_marker.color.b = 0.0;
    lookahead_marker_pub_->publish(lookahead_marker);

    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.header.frame_id = marker_frame_;
    arrow_marker.header.stamp = stamp;
    arrow_marker.ns = "path_following_v2";
    arrow_marker.id = 1;
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.action = valid ?
      visualization_msgs::msg::Marker::ADD :
      visualization_msgs::msg::Marker::DELETE;

    geometry_msgs::msg::Point start;
    start.x = 0.0;
    start.y = 0.0;
    start.z = 0.0;

    geometry_msgs::msg::Point end;
    end.x = std::cos(steering_angle);
    end.y = std::sin(steering_angle);
    end.z = 0.0;

    arrow_marker.points.push_back(start);
    arrow_marker.points.push_back(end);
    arrow_marker.scale.x = 0.05;
    arrow_marker.scale.y = 0.10;
    arrow_marker.scale.z = 0.10;
    arrow_marker.color.a = 1.0;
    arrow_marker.color.r = 0.0;
    arrow_marker.color.g = 0.0;
    arrow_marker.color.b = 1.0;
    steering_marker_pub_->publish(arrow_marker);
  }

  void controlLoop()
  {
    if (!pathFresh()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 2000,
        "Local path missing or stale. Holding stop.");
      if (stop_if_no_path_) {
        publishStop();
      }
      geometry_msgs::msg::Point p;
      publishMarkers(p, 0.0, false);
      return;
    }

    if (!ruleSpeedIndexFresh()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 2000,
        "Rule speed index missing or stale. Holding stop.");
      publishStop();
      geometry_msgs::msg::Point p;
      publishMarkers(p, 0.0, false);
      return;
    }

    tf2::Transform tf_map_to_base;
    if (!lookupRobotPose(tf_map_to_base)) {
      if (stop_if_no_path_) {
        publishStop();
      }
      geometry_msgs::msg::Point p;
      publishMarkers(p, 0.0, false);
      return;
    }

    const double rule_speed_mps = speedFromIndex(latest_rule_speed_index_);
    const bool residual_is_fresh = rlResidualFresh();
    const bool use_rl_residual = (speed_mode_ == 1) && residual_is_fresh;

    if (speed_mode_ == 1 && residual_is_fresh && !rl_residual_was_fresh_) {
      RCLCPP_INFO(
        get_logger(),
        "RL speed residual is fresh; using rule speed + RL residual.");
    }
    if (speed_mode_ == 1 && !residual_is_fresh && rl_residual_was_fresh_) {
      RCLCPP_WARN(
        get_logger(),
        "RL speed residual is stale/unhealthy; falling back to rule-based speed only.");
    }
    rl_residual_was_fresh_ = residual_is_fresh;

    const double residual_mps = use_rl_residual ? latest_rl_speed_residual_mps_ : 0.0;
    const double requested_speed = std::clamp(rule_speed_mps + residual_mps, speed_min_, speed_max_);
    const double commanded_speed = rateLimitSpeed(requested_speed);

    const double speed_for_lookahead =
      use_speed_dependent_steering_lookahead_ ? commanded_speed : last_commanded_speed_;
    const double active_lookahead_distance =
      use_speed_dependent_steering_lookahead_
        ? computeSpeedDependentSteeringLookahead(speed_for_lookahead)
        : fixed_steering_lookahead_m_;

    geometry_msgs::msg::Point lookahead_point_base;
    const bool found = findLookaheadPointInBaseFrame(
      tf_map_to_base,
      active_lookahead_distance,
      lookahead_point_base);

    if (!found) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 2000,
        "No valid forward lookahead point found.");
      if (stop_if_no_path_) {
        publishStop();
      }
      geometry_msgs::msg::Point p;
      publishMarkers(p, 0.0, false);
      return;
    }

    const double x = lookahead_point_base.x;
    const double y = lookahead_point_base.y;
    const double L = std::hypot(x, y);

    if (L < 1e-6) {
      if (stop_if_no_path_) {
        publishStop();
      }
      publishMarkers(lookahead_point_base, 0.0, false);
      return;
    }

    const double curvature = 2.0 * y / (L * L);
    double steering_angle = std::atan(wheelbase_ * curvature);

    const double steering_max_rad = steering_max_deg_ * M_PI / 180.0;
    steering_angle = std::clamp(steering_angle, -steering_max_rad, steering_max_rad);

    last_commanded_speed_ = commanded_speed;
    publishDrive(commanded_speed, steering_angle);
    publishMarkers(lookahead_point_base, steering_angle, true);

    RCLCPP_INFO_THROTTLE(
      get_logger(), steady_clock_, 1000,
      "cmd: speed=%.2f m/s, rule=%.2f, residual=%.2f (%s), mode=%d, steer=%.3f rad, active_Ld=%.2f, lookahead=(%.2f, %.2f)",
      commanded_speed,
      rule_speed_mps,
      residual_mps,
      use_rl_residual ? "fresh" : "rule_only",
      speed_mode_,
      steering_angle,
      active_lookahead_distance,
      x,
      y);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollowingV2Node>());
  rclcpp::shutdown();
  return 0;
}
