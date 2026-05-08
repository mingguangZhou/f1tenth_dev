#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

struct CenterlineWaypoint
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
    declare_parameter<std::string>("waypoints_topic", "/centerline_waypoints");
    declare_parameter<std::string>("generated_path_topic", "/path_following_v2/generated_path");
    declare_parameter<std::string>("target_speed_topic", "/path_following_v2/target_speed");
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<std::string>("robot_frame", "ego_racecar/base_link");

    declare_parameter<double>("publish_rate_hz", 20.0);
    declare_parameter<double>("tf_timeout_sec", 0.05);

    declare_parameter<std::string>("lateral_offset_mode", "curvature");  // zero, constant, curvature
    declare_parameter<double>("constant_lateral_offset", 0.0);
    declare_parameter<double>("max_lateral_offset", 0.35);
    declare_parameter<double>("curvature_offset_gain", 0.18);

    declare_parameter<double>("velocity_max", 2.0);
    declare_parameter<double>("velocity_min", 0.8);
    declare_parameter<double>("curvature_speed_gain", 2.0);
    declare_parameter<int>("curvature_lookahead_points", 15);
    declare_parameter<int>("local_path_horizon_points", 80);

    waypoints_topic_ = get_parameter("waypoints_topic").as_string();
    generated_path_topic_ = get_parameter("generated_path_topic").as_string();
    target_speed_topic_ = get_parameter("target_speed_topic").as_string();
    global_frame_ = get_parameter("global_frame").as_string();
    robot_frame_ = get_parameter("robot_frame").as_string();

    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();

    lateral_offset_mode_ = get_parameter("lateral_offset_mode").as_string();
    constant_lateral_offset_ = get_parameter("constant_lateral_offset").as_double();
    max_lateral_offset_ = get_parameter("max_lateral_offset").as_double();
    curvature_offset_gain_ = get_parameter("curvature_offset_gain").as_double();

    velocity_max_ = get_parameter("velocity_max").as_double();
    velocity_min_ = get_parameter("velocity_min").as_double();
    curvature_speed_gain_ = get_parameter("curvature_speed_gain").as_double();
    curvature_lookahead_points_ = get_parameter("curvature_lookahead_points").as_int();
    local_path_horizon_points_ = get_parameter("local_path_horizon_points").as_int();
    if (local_path_horizon_points_ < 2) {
      RCLCPP_WARN(get_logger(), "local_path_horizon_points must be >= 2; forcing to 2.");
      local_path_horizon_points_ = 2;
    }

    if (lateral_offset_mode_ != "zero" && lateral_offset_mode_ != "constant" && lateral_offset_mode_ != "curvature") {
      RCLCPP_WARN(get_logger(), "Unknown lateral_offset_mode '%s'; falling back to zero.", lateral_offset_mode_.c_str());
      lateral_offset_mode_ = "zero";
    }

    waypoints_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      waypoints_topic_,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&PathGeneratorNode::waypointsCallback, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      generated_path_topic_, rclcpp::QoS(1).reliable().transient_local());
    speed_pub_ = create_publisher<std_msgs::msg::Float64>(target_speed_topic_, 10);

    const auto period_ms = std::chrono::milliseconds(
      static_cast<int>(1000.0 / std::max(1.0, publish_rate_hz_)));
    timer_ = create_wall_timer(period_ms, std::bind(&PathGeneratorNode::publishLoop, this));

    RCLCPP_INFO(get_logger(), "path_generator started");
    RCLCPP_INFO(get_logger(), "  waypoints_topic: %s", waypoints_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  generated_path_topic: %s", generated_path_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  target_speed_topic: %s", target_speed_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  lateral_offset_mode: %s", lateral_offset_mode_.c_str());
    RCLCPP_INFO(get_logger(), "  local_path_horizon_points: %d", local_path_horizon_points_);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr waypoints_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<CenterlineWaypoint> waypoints_;
  bool waypoints_valid_{false};

  std::string waypoints_topic_;
  std::string generated_path_topic_;
  std::string target_speed_topic_;
  std::string global_frame_;
  std::string robot_frame_;
  std::string lateral_offset_mode_;

  double publish_rate_hz_{20.0};
  double tf_timeout_sec_{0.05};
  double constant_lateral_offset_{0.0};
  double max_lateral_offset_{0.35};
  double curvature_offset_gain_{0.18};
  double velocity_max_{2.0};
  double velocity_min_{0.8};
  double curvature_speed_gain_{2.0};
  int curvature_lookahead_points_{15};
  int local_path_horizon_points_{80};

  void waypointsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    constexpr std::size_t fields = 6;
    if (msg->data.empty() || msg->data.size() % fields != 0) {
      RCLCPP_WARN(get_logger(), "Received invalid /centerline_waypoints data size: %zu", msg->data.size());
      waypoints_valid_ = false;
      return;
    }

    std::vector<CenterlineWaypoint> parsed;
    parsed.reserve(msg->data.size() / fields);

    for (std::size_t i = 0; i + fields - 1 < msg->data.size(); i += fields) {
      CenterlineWaypoint wp;
      wp.index = static_cast<int>(std::lround(msg->data[i + 0]));
      wp.x = msg->data[i + 1];
      wp.y = msg->data[i + 2];
      wp.yaw = msg->data[i + 3];
      wp.curvature = msg->data[i + 4];
      wp.curvature_abs = msg->data[i + 5];
      parsed.push_back(wp);
    }

    waypoints_ = std::move(parsed);
    waypoints_valid_ = !waypoints_.empty();

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Received %zu centerline waypoints.", waypoints_.size());
  }

  bool lookupRobotPose(tf2::Transform & tf_map_to_base)
  {
    try {
      const auto tf_msg = tf_buffer_.lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_sec_));
      tf2::fromMsg(tf_msg.transform, tf_map_to_base);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF lookup failed (%s -> %s): %s", global_frame_.c_str(), robot_frame_.c_str(), ex.what());
      return false;
    }
  }

  int findNearestWaypointIndex(const tf2::Transform & tf_map_to_base) const
  {
    if (waypoints_.empty()) {
      return 0;
    }

    const double rx = tf_map_to_base.getOrigin().x();
    const double ry = tf_map_to_base.getOrigin().y();

    int best_idx = 0;
    double best_dist2 = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < waypoints_.size(); ++i) {
      const double dx = waypoints_[i].x - rx;
      const double dy = waypoints_[i].y - ry;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_dist2) {
        best_dist2 = d2;
        best_idx = static_cast<int>(i);
      }
    }
    return best_idx;
  }

  double computeOffset(const CenterlineWaypoint & wp) const
  {
    if (lateral_offset_mode_ == "zero") {
      return 0.0;
    }
    if (lateral_offset_mode_ == "constant") {
      return std::clamp(constant_lateral_offset_, -max_lateral_offset_, max_lateral_offset_);
    }

    // Dummy RL replacement rule: shift toward the inside of a curve.
    // Positive curvature with this centerline convention means positive normal-side offset.
    const double raw = curvature_offset_gain_ * wp.curvature;
    return std::clamp(raw, -max_lateral_offset_, max_lateral_offset_);
  }

  double computeTargetSpeed(int nearest_idx) const
  {
    if (waypoints_.empty()) {
      return 0.0;
    }

    const int n = static_cast<int>(waypoints_.size());
    const int idx = ((nearest_idx + curvature_lookahead_points_) % n + n) % n;
    const double curvature_abs = waypoints_[idx].curvature_abs;

    const double raw_speed = velocity_max_ / (1.0 + curvature_speed_gain_ * curvature_abs);
    return std::clamp(raw_speed, velocity_min_, velocity_max_);
  }

  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) const
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(q);
  }

  nav_msgs::msg::Path buildOffsetPath(int nearest_idx) const
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = global_frame_;

    if (waypoints_.empty()) {
      return path;
    }

    const int n = static_cast<int>(waypoints_.size());
    const int horizon = std::min(local_path_horizon_points_, n);
    path.poses.reserve(static_cast<std::size_t>(horizon));

    // Publish only a local forward segment of the offset path.
    // The segment starts from the waypoint nearest to the current car pose and
    // wraps around at the end because the centerline is a closed loop.
    for (int step = 0; step < horizon; ++step) {
      const int idx = ((nearest_idx + step) % n + n) % n;
      const auto & wp = waypoints_[idx];
      const double d = computeOffset(wp);

      // Left normal of the waypoint tangent: n = [-sin(yaw), cos(yaw)].
      const double nx = -std::sin(wp.yaw);
      const double ny = std::cos(wp.yaw);

      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = wp.x + d * nx;
      pose.pose.position.y = wp.y + d * ny;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = yawToQuaternion(wp.yaw);
      path.poses.push_back(pose);
    }

    return path;
  }

  void publishLoop()
  {
    if (!waypoints_valid_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No valid centerline waypoints yet.");
      return;
    }

    tf2::Transform tf_map_to_base;
    const bool pose_ok = lookupRobotPose(tf_map_to_base);
    const int nearest_idx = pose_ok ? findNearestWaypointIndex(tf_map_to_base) : 0;

    const auto path = buildOffsetPath(nearest_idx);
    path_pub_->publish(path);

    std_msgs::msg::Float64 speed_msg;
    speed_msg.data = computeTargetSpeed(nearest_idx);
    speed_pub_->publish(speed_msg);

    const double current_offset = computeOffset(waypoints_[nearest_idx]);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "dummy local path: nearest=%d, horizon=%d, speed=%.2f, offset=%.3f, curv_abs=%.3f",
      nearest_idx, local_path_horizon_points_, speed_msg.data, current_offset, waypoints_[nearest_idx].curvature_abs);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}