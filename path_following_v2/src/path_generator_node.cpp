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
    declare_parameter<std::string>("speed_index_topic", "/path_following_v2/speed_index");
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<std::string>("robot_frame", "ego_racecar/base_link");

    declare_parameter<double>("publish_rate_hz", 20.0);
    declare_parameter<double>("tf_timeout_sec", 0.05);
    declare_parameter<int>("local_path_horizon_points", 80);

    declare_parameter<double>("curvature_speed_gain", 2.0);
    declare_parameter<int>("curvature_lookahead_points", 10);
    declare_parameter<double>("min_speed_index", 0.0);
    declare_parameter<double>("max_speed_index", 1.0);
    declare_parameter<std::string>("speed_index_mode", "rule_based");

    raceline_waypoints_topic_ = get_parameter("raceline_waypoints_topic").as_string();
    local_path_topic_ = get_parameter("local_path_topic").as_string();
    speed_index_topic_ = get_parameter("speed_index_topic").as_string();
    global_frame_ = get_parameter("global_frame").as_string();
    robot_frame_ = get_parameter("robot_frame").as_string();

    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();
    local_path_horizon_points_ = get_parameter("local_path_horizon_points").as_int();

    curvature_speed_gain_ = get_parameter("curvature_speed_gain").as_double();
    curvature_lookahead_points_ = get_parameter("curvature_lookahead_points").as_int();
    min_speed_index_ = get_parameter("min_speed_index").as_double();
    max_speed_index_ = get_parameter("max_speed_index").as_double();
    speed_index_mode_ = get_parameter("speed_index_mode").as_string();

    if (speed_index_mode_ != "rule_based" && speed_index_mode_ != "external") {
      RCLCPP_WARN(
        get_logger(),
        "Unsupported speed_index_mode='%s'; falling back to 'rule_based'.",
        speed_index_mode_.c_str());
      speed_index_mode_ = "rule_based";
    }

    if (local_path_horizon_points_ < 2) {
      RCLCPP_WARN(get_logger(), "local_path_horizon_points must be >= 2; forcing to 2.");
      local_path_horizon_points_ = 2;
    }

    min_speed_index_ = std::clamp(min_speed_index_, 0.0, 1.0);
    max_speed_index_ = std::clamp(max_speed_index_, 0.0, 1.0);
    if (min_speed_index_ > max_speed_index_) {
      RCLCPP_WARN(get_logger(), "min_speed_index > max_speed_index; swapping them.");
      std::swap(min_speed_index_, max_speed_index_);
    }

    raceline_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      raceline_waypoints_topic_,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&PathGeneratorNode::racelineCallback, this, std::placeholders::_1));

    local_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      local_path_topic_, rclcpp::QoS(1).reliable().transient_local());

    if (speed_index_mode_ == "rule_based") {
      speed_index_pub_ = create_publisher<std_msgs::msg::Float64>(speed_index_topic_, 10);
    }

    const auto period_ms = std::chrono::milliseconds(
      static_cast<int>(1000.0 / std::max(1.0, publish_rate_hz_)));
    timer_ = create_wall_timer(period_ms, std::bind(&PathGeneratorNode::publishLoop, this));

    RCLCPP_INFO(get_logger(), "path_generator started as clean raceline local-path generator");
    RCLCPP_INFO(get_logger(), "  raceline_waypoints_topic: %s", raceline_waypoints_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  local_path_topic: %s", local_path_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  speed_index_mode: %s", speed_index_mode_.c_str());
    if (speed_index_mode_ == "rule_based") {
      RCLCPP_INFO(get_logger(), "  speed_index_topic: %s", speed_index_topic_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "  speed_index_topic disabled; expecting external publisher.");
    }
    RCLCPP_INFO(get_logger(), "  local_path_horizon_points: %d", local_path_horizon_points_);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr raceline_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_index_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<RacelineWaypoint> raceline_;
  bool raceline_valid_{false};

  std::string raceline_waypoints_topic_;
  std::string local_path_topic_;
  std::string speed_index_topic_;
  std::string global_frame_;
  std::string robot_frame_;
  std::string speed_index_mode_{"rule_based"};

  double publish_rate_hz_{20.0};
  double tf_timeout_sec_{0.05};
  double curvature_speed_gain_{2.0};
  double min_speed_index_{0.0};
  double max_speed_index_{1.0};

  int local_path_horizon_points_{80};
  int curvature_lookahead_points_{10};

  void racelineCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    constexpr std::size_t fields = 6;

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

    RCLCPP_INFO_THROTTLE(
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

  double computeSpeedIndex(int nearest_idx) const
  {
    if (raceline_.empty()) {
      return 0.0;
    }

    const int n = static_cast<int>(raceline_.size());
    const int idx = ((nearest_idx + curvature_lookahead_points_) % n + n) % n;
    const double curvature_abs = std::max(0.0, raceline_[idx].curvature_abs);

    // Normalized speed request in [0, 1].
    // 1.0 means follower speed_max; lower values slow down in high curvature.
    const double raw_index = 1.0 / (1.0 + curvature_speed_gain_ * curvature_abs);
    return std::clamp(raw_index, min_speed_index_, max_speed_index_);
  }

  void publishLoop()
  {
    if (!raceline_valid_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No valid raceline waypoints yet.");
      return;
    }

    tf2::Transform tf_map_to_base;
    const bool pose_ok = lookupRobotPose(tf_map_to_base);
    const int nearest_idx = pose_ok ? findNearestRacelineIndex(tf_map_to_base) : 0;

    const auto local_path = buildLocalRacelinePath(nearest_idx);
    local_path_pub_->publish(local_path);

    if (speed_index_mode_ == "rule_based") {
      std_msgs::msg::Float64 speed_index_msg;
      speed_index_msg.data = computeSpeedIndex(nearest_idx);
      speed_index_pub_->publish(speed_index_msg);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "local raceline: nearest=%d, horizon=%d, speed_index=%.3f, curv_abs=%.3f",
        nearest_idx,
        local_path_horizon_points_,
        speed_index_msg.data,
        raceline_[nearest_idx].curvature_abs);
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "local raceline: nearest=%d, horizon=%d, speed_index_mode=external, curv_abs=%.3f",
        nearest_idx,
        local_path_horizon_points_,
        raceline_[nearest_idx].curvature_abs);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
