#include <algorithm>
#include <cmath>
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

#include "tf2/LinearMath/Quaternion.h"
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
    declare_parameter<std::string>("path_topic", "/centerline_path");
    declare_parameter<std::string>("drive_topic", "/drive");
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<std::string>("robot_frame", "ego_racecar/base_link");

    declare_parameter<double>("control_rate_hz", 20.0);
    declare_parameter<double>("path_timeout_sec", 1.0);
    declare_parameter<double>("tf_timeout_sec", 0.1);

    declare_parameter<double>("lookahead_distance", 0.60);
    declare_parameter<double>("wheelbase", 0.33);
    declare_parameter<double>("steering_max_deg", 20.6);

    declare_parameter<double>("velocity_max", 2.0);
    declare_parameter<double>("velocity_min", 0.8);
    declare_parameter<double>("speed_steering_exponent", 1.0);

    declare_parameter<double>("min_forward_point_x", 0.05);
    declare_parameter<bool>("stop_if_no_path", true);

    declare_parameter<bool>("publish_markers", true);
    declare_parameter<std::string>("marker_frame", "ego_racecar/base_link");
    declare_parameter<std::string>("lookahead_marker_topic", "/path_following_v2/lookahead_marker");
    declare_parameter<std::string>("steering_marker_topic", "/path_following_v2/steering_marker");

    path_topic_ = get_parameter("path_topic").as_string();
    drive_topic_ = get_parameter("drive_topic").as_string();
    global_frame_ = get_parameter("global_frame").as_string();
    robot_frame_ = get_parameter("robot_frame").as_string();

    control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    path_timeout_sec_ = get_parameter("path_timeout_sec").as_double();
    tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();

    lookahead_distance_ = get_parameter("lookahead_distance").as_double();
    wheelbase_ = get_parameter("wheelbase").as_double();
    steering_max_deg_ = get_parameter("steering_max_deg").as_double();

    velocity_max_ = get_parameter("velocity_max").as_double();
    velocity_min_ = get_parameter("velocity_min").as_double();
    speed_steering_exponent_ = get_parameter("speed_steering_exponent").as_double();

    min_forward_point_x_ = get_parameter("min_forward_point_x").as_double();
    stop_if_no_path_ = get_parameter("stop_if_no_path").as_bool();

    publish_markers_ = get_parameter("publish_markers").as_bool();
    marker_frame_ = get_parameter("marker_frame").as_string();
    lookahead_marker_topic_ = get_parameter("lookahead_marker_topic").as_string();
    steering_marker_topic_ = get_parameter("steering_marker_topic").as_string();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&PathFollowingV2Node::pathCallback, this, std::placeholders::_1));

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);

    if (publish_markers_) {
      lookahead_marker_pub_ =
        create_publisher<visualization_msgs::msg::Marker>(lookahead_marker_topic_, 10);
      steering_marker_pub_ =
        create_publisher<visualization_msgs::msg::Marker>(steering_marker_topic_, 10);
    }

    const auto period_ms =
      std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, control_rate_hz_)));
    timer_ = create_wall_timer(period_ms, std::bind(&PathFollowingV2Node::controlLoop, this));

    RCLCPP_INFO(get_logger(), "path_following_v2 started");
    RCLCPP_INFO(get_logger(), "  path_topic: %s", path_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  drive_topic: %s", drive_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  global_frame: %s", global_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  robot_frame: %s", robot_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  lookahead_distance: %.3f", lookahead_distance_);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr steering_marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  nav_msgs::msg::Path latest_path_;
  bool path_valid_{false};
  rclcpp::Time last_path_receive_time_{0, 0, RCL_ROS_TIME};

  std::string path_topic_;
  std::string drive_topic_;
  std::string global_frame_;
  std::string robot_frame_;
  std::string marker_frame_;
  std::string lookahead_marker_topic_;
  std::string steering_marker_topic_;

  double control_rate_hz_{20.0};
  double path_timeout_sec_{1.0};
  double tf_timeout_sec_{0.1};
  double lookahead_distance_{0.60};
  double wheelbase_{0.33};
  double steering_max_deg_{20.6};
  double velocity_max_{2.0};
  double velocity_min_{0.8};
  double speed_steering_exponent_{1.0};
  double min_forward_point_x_{0.05};

  bool stop_if_no_path_{true};
  bool publish_markers_{true};

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty centerline path.");
      path_valid_ = false;
      return;
    }

    latest_path_ = *msg;
    path_valid_ = true;
    last_path_receive_time_ = now();

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Received centerline path with %zu poses in frame '%s'.",
      latest_path_.poses.size(), latest_path_.header.frame_id.c_str());
  }

  bool pathFresh() const
  {
    if (!path_valid_) {
      return false;
    }
    return (now() - last_path_receive_time_).seconds() <= path_timeout_sec_;
  }

  double computeSpeed(double steering_angle_rad) const
  {
    const double steering_max_rad = steering_max_deg_ * M_PI / 180.0;
    const double ratio = std::clamp(std::abs(steering_angle_rad) / steering_max_rad, 0.0, 1.0);
    const double shaped = std::pow(ratio, speed_steering_exponent_);
    return velocity_max_ - shaped * (velocity_max_ - velocity_min_);
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
        global_frame_.c_str(), robot_frame_.c_str(), ex.what());
      return false;
    }
  }

  bool findLookaheadPointInBaseFrame(
    const tf2::Transform & tf_map_to_base,
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
      if (dist < lookahead_distance_) {
        continue;
      }

      const double dist_error = std::abs(dist - lookahead_distance_);
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
        get_logger(), *get_clock(), 2000,
        "Path missing or stale. Holding stop.");
      if (stop_if_no_path_) {
        publishStop();
      }
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

    geometry_msgs::msg::Point lookahead_point_base;
    const bool found = findLookaheadPointInBaseFrame(tf_map_to_base, lookahead_point_base);

    if (!found) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
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

    const double speed = computeSpeed(steering_angle);

    publishDrive(speed, steering_angle);
    publishMarkers(lookahead_point_base, steering_angle, true);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "cmd: speed=%.2f m/s, steer=%.3f rad, lookahead=(%.2f, %.2f)",
      speed, steering_angle, x, y);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollowingV2Node>());
  rclcpp::shutdown();
  return 0;
}