#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>

class SafetyBubbleVisualizer : public rclcpp::Node {
public:
  SafetyBubbleVisualizer() : Node("safety_bubble_visualizer") {
    using std::placeholders::_1;

    center_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "bubble_center", 10,
      std::bind(&SafetyBubbleVisualizer::center_callback, this, _1));

    radius_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "bubble_radius", 10,
      std::bind(&SafetyBubbleVisualizer::radius_callback, this, _1));

    angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "desired_angle", 10,
      std::bind(&SafetyBubbleVisualizer::angle_callback, this, _1));

    marker_pub_sphere_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "bubble_sphere", 10);
    marker_pub_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "desired_arrow", 10);

    RCLCPP_INFO(this->get_logger(), "SafetyBubbleVisualizer node started.");
  }

private:
  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr center_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr radius_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_sub_;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_sphere_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_arrow_;

  // Internal state
  geometry_msgs::msg::Point center_;
  double radius_ = 0.0;
  double desired_angle_ = 0.0;

  void center_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    center_ = *msg;
    publish_bubble_marker();
  }

  void radius_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    radius_ = msg->data;
    publish_bubble_marker();
  }

  void angle_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    desired_angle_ = msg->data;
    publish_arrow_marker();
  }

  void publish_bubble_marker() {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego_racecar/laser";
    marker.header.stamp = this->now();
    marker.ns = "safety_bubble";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position = center_;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = radius_ * 2;
    marker.scale.y = radius_ * 2;
    marker.scale.z = 0.1;

    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    marker_pub_sphere_->publish(marker);
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    //                      "Published bubble marker.");
  }

  void publish_arrow_marker() {
    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.header.frame_id = "ego_racecar/laser";
    arrow_marker.header.stamp = this->now();
    arrow_marker.ns = "safety_bubble";
    arrow_marker.id = 1;
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.action = visualization_msgs::msg::Marker::ADD;

    // Tail at origin
    geometry_msgs::msg::Point tail, head;
    tail.x = 0.0;
    tail.y = 0.0;
    tail.z = 0.0;

    double arrow_length = 1.0;
    head.x = arrow_length * std::cos(desired_angle_);
    head.y = arrow_length * std::sin(desired_angle_);
    head.z = 0.0;

    arrow_marker.points.push_back(tail);
    arrow_marker.points.push_back(head);

    // Arrow size
    arrow_marker.scale.x = 0.05;  // shaft diameter
    arrow_marker.scale.y = 0.1;   // head diameter
    arrow_marker.scale.z = 0.1;   // head length

    arrow_marker.color.a = 1.0;
    arrow_marker.color.r = 0.0;
    arrow_marker.color.g = 1.0;
    arrow_marker.color.b = 0.0;

    marker_pub_arrow_->publish(arrow_marker);
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    //                     "Published arrow marker.");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyBubbleVisualizer>());
  rclcpp::shutdown();
  return 0;
}
