#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker.hpp"

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

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "bubble_marker", 10);
  }

private:
  void center_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    center_ = *msg;
    publish_marker();
  }

  void radius_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    radius_ = msg->data;
    publish_marker();
  }

  void publish_marker() {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego_racecar/base_link";
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
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker_pub_->publish(marker);
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr center_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr radius_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  geometry_msgs::msg::Point center_;
  double radius_ = 0.0;
};
  
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyBubbleVisualizer>());
  rclcpp::shutdown();
  return 0;
}
