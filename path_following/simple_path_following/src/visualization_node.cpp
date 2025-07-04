#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"

class VisualizationNode : public rclcpp::Node {
public:
    VisualizationNode() : Node("trajectory_visualizer") {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("trajectory_marker", 10);
        traj_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "trajectory", 10, std::bind(&VisualizationNode::path_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Visualization node started.");
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "ego_racecar/laser";  
        marker.header.stamp = this->now();
        marker.ns = "trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.05;  // line width
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        // Start at (0, 0)
        geometry_msgs::msg::Point start;
        start.x = 0.0;
        start.y = 0.0;
        start.z = 0.0;
        marker.points.push_back(start);

        // Append rest of the trajectory
        for (const auto& pose : msg->poses) {
            geometry_msgs::msg::Point p;
            p.x = pose.pose.position.x;
            p.y = pose.pose.position.y;
            p.z = 0.0;
            marker.points.push_back(p);
        }

        marker_pub_->publish(marker);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr traj_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualizationNode>());
    rclcpp::shutdown();
    return 0;
}
