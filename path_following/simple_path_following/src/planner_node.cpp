#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode() : Node("planner_node") {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::publish_path, this));
        RCLCPP_INFO(this->get_logger(), "Left-arc planner node started.");
    }

private:
    void publish_path() {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "laser";  // ego frame
        path_msg.header.stamp = this->now();

        for (double x = 0.5; x <= 10.0; x += 0.5) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "laser";
            pose.header.stamp = this->now();

            pose.pose.position.x = x;
            pose.pose.position.y = 0.2 * std::sin(x / 3.0);  // gentle left arc
            pose.pose.orientation.w = 1.0;

            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
