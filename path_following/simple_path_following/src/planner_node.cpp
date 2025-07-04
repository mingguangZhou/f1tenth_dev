#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode() : Node("planner_node") {
        traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&PlannerNode::publish_trajectory, this));

        RCLCPP_INFO(this->get_logger(), "Planner node started.");
        start_time_ = this->now();
    }

private:
    rclcpp::Time start_time_;

    void publish_trajectory() {
        rclcpp::Duration elapsed = this->now() - start_time_;
        double time_shift = elapsed.seconds();  // use seconds as a horizontal shift

        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "ego_racecar/laser";
        path_msg.header.stamp = this->now();

        // Generate moving sine-wave trajectory
        for (double x = 0.5; x <= 5.0; x += 0.1) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "ego_racecar/laser";
            pose.header.stamp = this->now();

            pose.pose.position.x = x;
            pose.pose.position.y = 0.3 * std::sin(x * 1.0 + time_shift);  // phase shift over time
            pose.pose.orientation.w = 1.0;

            path_msg.poses.push_back(pose);
        }

        traj_pub_->publish(path_msg);
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
