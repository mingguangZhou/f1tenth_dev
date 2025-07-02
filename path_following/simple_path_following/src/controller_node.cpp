#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode() : Node("controller_node") {
        traj_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "trajectory", 10, std::bind(&ControllerNode::path_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControllerNode::control_loop, this));

        lookahead_distance_ = 0.5;  // meters
        velocity_ = 0.2;  // constant speed

        RCLCPP_INFO(this->get_logger(), "Controller node started.");
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        trajectory_ = *msg;
    }

    void control_loop() {
        if (trajectory_.poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "No trajectory received.");
            return;
        }

        geometry_msgs::msg::PoseStamped lookahead;
        bool found = false;

        // Find first point >= lookahead distance
        for (const auto& pose : trajectory_.poses) {
            double dx = pose.pose.position.x;
            double dy = pose.pose.position.y;
            double dist = std::hypot(dx, dy);
            if (dist >= lookahead_distance_) {
                lookahead = pose;
                found = true;
                break;
            }
        }

        if (!found) {
            RCLCPP_WARN(this->get_logger(), "No valid lookahead point found.");
            return;
        }

        // Compute steering angle using pure pursuit formula
        double x = lookahead.pose.position.x;
        double y = lookahead.pose.position.y;
        double L = std::hypot(x, y);  // length to lookahead point

        double steering_angle = std::atan2(2.0 * y, L * L);

        // Output drive command
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->now();
        drive_msg.drive.speed = velocity_;
        drive_msg.drive.steering_angle = steering_angle;

        drive_pub_->publish(drive_msg);

        RCLCPP_INFO(this->get_logger(), "Lookahead point: (%.2f, %.2f), Steer: %.2f", x, y, steering_angle);
    }

    nav_msgs::msg::Path trajectory_;
    double lookahead_distance_;
    double velocity_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr traj_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
