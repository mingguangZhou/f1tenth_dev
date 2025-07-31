#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <cmath>

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode() : Node("controller_node") {
        // Parameters
        declare_parameter<double>("lookahead_distance", 0.6);
        declare_parameter<double>("velocity", 1.0);
        
        traj_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "trajectory", 10, std::bind(&ControllerNode::path_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        lookahead_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lookahead_marker", 10);
        steering_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("steering_arrow_marker", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControllerNode::control_loop, this));

        lookahead_distance_ = get_parameter("lookahead_distance").as_double();
        velocity_ = get_parameter("velocity").as_double();


        RCLCPP_INFO(this->get_logger(), "Controller node started.");
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty trajectory.");
            return;
        }

        // Filter out invalid poses (x < 0)
        nav_msgs::msg::Path filtered;
        filtered.header = msg->header;
        for (const auto& pose : msg->poses) {
            if (pose.pose.position.x >= 0.0) {
                filtered.poses.push_back(pose);
            }
        }

        if (filtered.poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "All trajectory points filtered out (x < 0).");
        } else {
            trajectory_ = filtered;
        }
    }

    void control_loop() {
        if (trajectory_.poses.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for valid trajectory...");
            return;
        }

        geometry_msgs::msg::PoseStamped lookahead;
        bool found = false;

        // Find the first point ≥ lookahead distance
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

        // Pure Pursuit steering
        double x = lookahead.pose.position.x;
        double y = lookahead.pose.position.y;
        double L = std::hypot(x, y);
        double steering_angle = std::atan2(2.0 * y, L * L);

        // Publish drive command
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->now();
        drive_msg.drive.speed = velocity_;
        drive_msg.drive.steering_angle = steering_angle;
        drive_pub_->publish(drive_msg);

        // ---- Marker: Lookahead Point ----
        visualization_msgs::msg::Marker lookahead_marker;
        lookahead_marker.header.frame_id = trajectory_.header.frame_id;
        lookahead_marker.header.stamp = drive_msg.header.stamp;
        lookahead_marker.ns = "lookahead_point";
        lookahead_marker.id = 0;
        lookahead_marker.type = visualization_msgs::msg::Marker::SPHERE;
        lookahead_marker.action = visualization_msgs::msg::Marker::ADD;
        lookahead_marker.pose.position = lookahead.pose.position;
        lookahead_marker.pose.orientation.w = 1.0;
        lookahead_marker.scale.x = 0.1;
        lookahead_marker.scale.y = 0.1;
        lookahead_marker.scale.z = 0.1;
        lookahead_marker.color.a = 1.0;
        lookahead_marker.color.g = 1.0;
        lookahead_marker.color.r = 0.0;
        lookahead_marker.color.b = 0.0;
        lookahead_marker_pub_->publish(lookahead_marker);

        // ---- Marker: Steering Arrow ----
        geometry_msgs::msg::Point start, end;
        start.x = 0.0;
        start.y = 0.0;
        end.x = std::cos(steering_angle);
        end.y = std::sin(steering_angle);

        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = trajectory_.header.frame_id;
        arrow_marker.header.stamp = drive_msg.header.stamp;
        arrow_marker.ns = "steering_arrow";
        arrow_marker.id = 1;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;
        arrow_marker.points.push_back(start);
        arrow_marker.points.push_back(end);
        arrow_marker.scale.x = 0.05; // shaft width
        arrow_marker.scale.y = 0.1;  // head width
        arrow_marker.scale.z = 0.1;
        arrow_marker.color.a = 1.0;
        arrow_marker.color.r = 0.0;
        arrow_marker.color.g = 0.0;
        arrow_marker.color.b = 1.0;
        steering_marker_pub_->publish(arrow_marker);

        // RCLCPP_INFO(this->get_logger(), "Steering: %.2f rad, Target: (%.2f, %.2f)", steering_angle, x, y);
    }

    nav_msgs::msg::Path trajectory_;
    double lookahead_distance_;
    double velocity_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr traj_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr steering_marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
