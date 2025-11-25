#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <cmath>

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode() : Node("controller_node") {
        // Parameters with default values to be reset by launch
        declare_parameter<double>("lookahead_distance", 0.5);
        declare_parameter<double>("velocity_max", 2.5);
        declare_parameter<double>("velocity_min", 1.0);
        declare_parameter<double>("steering_max_deg", 20.6);
        
        traj_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "trajectory", 10, std::bind(&ControllerNode::path_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/controller/drive_cmd", 10);
        lookahead_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lookahead_marker", 10);
        steering_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("steering_arrow_marker", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControllerNode::control_loop, this));

        lookahead_distance_ = get_parameter("lookahead_distance").as_double();
        velocity_max_ = get_parameter("velocity_max").as_double();
        velocity_min_ = get_parameter("velocity_min").as_double();
        steering_max_deg_ = get_parameter("steering_max_deg").as_double();

        RCLCPP_INFO(this->get_logger(), "Controller node started.");
    }

private:
    // Subscribers and publishers.
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr traj_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr steering_marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // trajectory message
    nav_msgs::msg::Path trajectory_;
    bool trajectory_valid_ = false;
    rclcpp::Time last_traj_time_;
    
    // Vehicle specs
    double  wheelbase_ = 0.33;

    // Parameters
    double lookahead_distance_;
    double velocity_max_;
    double velocity_min_;
    double steering_max_deg_;

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty trajectory.");
            trajectory_valid_ = false;
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
            trajectory_valid_ = false;
        } else {
            trajectory_ = filtered;
            trajectory_valid_ = true;
            last_traj_time_ = this->now();
        }
    }

    double compute_speed(double steering_angle) const {
        double abs_angle = std::abs(steering_angle);
        double steering_max_rad = steering_max_deg_ * M_PI / 180.0;
        double t = std::clamp(abs_angle / steering_max_rad, 0.0, 1.0);
        
        // Apply exponential curve: higher n = faster drop at large angles
        double n = 1.0;  // tweak this as needed
        double scaled_t = std::pow(t, n);

        return velocity_max_ - scaled_t * (velocity_max_ - velocity_min_);
    }

    void control_loop() {
        auto now = this->now();
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = now;

        // Timeout check: trajectory freshness
        if (trajectory_valid_ && (now - last_traj_time_).seconds() > 0.5) {
            RCLCPP_WARN(this->get_logger(), "Trajectory timeout (>0.5s), holding zero command.");
            trajectory_valid_ = false;
        }

        geometry_msgs::msg::PoseStamped lookahead;
        bool found = false;

        if (trajectory_valid_) {
            // Find first lookahead point beyond threshold
            for (const auto &pose : trajectory_.poses) {
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
                trajectory_valid_ = false;
            }
        }

        double steering_angle = 0.0;
        if (trajectory_valid_) {
            // Pure Pursuit steering
            double x = lookahead.pose.position.x;
            double y = lookahead.pose.position.y;
            double L = std::hypot(x, y);
            double curvature = 2.0 * y / (L * L);
            steering_angle = std::atan(curvature * wheelbase_);

            // Publish drive command
            drive_msg.drive.speed = compute_speed(steering_angle);
            drive_msg.drive.steering_angle = steering_angle;
            drive_pub_->publish(drive_msg);
        } else {
            drive_msg.drive.speed = 0.0;
            drive_msg.drive.steering_angle = 0.0;
            drive_pub_->publish(drive_msg);
        }

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
        arrow_marker.action = trajectory_valid_ && found
                                  ? visualization_msgs::msg::Marker::ADD
                                  : visualization_msgs::msg::Marker::DELETE;
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
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
