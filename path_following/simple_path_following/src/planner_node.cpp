#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode() : Node("planner_node") {
        // Subscription to left and right boundary topics
        left_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "left_boundary", 10,
            std::bind(&PlannerNode::left_callback, this, std::placeholders::_1));

        right_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "right_boundary", 10,
            std::bind(&PlannerNode::right_callback, this, std::placeholders::_1));

        // Publisher for centerline trajectory
        traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);

        RCLCPP_INFO(this->get_logger(), "Planner node started.");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr left_sub_, right_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
    nav_msgs::msg::Path::SharedPtr latest_left_, latest_right_;

    void left_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        latest_left_ = msg;
        try_publish_centerline();
    }

    void right_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        latest_right_ = msg;
        try_publish_centerline();
    }

    void try_publish_centerline() {
        // Ensure both boundaries have been received
        if (!latest_left_ || !latest_right_) return;

        const auto& left = latest_left_->poses;
        const auto& right = latest_right_->poses;
        size_t n = std::min(left.size(), right.size());

        if (n == 0) return;

        nav_msgs::msg::Path center_path;
        center_path.header.frame_id = "ego_racecar/laser";
        center_path.header.stamp = this->now();

        // Add starting point at ego position (0, 0)
        geometry_msgs::msg::PoseStamped origin_pose;
        origin_pose.header = center_path.header;
        origin_pose.pose.position.x = 0.0;
        origin_pose.pose.position.y = 0.0;
        origin_pose.pose.orientation.w = 1.0;
        center_path.poses.push_back(origin_pose);

        for (size_t i = 0; i < n; ++i) {
            const auto& lp = left[i].pose.position;
            const auto& rp = right[i].pose.position;

            // Compute midpoint
            geometry_msgs::msg::PoseStamped mid_pose;
            mid_pose.header = center_path.header;
            mid_pose.pose.position.x = 0.5 * (lp.x + rp.x);
            mid_pose.pose.position.y = 0.5 * (lp.y + rp.y);
            mid_pose.pose.orientation.w = 1.0;

            // Skip points behind ego car
            if (mid_pose.pose.position.x < 0.0) continue;

            center_path.poses.push_back(mid_pose);
        }

        traj_pub_->publish(center_path);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
