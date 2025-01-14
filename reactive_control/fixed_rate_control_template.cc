#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

using namespace std::chrono_literals;

class LidarControlNode : public rclcpp::Node
{
public:
    LidarControlNode()
    : Node("lidar_control_node")
    {
        // Subscriber to /scan topic
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarControlNode::lidar_callback, this, std::placeholders::_1));
        
        // Publisher to /drive topic
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        
        // Timer for control function at 10Hz
        timer_ = this->create_wall_timer(
            100ms, std::bind(&LidarControlNode::control_callback, this));
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        // Store the latest scan data
        std::lock_guard<std::mutex> lock(scan_mutex_);
        latest_scan_ = *scan_msg;
    }

    void control_callback()
    {
        // Lock the mutex to safely access the latest scan data
        std::lock_guard<std::mutex> lock(scan_mutex_);
        
        if (!latest_scan_.ranges.empty())
        {
            // Process the latest scan data to generate control command
            auto drive_msg = std::make_shared<ackermann_msgs::msg::AckermannDriveStamped>();
            // Example control logic (simple forward command)
            drive_msg->drive.speed = 1.0;
            drive_msg->drive.steering_angle = 0.0;

            // Publish the control command
            drive_pub_->publish(*drive_msg);
        }
    }

    // Subscriber and publisher
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    
    // Timer for control callback
    rclcpp::TimerBase::SharedPtr timer_;

    // Mutex to protect access to the latest scan data
    std::mutex scan_mutex_;
    
    // Variable to store the latest scan data
    sensor_msgs::msg::LaserScan latest_scan_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarControlNode>());
    rclcpp::shutdown();
    return 0;
}
