#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <algorithm>
#include <vector>
#include <cmath>
#include <mutex>

using namespace std::chrono_literals;

class SafetyBubble : public rclcpp::Node {
public:
    SafetyBubble() : Node("safety_bubble") {
        // Parameters
        declare_parameter<double>("bubble_radius", 0.0);
        declare_parameter<double>("speed", 0.0);
        declare_parameter<int>("window_size", 0);
        declare_parameter<double>("steering_fov", 0.0);

        // Subscribers and Publishers
        lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SafetyBubble::lidar_callback, this, std::placeholders::_1));
            
        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);

        // Initialize angles
        bubble_radius_ = get_parameter("bubble_radius").as_double();
        speed_ = get_parameter("speed").as_double();
        window_size_ = get_parameter("window_size").as_int();
        steering_fov_ = get_parameter("steering_fov").as_double();
    }

private:
    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    
    // Algorithm parameters
    double bubble_radius_;
    double speed_;
    int window_size_;
    double steering_fov_;
    
    // States
    std::vector<double> angles_;
    std::vector<int> good_angle_indices_;
    std::mutex data_mutex_;
    bool angles_initialized_ = false;

    std::vector<float> preprocess_lidar(const sensor_msgs::msg::LaserScan& scan) {
        std::vector<float> ranges(scan.ranges.begin(), scan.ranges.end());
        
        // Initialize angles once
        if (!angles_initialized_) {
            initialize_angles(scan);
            angles_initialized_ = true;
        }

        // Clean invalid values
        for (auto& val : ranges) {
            if (std::isnan(val)) val = 0.0f;
            if (std::isinf(val)) val = 5.0f;
        }

        // Apply moving average filter
        std::vector<float> smoothed(ranges.size(), 0.0f);
        const int half_window = window_size_ / 2;
        for (size_t i = 0; i < ranges.size(); ++i) {
            float sum = 0.0f;
            int count = 0;
            for (int j = -half_window; j <= half_window; ++j) {
                int idx = i + j;
                if (idx >= 0 && idx < static_cast<int>(ranges.size())) {
                    sum += ranges[idx];
                    count++;
                }
            }
            smoothed[i] = sum / count;
        }

        std::vector<float> processed;
        processed.reserve(good_angle_indices_.size());
        for (int idx : good_angle_indices_) {
            processed.push_back(std::min(std::max(smoothed[idx], 0.0f), 3.0f));
        }

        return processed;
    }

    void initialize_angles(const sensor_msgs::msg::LaserScan& scan) {
        const double min_angle = scan.angle_min;
        const double angle_inc = scan.angle_increment;
        const double fov_rad = steering_fov_ * M_PI / 180.0;

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            double angle = min_angle + i * angle_inc;
            angles_.push_back(angle);
            if (angle > -fov_rad && angle < fov_rad) {
                good_angle_indices_.push_back(i);
            }
        }
    }

    std::pair<int, int> find_max_gap(const std::vector<float>& ranges) {
        int max_start = 0;
        int max_end = 0;
        int current_start = 0;
        bool in_gap = false;

        for (size_t i = 0; i < ranges.size(); ++i) {
            if (ranges[i] > 0.1f && !in_gap) {
                current_start = i;
                in_gap = true;
            }
            else if ((ranges[i] <= 0.1f || i == ranges.size()-1) && in_gap) {
                in_gap = false;
                if (static_cast<int>(i - current_start) > (max_end - max_start)) {
                    max_start = current_start;
                    max_end = i;
                }
            }
        }
        return {max_start, max_end};
    }

    size_t find_best_point(const std::vector<float>& ranges, int start, int end) {
        size_t best_idx = start;
        float max_val = ranges[start];
        
        for (int i = start; i < end; ++i) {
            if (ranges[i] > max_val) {
                max_val = ranges[i];
                best_idx = i;
            }
        }
        return best_idx;
    }

    void set_bubble(std::vector<float>& ranges, size_t closest_idx) {
        const float bubble_radius = bubble_radius_;
        const float min_dist = ranges[closest_idx];
        const double dtheta = std::atan2(bubble_radius, min_dist);

        const double angle = angles_[good_angle_indices_[closest_idx]];
        const double min_angle = angle - dtheta;
        const double max_angle = angle + dtheta;

        for (size_t i = 0; i < good_angle_indices_.size(); ++i) {
            double current_angle = angles_[good_angle_indices_[i]];
            if (current_angle > min_angle && current_angle < max_angle) {
                ranges[i] = 0.0f;
            }
        }
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        auto processed = preprocess_lidar(*scan);
        if (processed.empty()) return;

        // Find closest point
        size_t closest_idx = std::min_element(processed.begin(), processed.end()) - processed.begin();
        
        // Set bubble
        set_bubble(processed, closest_idx);

        // Find max gap
        std::pair<int, int> gap_result = find_max_gap(processed);
        int gap_start = gap_result.first;
        int gap_end = gap_result.second;
        
        // Find best point
        size_t best_idx = find_best_point(processed, gap_start, gap_end);
        double desired_angle = angles_[good_angle_indices_[best_idx]];

        // Publish drive command
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = now();
        drive_msg.header.frame_id = "laser";
        drive_msg.drive.steering_angle = desired_angle;
        drive_msg.drive.speed = speed_;
        
        drive_pub_->publish(drive_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SafetyBubble>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
