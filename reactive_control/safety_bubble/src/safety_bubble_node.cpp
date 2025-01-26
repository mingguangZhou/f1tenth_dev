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
        declare_parameter<double>("perception_fov_deg", 0.0);
        declare_parameter<double>("kp", 0.0);
        declare_parameter<double>("ki", 0.0);
        declare_parameter<double>("kd", 0.0);
        declare_parameter<double>("steering_fov_deg", 0.0);

        // Subscribers and Publishers
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SafetyBubble::scan_callback, this, std::placeholders::_1));
            
        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);

        // Timer for control function at 20Hz
        timer_ = create_wall_timer(50ms, std::bind(&SafetyBubble::control_callback, this));

        // Initialize angles
        bubble_radius_ = get_parameter("bubble_radius").as_double();
        speed_ = get_parameter("speed").as_double();
        window_size_ = get_parameter("window_size").as_int();
        perception_fov_deg_ = get_parameter("perception_fov_deg").as_double();

        kp_ = get_parameter("kp").as_double();
        ki_ = get_parameter("ki").as_double();
        kd_ = get_parameter("kd").as_double();
        steering_fov_deg_ = get_parameter("steering_fov_deg").as_double();

        // Log parameters
        RCLCPP_INFO(this->get_logger(), "Initialized with parameters:");
        RCLCPP_INFO(this->get_logger(), "  bubble_radius        = %f", bubble_radius_);
        RCLCPP_INFO(this->get_logger(), "  speed               = %f", speed_);
        RCLCPP_INFO(this->get_logger(), "  window_size         = %d", window_size_);
        RCLCPP_INFO(this->get_logger(), "  perception_fov_deg  = %f", perception_fov_deg_);
        RCLCPP_INFO(this->get_logger(), "  kp                  = %f", kp_);
        RCLCPP_INFO(this->get_logger(), "  ki                  = %f", ki_);
        RCLCPP_INFO(this->get_logger(), "  kd                  = %f", kd_);
        RCLCPP_INFO(this->get_logger(), "  steering_fov_deg    = %f", steering_fov_deg_);

        prev_time_ = this->now();
    }

private:
    // boolean states
    bool scan_received_ = true;

    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    // Variable to store the latest scan data
    sensor_msgs::msg::LaserScan latest_scan_;
    
    // Timer for control callback
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time prev_time_;

    // Algorithm parameters
    double bubble_radius_;
    double speed_;
    int window_size_;
    double perception_fov_deg_;

    // PID parameters
    double kp_;
    double ki_;
    double kd_;
    double steering_fov_deg_;
    
    // States
    std::vector<double> angles_;
    std::vector<int> good_angle_indices_;
    std::mutex data_mutex_;
    bool angles_initialized_ = false;

    // PID states
    double integral_ = 0.0;
    double prev_error_ = 0.0;
    double prev_steering_angle_ = 0.0;

    std::vector<float> preprocess_scan(const sensor_msgs::msg::LaserScan& scan) {
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
        const double fov_rad = perception_fov_deg_ * M_PI / 180.0;

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            double angle = min_angle + i * angle_inc;
            angles_.push_back(angle);
            if (angle > -fov_rad && angle < fov_rad) {
                good_angle_indices_.push_back(i);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Angles initialized. Total angles: %zu, Good angle indices: %zu",
                    angles_.size(), good_angle_indices_.size());
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
        RCLCPP_INFO(this->get_logger(),
                    "Bubble set around closest_idx=%zu (dist=%.3f, angle=%.3f) -> removed angles in [%.3f, %.3f]",
                    closest_idx, min_dist, angle, min_angle, max_angle);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_scan_ = *scan_msg;
        scan_received_ = true;
    }

    void control_callback() {
        // Lock the mutex to safely access the latest scan data
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (!scan_received_) {
            RCLCPP_WARN(this->get_logger(), "No scan data received yet. Skipping control computation.");
            return;
        }
        
        if (latest_scan_.ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty scan data. Skipping control computation.");
            return;
        }

        auto processed = preprocess_scan(latest_scan_);
        if (processed.empty()) {
            return;
        }

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

        auto current_time = now();
        double dt = (current_time - prev_time_).seconds();

        // Handle first run or large time jumps
        if (dt <= 0 || dt > 0.1) dt = 0.05;  // Default to 20Hz

        double error = desired_angle - prev_steering_angle_;

        // Calculate PID terms
        double P = kp_ * error;
        integral_ += error * dt;
        double I = ki_ * integral_;
        double D = kd_ * ((error - prev_error_) / dt);

        // Calculate steering adjustment
        double steering_output = P + I + D;
        double new_steering_angle = prev_steering_angle_ + steering_output;
        double steering_fov_rad = steering_fov_deg_ * M_PI / 180.0;

        // Clamp steering angle to physical limits
        new_steering_angle = std::clamp(new_steering_angle, 
                                       -steering_fov_rad, 
                                       steering_fov_rad);

        // Only accumulate integral when not saturated
        if (new_steering_angle != prev_steering_angle_ + steering_output) {
            integral_ -= error * dt;
        }

        // Update PID state
        prev_error_ = error;
        prev_steering_angle_ = new_steering_angle;
        prev_time_ = current_time;

        // Log key info
        // RCLCPP_INFO(this->get_logger(),
        //             "closest_idx=%zu gap=[%d,%d] best_idx=%zu desired_angle=%.4f dt=%.4f error=%.4f",
        //             closest_idx, gap_start, gap_end, best_idx, desired_angle, dt, error);

        // RCLCPP_INFO(this->get_logger(),
        //             "PID: P=%.4f I=%.4f D=%.4f => steering_output=%.4f => new_angle=%.4f",
        //             P, I, D, steering_output, new_steering_angle);

        // Publish PID-corrected command
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = current_time;
        drive_msg.header.frame_id = "laser";
        drive_msg.drive.steering_angle = new_steering_angle;
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
