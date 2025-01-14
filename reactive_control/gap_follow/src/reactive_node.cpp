#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include <algorithm>

template <typename T>
T clamp(T value, T min, T max) {
    return (value < min) ? min : (value > max) ? max : value;
}

// to facilitate more intuitive and readable expressions for time durations
using namespace std::chrono_literals;

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// create ROS subscribers and publishers
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", 10, std::bind(&ReactiveFollowGap::scan_callback, this, std::placeholders::_1));
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "ego_racecar/odom", 10, std::bind(&ReactiveFollowGap::drive_callback, this, std::placeholders::_1));
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

        // Timer for control function at 20Hz
        timer_ = this->create_wall_timer(50ms, std::bind(&ReactiveFollowGap::control_callback, this));

        RCLCPP_INFO(this->get_logger(), "Reactive Node has been started");

        // Declare parameters with default values
        this->declare_parameter<double>("kp", 0.0);
        this->declare_parameter<double>("ki", 0.0);
        this->declare_parameter<double>("kd", 0.0);

        this->declare_parameter<double>("ttc_threshold_sec", 0.0);
        this->declare_parameter<double>("side_limit_m", 0.0);
        this->declare_parameter<double>("bubble_window_deg", 0.0);
        this->declare_parameter<double>("counter_steer_deg", 0.0);

        this->declare_parameter<double>("speed_high", 0.0);
        this->declare_parameter<double>("speed_mid", 0.0);
        this->declare_parameter<double>("speed_low", 0.0);
        this->declare_parameter<double>("goal_window_deg", 0.0);

        // Get parameters from the parameter server (YAML file)
        kp_ = this->get_parameter("kp").as_double();
        ki_ = this->get_parameter("ki").as_double();
        kd_ = this->get_parameter("kd").as_double();

        ttc_threshold_sec_ = this->get_parameter("ttc_threshold_sec").as_double();
        side_limit_m_ = this->get_parameter("side_limit_m").as_double();
        bubble_window_deg_ = this->get_parameter("bubble_window_deg").as_double();
        counter_steer_deg_ = this->get_parameter("counter_steer_deg").as_double();

        speed_high_ = this->get_parameter("speed_high").as_double();
        speed_mid_ = this->get_parameter("speed_mid").as_double();
        speed_low_ = this->get_parameter("speed_low").as_double();
        goal_window_deg_ = this->get_parameter("goal_window_deg").as_double();
    }

private:
    // flags
    bool scan_received_ = false;
    bool AEB_on_ = false; 

    double veh_half_width_ = 0.15; // 296mm wide for Traxxas Slash 4x4 Premium Chassis
    double veh_wheelbase_ = 0.32; 
    double steering_correction_ = 0.0;
    double dt = 0.05;
    double speed_curr = 0.0;

    // PID CONTROL PARAMS
    double kp_;
    double ki_;
    double kd_;

    // Protection PARAMS
    double ttc_threshold_sec_;
    double side_limit_m_;
    double bubble_window_deg_;
    double counter_steer_deg_;

    // Speed and performance
    double speed_high_;
    double speed_mid_;
    double speed_low_;
    double goal_window_deg_;

    double prev_error = 0.0;
    double integral = 0.0;

    /// create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

    // Timer for control callback
    rclcpp::TimerBase::SharedPtr timer_;

    // Mutex to protect access to the latest scan data
    std::mutex scan_mutex_;
    
    // Variable to store the latest scan data
    sensor_msgs::msg::LaserScan latest_scan_;
    

    void preprocess_lidar(std::vector<float>& ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)

        int size = ranges.size();
        if (size == 0) {
            RCLCPP_WARN(this->get_logger(), "Empty ranges received in preprocess_lidar.");
            return;
        }

        // Step 1: Convert NaNs and infinity values to 0.0
        for (int i = 0; i < size; ++i) {
            if (std::isnan(ranges[i]) || std::isinf(ranges[i])|| ranges[i] < 0.0f) {
                ranges[i] = 0.0f;
            }
        }

        // Step 2: Average each 10 beams, ignoring 0.0 values, and replace all 10 beams with the average
        for (int i = 0; i < size; i += 10) {
            float sum = 0.0f;
            int count = 0;
            
            // Calculate the sum and count of non-zero values in the group
            for (int j = i; j < i + 10 && j < size; ++j) {
                if (ranges[j] != 0.0f) {
                    sum += ranges[j];
                    ++count;
                }
            }
            
            // Compute the average, if there are non-zero values
            float average = (count > 0) ? sum / count : 0.0f;
            
            // Replace all values in the current group with the average
            for (int j = i; j < i + 10 && j < size; ++j) {
                ranges[j] = average;
            }
        }
    }

    void find_max_gap(std::vector<float>& ranges, int* indice, float angle_min, float angle_increment_rad)
    {   
        // bounce checking
        int size = ranges.size();
        if (size == 0) {
            RCLCPP_WARN(this->get_logger(), "Empty ranges received in find_max_gap.");
            indice[0] = 0;
            indice[1] = 0;
            return;
        }
        
        // Find the closest point safely within the 'front window'

        float window_range = bubble_window_deg_ * (M_PI / 180);
        int window_start_index = static_cast<int>(((-window_range/2)-angle_min)/angle_increment_rad) - 1;
        int window_end_index = static_cast<int>(((window_range/2)-angle_min)/angle_increment_rad) - 1;

        auto min_iter = std::min_element(ranges.begin() + window_start_index, ranges.begin() + window_end_index);
        int min_index = std::distance(ranges.begin(), min_iter);
        float min_value = *min_iter;

        RCLCPP_INFO(this->get_logger(), "Min dist %f [m] at %f [deg]", 
                    min_value, (angle_min + angle_increment_rad * min_index)*(180.0 / M_PI));
        
        // Compute bubble size with safety checks
        float bubble_ang_rad = std::atan2(veh_half_width_, min_value);
        int bubble_range_index = static_cast<int>(bubble_ang_rad / angle_increment_rad);
        
        int start = std::max(0, min_index - bubble_range_index);
        int end = std::min(size - 1, min_index + bubble_range_index);
        
        // Zero out points within the bubble
        std::fill(ranges.begin() + start, ranges.begin() + end + 1, 0.0f);

        // find the longest gap range
        int gap_1_lb = 0;
        int gap_1_rb = min_index - bubble_range_index - 1;
        while (ranges[gap_1_rb]==0.0f && gap_1_rb > gap_1_lb) {
           gap_1_rb--; 
        }
        
        int gap_2_rb = size - 1;
        int gap_2_lb = min_index + bubble_range_index + 1;
        while (ranges[gap_2_lb]==0.0f && gap_2_lb < gap_2_rb) {
           gap_2_lb++; 
        }

        int gap_1_len = gap_1_rb - gap_1_lb;
        int gap_2_len = gap_2_rb - gap_2_lb;

        if (gap_2_len > gap_1_len) {
            indice[0] = gap_2_lb;
            indice[1] = gap_2_rb;
        } else {
            indice[0] = gap_1_lb;
            indice[1] = gap_1_rb;
        }

        RCLCPP_INFO(this->get_logger(), "Max gap at (%f , %f) [deg]", 
                    (angle_min + angle_increment_rad * indice[0])*(180.0 / M_PI),
                    (angle_min + angle_increment_rad * indice[1])*(180.0 / M_PI));

    }

    int find_max_dist_point(std::vector<float>& ranges, const std::vector<float>& scan_angles, int* indice, float angle_increment)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        float max_dist = ranges[indice[0]];
        int max_dist_index = indice[0];

        for (int i = indice[0]+1; i < indice[1]; i++) 
        {
            if (ranges[i] > max_dist) 
            {
                max_dist = ranges[i];
                max_dist_index = i;
            }
        }

        // Smooth the turn by find the mininum angle in a angle range
        int window_lb_index = max_dist_index - ((goal_window_deg_/2) * (M_PI / 180.0) / angle_increment);
        int window_rb_index = max_dist_index + ((goal_window_deg_/2) * (M_PI / 180.0) / angle_increment);
        // find the minimum angle so that with minimum effort to turn
        int best_point_index = window_lb_index;
        for (int j = window_lb_index+1; j < window_rb_index; j++)
        {
            if (std::abs(scan_angles[j]) < std::abs(scan_angles[best_point_index]))
            {
               best_point_index = j; 
            }
        }
        best_point_index = clamp(best_point_index, indice[0], indice[1]);
        
        return best_point_index;
    }

    int find_mid_point(int* indice)
    {
        int mid_point = (indice[0] + indice[1]) / 2;
        return mid_point;
    }

    bool emergency_brake(std::vector<float>& ranges, const std::vector<float>& scan_angles)
    {
        // Get the array of range rates
        int size = scan_angles.size();
        std::vector<float> range_rates(size);
        for (int i = 0; i < size; ++i) 
        {
            range_rates[i] = std::max(speed_curr * std::cos(scan_angles[i]), 0.0);
        }

        // Get the TTC array
        std::vector<float> TTC_array(size);
        for (int i = 0; i < size; ++i) {
            if (std::isnan(ranges[i])|| ranges[i] <= 0.0f) {
                TTC_array[i] = std::numeric_limits<float>::infinity();
            } else {
                TTC_array[i] = (range_rates[i] != 0) ? (ranges[i] / range_rates[i]) : std::numeric_limits<float>::infinity();
            }
        }

        /// publish drive/brake message
        float min_value = TTC_array[0];
        int min_index = 0;

        for (int i = 1; i < size; i++) {
            if (TTC_array[i] < min_value) {
                min_value = TTC_array[i];
                min_index = i;
            }
        }

        if (min_value <= static_cast<float>(ttc_threshold_sec_))
        {
            float AEB_angle = scan_angles[min_index] * (180.0 / M_PI);
            RCLCPP_INFO(this->get_logger(), "AEB on with TTC:  %f [sec] at the angle %f [deg], with dist %f [m] at index %d, of range rate %f [m/s]", 
                        min_value, AEB_angle, ranges[min_index], min_index, range_rates[min_index]);
            return true;
        } else {
            return false;
        }
    }

    bool side_protection(std::vector<float>& ranges, const std::vector<float>& scan_angles, double& steering_correction)
    {
        int size = scan_angles.size();
        // Right side protection: Too close on the right (-π/2 and below), steer left (positive correction)
        for (int i = 0; scan_angles[i] < (-M_PI / 2); i++) 
        {
            if (!std::isnan(ranges[i]) && (ranges[i] > 0.0) && (ranges[i] <= side_limit_m_)) 
            {
                RCLCPP_INFO(this->get_logger(), "Too close on the right at %f [deg] with dist %f [m]", 
                            scan_angles[i] * (180.0 / M_PI), ranges[i]);

                // Apply a left counter-steer (positive angle)
                steering_correction = counter_steer_deg_ * (M_PI / 180);
                return true;
            }
        }

        // Left side protection: Too close on the left (π/2 and above), steer right (negative correction)
        for (int j = size - 1; scan_angles[j] > (M_PI / 2); j--) 
        {
            if (!std::isnan(ranges[j]) && (ranges[j] > 0.0) && (ranges[j] <= side_limit_m_))
            {
                RCLCPP_INFO(this->get_logger(), "Too close on the left at %f [deg] with dist %f [m]", 
                            scan_angles[j] * (180.0 / M_PI), ranges[j]);

                // Apply a right counter-steer (negative angle)
                steering_correction = - counter_steer_deg_ * (M_PI / 180);;
                return true;
            }
        }

        // No side protection needed
        steering_correction = 0.0;
        return false;
    }


    void control_command(float angle_error, double time_diff, bool side_protect_need)
    {   
        // Based on the calculated error, publish vehicle control
        // double yaw_rate_desired = static_cast<double>(angle_error);
        // double steering_angle = (speed > 0.0) ? atan(yaw_rate_desired * veh_wheelbase_ / speed) : 0.0;
        // double steering_angle = yaw_rate_desired;


        // Using PID to generate steering angle command
        double steering_angle = kp_ * angle_error + ki_ * integral + kd_ * (angle_error - prev_error) / time_diff;

        // Clamp steering angle to vehicle's limits
        const double max_steering_angle = 1.0; 
        steering_angle = clamp(steering_angle, -max_steering_angle, max_steering_angle);
        
        // Determine velocity based on steering angle
        double angle_deg = steering_angle * (180.0 / M_PI);
        double velocity = speed_high_; // default speed
        
        if (std::abs(angle_deg) > 20.0) {
            velocity = speed_low_;
        } else if (std::abs(angle_deg) > 10.0) {
            velocity = speed_mid_;
        }

        // Prepare and publish drive message
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "base_link"; 
        // fill in drive message and publish
        drive_msg.drive.speed = static_cast<float>(velocity);
        drive_msg.drive.steering_angle = side_protect_need ? 
            static_cast<float>(steering_correction_) : static_cast<float>(steering_angle);
        drive_publisher_ ->publish(drive_msg);
        
        RCLCPP_INFO(this->get_logger(), "Published drive command: speed=%.2f m/s, steering=%.2f degrees",
                drive_msg.drive.speed, angle_deg);

        integral += angle_error * dt;
        prev_error = angle_error;
    }


    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// update current speed
        speed_curr = msg->twist.twist.linear.x;
    }


    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Store the latest scan data
        std::lock_guard<std::mutex> lock(scan_mutex_);
        latest_scan_ = *scan_msg;
        scan_received_ = true;
    }


    void control_callback()
    {
        // Lock the mutex to safely access the latest scan data
        std::lock_guard<std::mutex> lock(scan_mutex_);

        if (!scan_received_) {
            RCLCPP_WARN(this->get_logger(), "No scan data received yet. Skipping control computation.");
            return;
        }
        
        if (latest_scan_.ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty scan data. Skipping control computation.");
            return;
        }

        preprocess_lidar(latest_scan_.ranges);
        
        // bool activate_AEB = emergency_brake(latest_scan_.ranges, latest_scan_.angle_min, latest_scan_.angle_increment);

        // if (activate_AEB) {
        //     AEB_on_ = true;
        // // } else if (AEB_on_) {
        // //     // Check if conditions have improved to deactivate AEB
        // //     AEB_on_ = emergency_brake(latest_scan_.ranges, latest_scan_.angle_min, latest_scan_.angle_increment);
        // }
        
        // if (AEB_on_) {
        //     auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        //     drive_msg.header.stamp = this->now();
        //     drive_msg.header.frame_id = "base_link";
        //     drive_msg.drive.speed = 0.0;
        //     drive_msg.drive.steering_angle = 0.0;
        //     drive_publisher_->publish(drive_msg);
        //     // RCLCPP_WARN(this->get_logger(), "Emergency Brake Activated!");
        //     return;
        // }
        
        int gap_indices[2] = {0, static_cast<int>(latest_scan_.ranges.size() - 1)};

        // set up a scan angles array
        int size = latest_scan_.ranges.size();
        std::vector<float> lidar_scan_angles(size);
        for (int i = 0; i < size; ++i) 
        {
            lidar_scan_angles[i] = latest_scan_.angle_min + i * latest_scan_.angle_increment;
        }

        find_max_gap(latest_scan_.ranges, gap_indices, latest_scan_.angle_min, latest_scan_.angle_increment);
        int goal_index = find_max_dist_point(latest_scan_.ranges, lidar_scan_angles, gap_indices, latest_scan_.angle_increment);
        // int goal_index = find_mid_point(gap_indices);
        
        float delta_theta = lidar_scan_angles[goal_index];
        RCLCPP_INFO(this->get_logger(), "Goal point: index=%d, angle=%.2f degrees",
                goal_index, delta_theta * (180.0 / M_PI));
        bool activate_side_protect = side_protection(latest_scan_.ranges, lidar_scan_angles, steering_correction_);
        
        control_command(delta_theta, dt, activate_side_protect);
    }
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}