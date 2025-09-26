#include <memory>
#include <limits>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/path.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

using std::placeholders::_1;

class DriveArbitratorNode : public rclcpp::Node
{
public:
  DriveArbitratorNode() : Node("drive_arbitrator_node")
  {
    // Parameters (loaded from YAML)
    controller_cmd_topic_   = this->declare_parameter<std::string>("controller_cmd_topic", "/controller/drive_cmd");
    safety_cmd_topic_       = this->declare_parameter<std::string>("safety_cmd_topic", "/safety_bubble/drive_cmd");
    output_cmd_topic_       = this->declare_parameter<std::string>("output_cmd_topic", "/drive");
    scan_topic_             = this->declare_parameter<std::string>("scan_topic", "/scan");
    left_boundary_topic_    = this->declare_parameter<std::string>("left_boundary_topic", "/left_boundary");
    right_boundary_topic_   = this->declare_parameter<std::string>("right_boundary_topic", "/right_boundary");

    arbitration_hz_         = this->declare_parameter<double>("arbitration_hz", 50.0);
    min_boundary_points_    = this->declare_parameter<int>("min_boundary_points", 5);
    required_boundaries_    = this->declare_parameter<int>("required_boundaries", 2);
    scan_timeout_s_         = this->declare_parameter<double>("scan_timeout", 0.2);
    boundary_timeout_s_     = this->declare_parameter<double>("boundary_timeout", 0.2);
    controller_timeout_s_   = this->declare_parameter<double>("controller_timeout", 0.2);
    safety_timeout_s_       = this->declare_parameter<double>("safety_timeout", 0.5);
    obstacle_stop_distance_ = this->declare_parameter<double>("obstacle_stop_distance", 0.5);
    front_cone_angle_deg_   = this->declare_parameter<double>("front_cone_angle_deg", 30.0);

    // Subscriptions
    controller_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      controller_cmd_topic_, 10,
      std::bind(&DriveArbitratorNode::controllerCallback, this, _1));

    safety_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      safety_cmd_topic_, 10,
      std::bind(&DriveArbitratorNode::safetyCallback, this, _1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10,
      std::bind(&DriveArbitratorNode::scanCallback, this, _1));

    left_boundary_sub_ = create_subscription<nav_msgs::msg::Path>(
      left_boundary_topic_, 10,
      std::bind(&DriveArbitratorNode::leftBoundaryCallback, this, _1));

    right_boundary_sub_ = create_subscription<nav_msgs::msg::Path>(
      right_boundary_topic_, 10,
      std::bind(&DriveArbitratorNode::rightBoundaryCallback, this, _1));

    // Publisher
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(output_cmd_topic_, 10);

    // Arbitration timer
    auto period = std::chrono::milliseconds((int)(1000.0 / arbitration_hz_));
    arbitration_timer_ = this->create_wall_timer(
      period,
      std::bind(&DriveArbitratorNode::arbitrationLoop, this));
  }

private:
  // --- Callbacks ---
  void controllerCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
  {
    latest_controller_cmd_ = msg;
    last_controller_time_ = now();
  }

  void safetyCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
  {
    latest_safety_cmd_ = msg;
    last_safety_time_ = now();
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    latest_scan_ = msg;
    last_scan_time_ = now();
  }

  void leftBoundaryCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    latest_left_boundary_ = msg;
    last_boundary_left_time_ = now();
  }

  void rightBoundaryCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    latest_right_boundary_ = msg;
    last_boundary_right_time_ = now();
  }

  // --- Arbitration Logic ---
  void arbitrationLoop()
  {
    rclcpp::Time now_time = now();

    // Freshness checks
    bool scan_fresh      = (now_time - last_scan_time_).seconds() < scan_timeout_s_;
    bool left_fresh      = (now_time - last_boundary_left_time_).seconds() < boundary_timeout_s_;
    bool right_fresh     = (now_time - last_boundary_right_time_).seconds() < boundary_timeout_s_;
    bool controller_fresh= (now_time - last_controller_time_).seconds() < controller_timeout_s_;
    bool safety_fresh    = (now_time - last_safety_time_).seconds() < safety_timeout_s_;

    // --- F1: Boundary check ---
    int valid_boundary_count = 0;
    if (latest_left_boundary_ && latest_left_boundary_->poses.size() >= (size_t)min_boundary_points_ && left_fresh)
      valid_boundary_count++;
    if (latest_right_boundary_ && latest_right_boundary_->poses.size() >= (size_t)min_boundary_points_ && right_fresh)
      valid_boundary_count++;

    bool sufficient_boundaries = (valid_boundary_count >= required_boundaries_);

    // --- F2: Obstacle front check ---
    bool front_clear = true;
    if (latest_scan_ && scan_fresh) {
      const auto & ranges = latest_scan_->ranges;
      int total_points = ranges.size();
      double angle_min = latest_scan_->angle_min;
      double angle_inc = latest_scan_->angle_increment;

      double min_dist = std::numeric_limits<double>::infinity();

      // define ±cone around front
      double cone_half = (front_cone_angle_deg_ * M_PI / 180.0);
      int start_idx = std::max(0, (int)((-cone_half - angle_min) / angle_inc));
      int end_idx   = std::min(total_points - 1, (int)((cone_half - angle_min) / angle_inc));

      for (int i = start_idx; i <= end_idx; i++) {
        float d = ranges[i];
        if (std::isfinite(d) && d > 0.01) {
          if (d < min_dist) min_dist = d;
        }
      }

      if (min_dist < obstacle_stop_distance_) {
        front_clear = false;
      }
    }

    // --- Arbitration decision ---
    ackermann_msgs::msg::AckermannDriveStamped output_cmd;

    if (sufficient_boundaries && front_clear && latest_controller_cmd_ && controller_fresh) {
      // Use controller command
      output_cmd = *latest_controller_cmd_;
      RCLCPP_DEBUG(this->get_logger(), "Using controller command.");
    } else if (latest_safety_cmd_ && safety_fresh) {
      // Fallback to safety bubble
      output_cmd = *latest_safety_cmd_;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Fallback: using safety bubble command.");
    } else {
      // Emergency stop
      output_cmd.header.stamp = now();
      output_cmd.drive.speed = 0.0;
      output_cmd.drive.steering_angle = 0.0;
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "No valid command -> STOP.");
    }

    drive_pub_->publish(output_cmd);
  }

  // --- Node State ---
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr controller_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr safety_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr left_boundary_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr right_boundary_sub_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

  rclcpp::TimerBase::SharedPtr arbitration_timer_;

  // Latest msgs
  ackermann_msgs::msg::AckermannDriveStamped::SharedPtr latest_controller_cmd_;
  ackermann_msgs::msg::AckermannDriveStamped::SharedPtr latest_safety_cmd_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  nav_msgs::msg::Path::SharedPtr latest_left_boundary_;
  nav_msgs::msg::Path::SharedPtr latest_right_boundary_;

  // Timestamps
  rclcpp::Time last_scan_time_{0,0,RCL_ROS_TIME};
  rclcpp::Time last_boundary_left_time_{0,0,RCL_ROS_TIME};
  rclcpp::Time last_boundary_right_time_{0,0,RCL_ROS_TIME};
  rclcpp::Time last_controller_time_{0,0,RCL_ROS_TIME};
  rclcpp::Time last_safety_time_{0,0,RCL_ROS_TIME};

  // Parameters
  std::string controller_cmd_topic_;
  std::string safety_cmd_topic_;
  std::string output_cmd_topic_;
  std::string scan_topic_;
  std::string left_boundary_topic_;
  std::string right_boundary_topic_;
  double arbitration_hz_;
  int min_boundary_points_;
  int required_boundaries_;
  double scan_timeout_s_;
  double boundary_timeout_s_;
  double controller_timeout_s_;
  double safety_timeout_s_;
  double obstacle_stop_distance_;
  double front_cone_angle_deg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveArbitratorNode>());
  rclcpp::shutdown();
  return 0;
}
