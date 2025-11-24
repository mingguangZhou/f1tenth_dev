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
    // -------------------------- Parameters --------------------------
    // Topics
    declare_parameter<std::string>("controller_cmd_topic", "/controller/drive_cmd");
    declare_parameter<std::string>("safety_cmd_topic", "/safety_bubble/drive_cmd");
    declare_parameter<std::string>("output_cmd_topic", "/drive");
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("left_boundary_topic", "/left_boundary");
    declare_parameter<std::string>("right_boundary_topic", "/right_boundary");

    // Arbitration control
    declare_parameter<bool>("enable_arbitration", true);
    declare_parameter<double>("arbitration_hz", 50.0);

    // Boundary & timeout params
    declare_parameter<int>("min_boundary_points", 5);
    declare_parameter<int>("required_boundaries", 2);
    declare_parameter<double>("scan_timeout_s", 0.2);
    declare_parameter<double>("boundary_timeout_s", 0.2);
    declare_parameter<double>("controller_timeout_s", 0.2);
    declare_parameter<double>("safety_timeout_s", 0.5);

    // Safety parameters
    declare_parameter<double>("obstacle_stop_distance", 0.6);
    declare_parameter<double>("front_cone_angle_deg", 30.0);

    // Between-boundaries region parameters
    declare_parameter<double>("between_region_distance", 2.0);
    declare_parameter<int>("between_region_min_points", 500);
    declare_parameter<double>("scanner_origin_x", 0.27);

    // -------------------------- Load Parameters --------------------------
    controller_cmd_topic_   = get_parameter("controller_cmd_topic").as_string();
    safety_cmd_topic_       = get_parameter("safety_cmd_topic").as_string();
    output_cmd_topic_       = get_parameter("output_cmd_topic").as_string();
    scan_topic_             = get_parameter("scan_topic").as_string();
    left_boundary_topic_    = get_parameter("left_boundary_topic").as_string();
    right_boundary_topic_   = get_parameter("right_boundary_topic").as_string();

    enable_arbitration_     = get_parameter("enable_arbitration").as_bool();
    arbitration_hz_         = get_parameter("arbitration_hz").as_double();

    min_boundary_points_    = get_parameter("min_boundary_points").as_int();
    required_boundaries_    = get_parameter("required_boundaries").as_int();
    scan_timeout_s_         = get_parameter("scan_timeout_s").as_double();
    boundary_timeout_s_     = get_parameter("boundary_timeout_s").as_double();
    controller_timeout_s_   = get_parameter("controller_timeout_s").as_double();
    safety_timeout_s_       = get_parameter("safety_timeout_s").as_double();

    obstacle_stop_distance_ = get_parameter("obstacle_stop_distance").as_double();
    front_cone_angle_deg_   = get_parameter("front_cone_angle_deg").as_double();

    between_region_distance_     = get_parameter("between_region_distance").as_double();
    between_region_min_points_   = get_parameter("between_region_min_points").as_int();
    scanner_origin_x_            = get_parameter("scanner_origin_x").as_double();

    // -------------------------- Subscriptions --------------------------
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
  // -------------------- Callbacks --------------------
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

  // -------------------- Arbitration Logic --------------------
  void arbitrationLoop()
  {
    rclcpp::Time now_time = now();

    if (!enable_arbitration_) {
      if (latest_controller_cmd_) {
        drive_pub_->publish(*latest_controller_cmd_);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "[ARB-OFF] Forwarding controller command.");
      }
      return;
    }

    // Freshness
    bool scan_fresh      = (now_time - last_scan_time_).seconds() < scan_timeout_s_;
    bool left_fresh      = (now_time - last_boundary_left_time_).seconds() < boundary_timeout_s_;
    bool right_fresh     = (now_time - last_boundary_right_time_).seconds() < boundary_timeout_s_;
    bool controller_fresh= (now_time - last_controller_time_).seconds() < controller_timeout_s_;
    bool safety_fresh    = (now_time - last_safety_time_).seconds() < safety_timeout_s_;

    // --- F1: Boundary check ---
    int valid_boundary_count = 0;
    size_t left_count = latest_left_boundary_ ? latest_left_boundary_->poses.size() : 0;
    size_t right_count = latest_right_boundary_ ? latest_right_boundary_->poses.size() : 0;

    if (latest_left_boundary_ && left_count >= (size_t)min_boundary_points_ && left_fresh)
      valid_boundary_count++;
    if (latest_right_boundary_ && right_count >= (size_t)min_boundary_points_ && right_fresh)
      valid_boundary_count++;

    bool sufficient_boundaries = (valid_boundary_count >= required_boundaries_);

    // --- F2: Obstacle front check ---
    bool front_clear = true;
    double min_front_dist = std::numeric_limits<double>::infinity();

    if (latest_scan_ && scan_fresh && !latest_scan_->ranges.empty()) {
      const auto & ranges = latest_scan_->ranges;
      int total_points = ranges.size();
      double angle_min = latest_scan_->angle_min;
      double angle_inc = latest_scan_->angle_increment;

      double cone_half = (front_cone_angle_deg_ * M_PI / 180.0);
      int start_idx = std::max(0, (int)((-cone_half - angle_min) / angle_inc));
      int end_idx   = std::min(total_points - 1, (int)((cone_half - angle_min) / angle_inc));

      for (int i = start_idx; i <= end_idx; i++) {
        float d = ranges[i];
        if (std::isfinite(d) && d > 0.01)
          if (d < min_front_dist) min_front_dist = d;
      }
      if (min_front_dist < obstacle_stop_distance_) front_clear = false;
    }

    // --- F3: Between-boundaries region obstacle check ---
    bool region_clear = true;

    if (latest_scan_ && latest_left_boundary_ && latest_right_boundary_ &&
        left_count >= 2 && right_count >= 2 && scan_fresh && left_fresh && right_fresh)
    {
      // pick 4 candidate points (start & end of each)
      std::vector<std::pair<double,double>> left_pts = {
        {latest_left_boundary_->poses.front().pose.position.x, latest_left_boundary_->poses.front().pose.position.y},
        {latest_left_boundary_->poses.back().pose.position.x,  latest_left_boundary_->poses.back().pose.position.y}
      };
      std::vector<std::pair<double,double>> right_pts = {
        {latest_right_boundary_->poses.front().pose.position.x, latest_right_boundary_->poses.front().pose.position.y},
        {latest_right_boundary_->poses.back().pose.position.x,  latest_right_boundary_->poses.back().pose.position.y}
      };

      // find closest pair
      double min_dist = std::numeric_limits<double>::infinity();
      std::pair<double,double> L_pt, R_pt;
      for (auto &lp : left_pts) {
        for (auto &rp : right_pts) {
          double dx = lp.first - rp.first;
          double dy = lp.second - rp.second;
          double d = std::hypot(dx, dy);
          if (d < min_dist) {
            min_dist = d;
            L_pt = lp;
            R_pt = rp;
          }
        }
      }

      // compute angles from scanner origin (scanner at x = scanner_origin_x_, y = 0)
      double angle_L = std::atan2(L_pt.second - 0.0, L_pt.first - scanner_origin_x_);
      double angle_R = std::atan2(R_pt.second - 0.0, R_pt.first - scanner_origin_x_);
      // shortest signed difference from L to R
      double delta = std::atan2(std::sin(angle_R - angle_L), std::cos(angle_R - angle_L));
      // center angle is halfway along the shortest arc
      double center_angle = angle_L + delta * 0.5;
      // half-angle range (positive)
      double half_angle_range = std::abs(delta) * 0.5;

      // count scan points within the region
      const auto & ranges = latest_scan_->ranges;
      int total_points = ranges.size();
      double angle_min = latest_scan_->angle_min;
      double angle_inc = latest_scan_->angle_increment;

      int inside_count = 0;
      for (int i = 0; i < total_points; i++) {
        double ang = angle_min + i * angle_inc;
        double dist = ranges[i];
        if (!std::isfinite(dist) || dist <= 0.01) continue;
        if (std::abs(ang - center_angle) < half_angle_range &&
            dist < between_region_distance_)
        {
          inside_count++;
        }
      }

      if (inside_count > between_region_min_points_) {
        region_clear = false;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Between-boundaries region blocked: %d scan points inside.", inside_count);
      }
    }

    // --- F4: Controller validity ---
    bool controller_cmd_valid = (latest_controller_cmd_) && (latest_controller_cmd_->drive.speed > 0.01);

    // -------------------- Decision --------------------
    ackermann_msgs::msg::AckermannDriveStamped output_cmd;

    if (sufficient_boundaries && front_clear && region_clear &&
        controller_cmd_valid && controller_fresh)
    {
      output_cmd = *latest_controller_cmd_;
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
        "Normal mode: controller command used.");
    }
    else if (latest_safety_cmd_ && safety_fresh)
    {
      output_cmd = *latest_safety_cmd_;
      std::string reason;
      if (!sufficient_boundaries) reason = "insufficient boundaries";
      else if (!front_clear) reason = "obstacle in front";
      else if (!region_clear) reason = "obstacle between boundaries";
      else if (!controller_fresh) reason = "controller timeout";
      else reason = "controller invalid";

      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Fallback: using safety bubble (reason: %s).", reason.c_str());
    }
    else
    {
      output_cmd.header.stamp = now();
      output_cmd.drive.speed = 0.0;
      output_cmd.drive.steering_angle = 0.0;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "No valid command -> STOP.");
    }

    drive_pub_->publish(output_cmd);
  }

  // -------------------- Members --------------------
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr controller_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr safety_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr left_boundary_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr right_boundary_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr arbitration_timer_;

  ackermann_msgs::msg::AckermannDriveStamped::SharedPtr latest_controller_cmd_;
  ackermann_msgs::msg::AckermannDriveStamped::SharedPtr latest_safety_cmd_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  nav_msgs::msg::Path::SharedPtr latest_left_boundary_;
  nav_msgs::msg::Path::SharedPtr latest_right_boundary_;

  rclcpp::Time last_scan_time_{0,0,RCL_ROS_TIME};
  rclcpp::Time last_boundary_left_time_{0,0,RCL_ROS_TIME};
  rclcpp::Time last_boundary_right_time_{0,0,RCL_ROS_TIME};
  rclcpp::Time last_controller_time_{0,0,RCL_ROS_TIME};
  rclcpp::Time last_safety_time_{0,0,RCL_ROS_TIME};

  std::string controller_cmd_topic_, safety_cmd_topic_, output_cmd_topic_, scan_topic_, left_boundary_topic_, right_boundary_topic_;
  bool enable_arbitration_;
  double arbitration_hz_;
  int min_boundary_points_, required_boundaries_;
  double scan_timeout_s_, boundary_timeout_s_, controller_timeout_s_, safety_timeout_s_;
  double obstacle_stop_distance_, front_cone_angle_deg_;
  double between_region_distance_;
  int between_region_min_points_;
  double scanner_origin_x_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveArbitratorNode>());
  rclcpp::shutdown();
  return 0;
}
