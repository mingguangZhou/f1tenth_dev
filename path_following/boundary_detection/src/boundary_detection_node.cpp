#include <algorithm>
#include <cmath>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "boundary_detector.hpp"

class BoundaryDetectionNode : public rclcpp::Node {
 public:
  BoundaryDetectionNode()
      : Node("boundary_detection_node"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&BoundaryDetectionNode::scan_callback,
                  this, std::placeholders::_1));
    boundary_markerarray_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "/boundary_markers", 10);
    left_boundary_pub_ =
        create_publisher<nav_msgs::msg::Path>("left_boundary", 10);
    right_boundary_pub_ =
        create_publisher<nav_msgs::msg::Path>("right_boundary", 10);

    RCLCPP_INFO(get_logger(), "Boundary Detection Node has been started.");
  }

 private:
  static constexpr float INVALID_DISTANCE = 10.0f;
  static constexpr float MIN_VALID_DISTANCE = 0.05f;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boundary_markerarray_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr left_boundary_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr right_boundary_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string output_frame_id_ = "base_link";  // default

  BoundaryDetector detector_;  // moved detection logic/state out of this node

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped tf;
    if (msg->header.frame_id == "laser") {
      try {
        tf = tf_buffer_.lookupTransform("base_link", "laser", tf2::TimePointZero);
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        return;
      }
    } else if (msg->header.frame_id == "ego_racecar/laser") {
      // In sim, assume identity transform (already in correct base frame)
      tf.header.stamp = msg->header.stamp;
      tf.header.frame_id = "ego_racecar/base_link";
      tf.child_frame_id = "ego_racecar/laser";
      tf.transform.translation.x = 0.0;
      tf.transform.translation.y = 0.0;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation.x = 0.0;
      tf.transform.rotation.y = 0.0;
      tf.transform.rotation.z = 0.0;
      tf.transform.rotation.w = 1.0;
    } else {
      RCLCPP_WARN(this->get_logger(), "Unexpected frame_id '%s', assuming identity transform.", msg->header.frame_id.c_str());
      tf.header.stamp = msg->header.stamp;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = msg->header.frame_id;
      tf.transform.translation.x = 0.0;
      tf.transform.translation.y = 0.0;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation.x = 0.0;
      tf.transform.rotation.y = 0.0;
      tf.transform.rotation.z = 0.0;
      tf.transform.rotation.w = 1.0;
    }


    auto cleaned_msg = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
    preprocess_scan(*cleaned_msg);

    Boundaries b = detector_.detect_boundaries(cleaned_msg, tf);

    // Decide output frame ID based on input scan frame
    if (msg->header.frame_id == "ego_racecar/laser") {
      output_frame_id_ = "ego_racecar/base_link";
    } else if (msg->header.frame_id == "laser") {
      output_frame_id_ = "base_link";
    } else {
      RCLCPP_WARN(this->get_logger(), "Unexpected scan frame_id: %s, defaulting to base_link", msg->header.frame_id.c_str());
      output_frame_id_ = "base_link";
    }

    // Set header with updated frame
    std_msgs::msg::Header header = msg->header;
    header.frame_id = output_frame_id_;

    publish_boundary_markers(b, header);
    publish_boundary_paths(b, header);
  }

  void preprocess_scan(sensor_msgs::msg::LaserScan &scan) {
    for (auto &r : scan.ranges) {
      if (std::isnan(r)) {
        r = 0.0f;
      } else if (!std::isfinite(r)) {
        r = INVALID_DISTANCE;
      }
    }
  }

  void publish_boundary_markers(const Boundaries &boundaries, const std_msgs::msg::Header &header) {
    visualization_msgs::msg::Marker left_m;
    left_m.header = header;
    left_m.ns = "boundaries";
    left_m.id = 0;
    left_m.type = left_m.LINE_STRIP;
    left_m.action = left_m.ADD;
    left_m.scale.x = 0.05f;
    left_m.color.r = 1.0f;
    left_m.color.a = 1.0f;
    for (auto &pt : boundaries.left) {
      geometry_msgs::msg::Point p;
      p.x = pt.x; p.y = pt.y; p.z = 0.0;
      left_m.points.push_back(p);
    }

    visualization_msgs::msg::Marker right_m = left_m;
    right_m.id = 1;
    right_m.color.r = 0.0f;
    right_m.color.b = 1.0f;
    right_m.points.clear();
    for (auto &pt : boundaries.right) {
      geometry_msgs::msg::Point p;
      p.x = pt.x; p.y = pt.y; p.z = 0.0;
      right_m.points.push_back(p);
    }

    visualization_msgs::msg::MarkerArray ma;
    ma.markers.push_back(left_m);
    ma.markers.push_back(right_m);
    boundary_markerarray_pub_->publish(ma);
  }

  void publish_boundary_paths(const Boundaries &boundaries, const std_msgs::msg::Header &header) {
    nav_msgs::msg::Path left_path;
    nav_msgs::msg::Path right_path;
    left_path.header = header;
    right_path.header = header;

    for (auto &pt : boundaries.left) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = header;
      ps.pose.position.x = pt.x;
      ps.pose.position.y = pt.y;
      ps.pose.orientation.w = 1.0;
      left_path.poses.push_back(ps);
    }
    for (auto &pt : boundaries.right) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = header;
      ps.pose.position.x = pt.x;
      ps.pose.position.y = pt.y;
      ps.pose.orientation.w = 1.0;
      right_path.poses.push_back(ps);
    }

    left_boundary_pub_->publish(left_path);
    right_boundary_pub_->publish(right_path);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoundaryDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
