#include <algorithm>
#include <cmath>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <nav_msgs/msg/path.hpp>

struct Vec2f {
  float x;
  float y;
};

struct Boundaries {
  std::vector<Vec2f> left;
  std::vector<Vec2f> right;
};

struct IndexRange {
  size_t start;
  size_t end;
};

class BoundaryDetectionNode : public rclcpp::Node {
 public:
  BoundaryDetectionNode() : Node("boundary_detection_node") {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&BoundaryDetectionNode::scan_callback, this,
                  std::placeholders::_1));
    boundary_markerarray_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/boundary_markers", 10);
    left_boundary_pub_ = this->create_publisher<nav_msgs::msg::Path>("left_boundary", 10);
    right_boundary_pub_ = this->create_publisher<nav_msgs::msg::Path>("right_boundary", 10);
    
    RCLCPP_INFO(this->get_logger(),
                "Boundary Detection Node has been started.");
  }

 private:
  static constexpr float INVALID_DISTANCE = 10.0f;
  static constexpr float MIN_VALID_DISTANCE = 0.05f;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Pre-clean invalid values
    auto cleaned_msg = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
    preprocess_scan(*cleaned_msg);

    Boundaries boundaries = detect_boundaries(cleaned_msg);
    publish_boundary_markers(boundaries, msg->header);
    publish_boundary_paths(boundaries, msg->header);
  }

  void preprocess_scan(sensor_msgs::msg::LaserScan &scan) {
    for (auto &val : scan.ranges) {
      if (std::isnan(val)) {
        val = 0.0f;
      } else if (std::isinf(val)) {
        val = INVALID_DISTANCE;
      }
    }
  }

  bool is_valid_range(float r) const {
    return r > MIN_VALID_DISTANCE && r < INVALID_DISTANCE;
  }

  void publish_boundary_markers(const Boundaries &boundaries,
                                const std_msgs::msg::Header &header) {
    visualization_msgs::msg::Marker left_marker;
    left_marker.header = header;
    left_marker.ns = "boundaries";
    left_marker.id = 0;
    left_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    left_marker.action = visualization_msgs::msg::Marker::ADD;
    left_marker.scale.x = 0.05;
    left_marker.color.r = 1.0;
    left_marker.color.g = 0.0;
    left_marker.color.b = 0.0;
    left_marker.color.a = 1.0;
    left_marker.points.reserve(boundaries.left.size());
    for (const auto &pt : boundaries.left) {
      geometry_msgs::msg::Point p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = 0.0;
      left_marker.points.push_back(p);
    }

    visualization_msgs::msg::Marker right_marker = left_marker;
    right_marker.id = 1;
    right_marker.color.r = 0.0;
    right_marker.color.g = 0.0;
    right_marker.color.b = 1.0;
    right_marker.points.clear();
    right_marker.points.reserve(boundaries.right.size());
    for (const auto &pt : boundaries.right) {
      geometry_msgs::msg::Point p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = 0.0;
      right_marker.points.push_back(p);
    }

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(left_marker);
    marker_array.markers.push_back(right_marker);
    boundary_markerarray_pub_->publish(marker_array);
  }

  void publish_boundary_paths(const Boundaries &boundaries,
                            const std_msgs::msg::Header &header) {
    nav_msgs::msg::Path left_path;
    nav_msgs::msg::Path right_path;
    left_path.header = header;
    right_path.header = header;

    for (const auto &pt : boundaries.left) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = header;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        pose.pose.orientation.w = 1.0;
        left_path.poses.push_back(pose);
    }

    for (const auto &pt : boundaries.right) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = header;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        pose.pose.orientation.w = 1.0;
        right_path.poses.push_back(pose);
    }

    left_boundary_pub_->publish(left_path);
    right_boundary_pub_->publish(right_path);
  }


  IndexRange grow(const sensor_msgs::msg::LaserScan::SharedPtr &msg,
                  size_t index, float threshold) {
    IndexRange range;
    range.start = index;
    range.end = index;

    auto calculate_point = [&](size_t idx) {
      float angle = msg->angle_min + idx * msg->angle_increment;
      float x = msg->ranges[idx] * std::cos(angle);
      float y = msg->ranges[idx] * std::sin(angle);
      return Vec2f{x, y};
    };

    if (!is_valid_range(msg->ranges[index])) {
      return range;  // Invalid seed point
    }

    Vec2f last_point = calculate_point(index);

    // Expand backwards
    for (size_t i = index; i > 0; --i) {
      float r = msg->ranges[i - 1];
      if (!is_valid_range(r)) continue;

      Vec2f current_point = calculate_point(i - 1);
      float distance = std::hypot(current_point.x - last_point.x,
                                  current_point.y - last_point.y);
      if (distance < threshold) {
        range.start = i - 1;
        last_point = current_point;
      } else {
        break;
      }
    }

    last_point = calculate_point(index);

    // Expand forwards
    for (size_t i = index; i < msg->ranges.size() - 1; ++i) {
      float r = msg->ranges[i + 1];
      if (!is_valid_range(r)) continue;

      Vec2f current_point = calculate_point(i + 1);
      float distance = std::hypot(current_point.x - last_point.x,
                                  current_point.y - last_point.y);
      if (distance < threshold) {
        range.end = i + 1;
        last_point = current_point;
      } else {
        break;
      }
    }

    return range;
  }

  Boundaries detect_boundaries(
      const sensor_msgs::msg::LaserScan::SharedPtr &msg) {
    Boundaries boundaries;
    float angle_min = msg->angle_min;
    float angle_increment = msg->angle_increment;
    const float distance_threshold =
        // std::min(msg->range_max * msg->angle_increment * 2, 0.5f);
        std::min(msg->range_max * msg->angle_increment * 6, 0.5f); // temporarily make it big for simulation

    size_t left_target_index =
        static_cast<size_t>((-M_PI_2 - angle_min) / angle_increment);
    size_t right_target_index =
        static_cast<size_t>((M_PI_2 - angle_min) / angle_increment);
    left_target_index = std::min(left_target_index, msg->ranges.size() - 1);
    right_target_index = std::min(right_target_index, msg->ranges.size() - 1);

    IndexRange left_range = grow(msg, left_target_index, distance_threshold);
    for (size_t i = left_range.start; i <= left_range.end; ++i) {
      float angle = angle_min + i * angle_increment;
      float r = msg->ranges[i];
      if (is_valid_range(r)) {
        boundaries.left.push_back(Vec2f{r * std::cos(angle),
                                        r * std::sin(angle)});
      }
    }

    IndexRange right_range = grow(msg, right_target_index, distance_threshold);
    for (size_t i = right_range.start; i <= right_range.end; ++i) {
      float angle = angle_min + i * angle_increment;
      float r = msg->ranges[i];
      if (is_valid_range(r)) {
        boundaries.right.push_back(Vec2f{r * std::cos(angle),
                                         r * std::sin(angle)});
      }
    }

    return boundaries;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      boundary_markerarray_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr left_boundary_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr right_boundary_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoundaryDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
