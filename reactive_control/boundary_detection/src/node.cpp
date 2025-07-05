#include <algorithm>
#include <cmath>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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
    RCLCPP_INFO(this->get_logger(),
                "Boundary Detection Node has been started.");
  }

 private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    Boundaries boundaries = detect_boundaries(msg);
    publish_boundary_markers(boundaries, msg->header);
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

    Vec2f last_point = calculate_point(index);

    // Expand backwards
    for (size_t i = index; i > 0; --i) {
      Vec2f current_point = calculate_point(i - 1);
      float distance = std::hypot(current_point.x - last_point.x,
                                  current_point.y - last_point.y);
      if (distance < threshold) {
        range.start = i - 1;
        last_point = current_point;  // Update last point to the new one
      } else {
        break;
      }
    }
    last_point = calculate_point(index);
    // Expand forwards
    for (size_t i = index; i < msg->ranges.size() - 1; ++i) {
      Vec2f current_point = calculate_point(i + 1);
      float distance = std::hypot(current_point.x - last_point.x,
                                  current_point.y - last_point.y);
      if (distance < threshold) {
        range.end = i + 1;
        last_point = current_point;  // Update last point to the new one
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
        std::min(msg->range_max * msg->angle_increment * 2, 0.5f);

    size_t left_target_index =
        static_cast<size_t>((-M_PI_2 - angle_min) / angle_increment);
    size_t right_target_index =
        static_cast<size_t>((M_PI_2 - angle_min) / angle_increment);
    // Ensure indices are within bounds
    left_target_index = std::min(left_target_index, msg->ranges.size() - 1);
    right_target_index = std::min(right_target_index, msg->ranges.size() - 1);

    // Get left boundary range and points
    IndexRange left_range = grow(msg, left_target_index, distance_threshold);
    for (size_t i = left_range.start; i <= left_range.end; ++i) {
      float angle = angle_min + i * angle_increment;
      float r = msg->ranges[i];
      if (std::isfinite(r) && r > msg->range_min && r < msg->range_max) {
        boundaries.left.push_back(
            Vec2f{r * std::cos(angle), r * std::sin(angle)});
      }
    }

    // Get right boundary range and points
    IndexRange right_range = grow(msg, right_target_index, distance_threshold);
    for (size_t i = right_range.start; i <= right_range.end; ++i) {
      float angle = angle_min + i * angle_increment;
      float r = msg->ranges[i];
      if (std::isfinite(r) && r > msg->range_min && r < msg->range_max) {
        boundaries.right.push_back(
            Vec2f{r * std::cos(angle), r * std::sin(angle)});
      }
    }
    return boundaries;
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      boundary_markerarray_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoundaryDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
