#include <algorithm>
#include <cmath>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
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

class BoundaryDetectionNode : public rclcpp::Node {
 public:
  BoundaryDetectionNode()
      : Node("boundary_detection_node") {
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
  // vehicle specs
  float laser_offset_x_ = 0.27;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      boundary_markerarray_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr left_boundary_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr right_boundary_pub_;

  struct State {
    size_t nearest_left_scan_idx = 0;
    size_t nearest_right_scan_idx = 0;
  };

  std::optional<State> state_;

  void scan_callback(
      const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Copy and clean up the scan
    auto cleaned_msg = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
    preprocess_scan(*cleaned_msg);

    // Detect raw boundaries
    Boundaries b = detect_boundaries(cleaned_msg);

    // --- Enforce both lists to run in increasing +X order ---
    auto enforce_forward_s = [&](std::vector<Vec2f> &pts) {
      if (pts.size() < 2) return;
      if (pts.front().x > pts.back().x) {
        std::reverse(pts.begin(), pts.end());
      }
    };
    enforce_forward_s(b.left);
    enforce_forward_s(b.right);
    // --------------------------------------------------------

    // Publish as markers and as Path messages
    publish_boundary_markers(b, msg->header);
    publish_boundary_paths(b, msg->header);
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

  bool is_valid_range(float r) const {
    return r > MIN_VALID_DISTANCE && r < INVALID_DISTANCE;
  }

  void publish_boundary_markers(
      const Boundaries &boundaries,
      const std_msgs::msg::Header &header) {
    visualization_msgs::msg::Marker left_m;
    left_m.header = header;
    left_m.ns = "boundaries";
    left_m.id = 0;
    left_m.type = left_m.LINE_STRIP;
    left_m.action = left_m.ADD;
    left_m.scale.x = 0.05f;
    left_m.color.r = 1.0f;
    left_m.color.a = 1.0f;
    left_m.points.reserve(boundaries.left.size());
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
    right_m.points.reserve(boundaries.right.size());
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

  void publish_boundary_paths(
      const Boundaries &boundaries,
      const std_msgs::msg::Header &header) {
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

  struct IndexRange { size_t start, end; };

  size_t find_smallest_range_around(
      const sensor_msgs::msg::LaserScan::SharedPtr &scan,
      size_t idx) {
    size_t start = (idx >= 10) ? idx - 10 : 0;
    size_t end = std::min(idx + 10, scan->ranges.size() - 1);
    float min_range = std::numeric_limits<float>::max();
    size_t min_idx = idx;
    for (size_t i = start; i <= end; ++i) {
      float r = scan->ranges[i];
      if (is_valid_range(r) && r < min_range) {
        min_range = r;
        min_idx = i;
      }
    }
    return min_idx;
  }

  IndexRange grow(
      const sensor_msgs::msg::LaserScan::SharedPtr &scan,
      size_t idx, float thresh) {
    IndexRange r{idx, idx};
    if (!is_valid_range(scan->ranges[idx])) {
      return r;
    }
    auto calc = [&](size_t i) {
      float a = scan->angle_min + i * scan->angle_increment;
      return Vec2f{
        scan->ranges[i] * std::cos(a) + laser_offset_x_, // add x offset, so the origin is united to the rear axis
        scan->ranges[i] * std::sin(a)
      };
    };
    Vec2f last = calc(idx);
    // backward
    for (size_t i = idx; i-- > 0; ) {
      if (!is_valid_range(scan->ranges[i])) continue;
      Vec2f cur = calc(i);
      if (std::hypot(cur.x - last.x, cur.y - last.y) < thresh) {
        r.start = i; last = cur;
      } else break;
    }
    // forward
    last = calc(idx);
    for (size_t i = idx + 1; i < scan->ranges.size(); ++i) {
      if (!is_valid_range(scan->ranges[i])) continue;
      Vec2f cur = calc(i);
      if (std::hypot(cur.x - last.x, cur.y - last.y) < thresh) {
        r.end = i; last = cur;
      } else break;
    }
    return r;
  }

  Boundaries detect_boundaries(
      const sensor_msgs::msg::LaserScan::SharedPtr &scan) {
    float amin = scan->angle_min;
    float ainc = scan->angle_increment;
    float thresh = std::min(scan->range_max * ainc * 6, 0.5f);

    size_t li, ri;
    if (state_.has_value()) {
      li = find_smallest_range_around(scan, state_->nearest_left_scan_idx);
      ri = find_smallest_range_around(scan, state_->nearest_right_scan_idx);
    } else {
      li = std::min<size_t>(
        (static_cast<float>(-M_PI_2) - amin) / ainc,
        scan->ranges.size() - 1
      );
      ri = std::min<size_t>(
        (static_cast<float>( M_PI_2) - amin) / ainc,
        scan->ranges.size() - 1
      );
    }

    auto Lr = grow(scan, li, thresh);
    auto Rr = grow(scan, ri, thresh);
    Boundaries b;
    if (Lr.end >= Rr.start && Lr.start <= Rr.end) {
      state_ = std::nullopt;
      return Boundaries{};
    }
    // Find nearest points (smallest range) in both boundaries
    size_t nearest_left = Lr.start, nearest_right = Rr.start;
    float min_left_dist = std::numeric_limits<float>::max();
    float min_right_dist = std::numeric_limits<float>::max();

    b.left.clear();
    for (size_t i = Lr.start; i <= Lr.end; ++i) {
      float r = scan->ranges[i];
      if (is_valid_range(r)) {
      float a = amin + i * ainc;
      Vec2f pt{ r * std::cos(a), r * std::sin(a) };
      b.left.push_back(pt);
      if (r < min_left_dist) {
        min_left_dist = r;
        nearest_left = i;
      }
      }
    }

    b.right.clear();
    for (size_t i = Rr.start; i <= Rr.end; ++i) {
      float r = scan->ranges[i];
      if (is_valid_range(r)) {
      float a = amin + i * ainc;
      Vec2f pt{ r * std::cos(a), r * std::sin(a) };
      b.right.push_back(pt);
      if (r < min_right_dist) {
        min_right_dist = r;
        nearest_right = i;
      }
      }
    }

    state_ = State{nearest_left, nearest_right};

    return b;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoundaryDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
