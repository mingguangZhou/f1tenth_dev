#pragma once

#include <vector>
#include <optional>
#include <cstddef>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

struct Vec2f {
  float x;
  float y;
};

struct Boundaries {
  std::vector<Vec2f> left;
  std::vector<Vec2f> right;
};

class BoundaryDetector {
 public:
  Boundaries detect_boundaries(
    const sensor_msgs::msg::LaserScan::SharedPtr &scan,
    const geometry_msgs::msg::TransformStamped &tf);

 private:
  static constexpr float INVALID_DISTANCE = 10.0f;
  static constexpr float MIN_VALID_DISTANCE = 0.05f;

  struct State {
    size_t nearest_left_scan_idx = 0;
    size_t nearest_right_scan_idx = 0;
  };
  std::optional<State> state_;

  struct IndexRange { size_t start, end; };

  static Vec2f scan_idx_to_xy(const sensor_msgs::msg::LaserScan::SharedPtr &scan, size_t idx);
  bool is_valid_range(float r) const;
  size_t find_smallest_range_around(const sensor_msgs::msg::LaserScan::SharedPtr &scan, size_t idx);
  IndexRange grow(const sensor_msgs::msg::LaserScan::SharedPtr &scan, size_t idx, float thresh);
  void grow_front_boundaries_to_largest_gap(const sensor_msgs::msg::LaserScan::SharedPtr &scan,
                                            IndexRange *left_boundary_range,
                                            IndexRange *right_boundary_range);
};