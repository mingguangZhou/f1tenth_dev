#include "boundary_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

bool BoundaryDetector::is_valid_range(float r) const {
  return r > MIN_VALID_DISTANCE && r < INVALID_DISTANCE;
}

Vec2f BoundaryDetector::scan_idx_to_xy(const sensor_msgs::msg::LaserScan::SharedPtr &scan, size_t idx) {
  float a = scan->angle_min + static_cast<float>(idx) * scan->angle_increment;
  float r = scan->ranges[idx];
  return Vec2f{r * std::cos(a), r * std::sin(a)};
}

size_t BoundaryDetector::find_smallest_range_around(const sensor_msgs::msg::LaserScan::SharedPtr &scan, size_t idx) {
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

BoundaryDetector::IndexRange BoundaryDetector::grow(
  const sensor_msgs::msg::LaserScan::SharedPtr &scan, size_t idx, float thresh) 
{
  IndexRange r{idx, idx};
  if (!is_valid_range(scan->ranges[idx])) return r;
  Vec2f last = scan_idx_to_xy(scan, idx);
  for (size_t i = idx; i-- > 0;) {
    if (!is_valid_range(scan->ranges[i])) continue;
    Vec2f cur = scan_idx_to_xy(scan, i);
    if (std::hypot(cur.x - last.x, cur.y - last.y) < thresh) {
      r.start = i; last = cur;
    } else break;
  }
  last = scan_idx_to_xy(scan, idx);
  for (size_t i = idx + 1; i < scan->ranges.size(); ++i) {
    if (!is_valid_range(scan->ranges[i])) continue;
    Vec2f cur = scan_idx_to_xy(scan, i);
    if (std::hypot(cur.x - last.x, cur.y - last.y) < thresh) {
      r.end = i; last = cur;
    } else break;
  }
  return r;
}

void BoundaryDetector::grow_front_boundaries_to_largest_gap(
  const sensor_msgs::msg::LaserScan::SharedPtr &scan,
  IndexRange *left_boundary_range,
  IndexRange *right_boundary_range) 
{
  std::vector<size_t> valid_indices;
  for (size_t i = left_boundary_range->end; i <= right_boundary_range->start; ++i) {
    if (is_valid_range(scan->ranges[i])) {
      valid_indices.push_back(i);
    }
  }
  if (valid_indices.size() < 2) {
    return;
  }

  float max_gap = 0.0f;
  size_t best_left = valid_indices.front();
  size_t best_right = valid_indices.back();
  for (size_t k = 0; k + 1 < valid_indices.size(); ++k) {
    size_t i = valid_indices[k];
    size_t j = valid_indices[k + 1];
    Vec2f pi = scan_idx_to_xy(scan, i);
    Vec2f pj = scan_idx_to_xy(scan, j);
    float d = std::hypot(pi.x - pj.x, pi.y - pj.y);
    if (d > max_gap) {
      max_gap = d;
      best_left = i;
      best_right = j;
    }
  }

  left_boundary_range->end = best_left;
  right_boundary_range->start = best_right;
}

namespace {
// Ensure boundary points are ordered with increasing x (forward s)
inline void enforce_forward_s(std::vector<Vec2f>* pts) {
  if (pts->size() < 2) return;
  if (pts->front().x > pts->back().x) {
    std::reverse(pts->begin(), pts->end());
  }
}
}  // namespace

Boundaries BoundaryDetector::detect_boundaries(
  const sensor_msgs::msg::LaserScan::SharedPtr &scan,
  const geometry_msgs::msg::TransformStamped &tf) 
{
  float amin = scan->angle_min;
  float ainc = scan->angle_increment;
  float thresh = std::min(scan->range_max * ainc * 6.0f, 0.5f);

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
  grow_front_boundaries_to_largest_gap(scan, &Lr, &Rr);
  size_t nearest_left = Lr.start, nearest_right = Rr.start;
  float min_left_dist = std::numeric_limits<float>::max();
  float min_right_dist = std::numeric_limits<float>::max();

  b.left.clear();
  for (size_t i = Lr.start; i <= Lr.end; ++i) {
    float r = scan->ranges[i];
    if (is_valid_range(r)) {
      float a = amin + i * ainc;
      Vec2f pt{ r * std::cos(a), r * std::sin(a) };

      geometry_msgs::msg::PointStamped in, out;
      in.header = scan->header;
      in.point.x = pt.x;
      in.point.y = pt.y;
      in.point.z = 0.0;
      tf2::doTransform(in, out, tf);

      b.left.push_back({static_cast<float>(out.point.x), static_cast<float>(out.point.y)});

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

      geometry_msgs::msg::PointStamped in, out;
      in.header = scan->header;
      in.point.x = pt.x;
      in.point.y = pt.y;
      in.point.z = 0.0;
      tf2::doTransform(in, out, tf);

      b.right.push_back({static_cast<float>(out.point.x), static_cast<float>(out.point.y)});

      if (r < min_right_dist) {
        min_right_dist = r;
        nearest_right = i;
      }
    }
  }

  // Enforce forward ordering here instead of the node
  enforce_forward_s(&b.left);
  enforce_forward_s(&b.right);

  state_ = State{nearest_left, nearest_right};
  return b;
}