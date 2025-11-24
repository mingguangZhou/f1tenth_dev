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
    float dx = cur.x - last.x;
    float dy = cur.y - last.y;
    if (dx*dx + dy*dy < thresh * thresh) {
      r.start = i; last = cur;
    } else break;
  }
  last = scan_idx_to_xy(scan, idx);
  for (size_t i = idx + 1; i < scan->ranges.size(); ++i) {
    if (!is_valid_range(scan->ranges[i])) continue;
    Vec2f cur = scan_idx_to_xy(scan, i);
    float dx = cur.x - last.x;
    float dy = cur.y - last.y;
    if (dx*dx + dy*dy < thresh * thresh) {
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
  const size_t comp_start = left_boundary_range->start;
  const size_t comp_end = right_boundary_range->end;

  // Precompute validity and XY for the region once to avoid repeated trig
  std::vector<bool> valid(scan->ranges.size(), false);
  std::vector<Vec2f> xy(scan->ranges.size());
  for (size_t i = comp_start; i <= comp_end; ++i) {
    valid[i] = is_valid_range(scan->ranges[i]);
    if (valid[i]) {
      xy[i] = scan_idx_to_xy(scan, i);
    }
  }

  // Collect valid indices in the mid region [left.end, right.start]
  std::vector<size_t> mid_valid;
  for (size_t i = left_boundary_range->end; i <= right_boundary_range->start; ++i) {
    if (valid[i]) {
      mid_valid.push_back(i);
    }
  }
  if (mid_valid.size() < 2) {
    return;
  }

  float best_gap = -1.0f;
  size_t best_left = mid_valid.front();
  size_t best_right = mid_valid.back();
  std::unordered_map<uint64_t, float> dist_cache;
  dist_cache.reserve(mid_valid.size());

  // Evaluate each split between consecutive valid indices in the mid region
  for (size_t k = 0; k + 1 < mid_valid.size(); ++k) {
    size_t left_end = mid_valid[k];
    size_t right_start = mid_valid[k + 1];
    float gap = min_cross_distance(
      valid, xy,
      left_boundary_range->start, left_end,
      right_start, right_boundary_range->end,
      dist_cache,
      best_gap);
    if (gap > best_gap) {
      best_gap = gap;
      best_left = left_end;
      best_right = right_start;
    }
  }

  left_boundary_range->end = best_left;
  right_boundary_range->start = best_right;
}

static inline uint64_t pack_pair_key(uint32_t a, uint32_t b) {
  return (static_cast<uint64_t>(a) << 32) | static_cast<uint64_t>(b);
}

float BoundaryDetector::min_cross_distance(
  const std::vector<bool> &valid,
  const std::vector<Vec2f> &xy,
  size_t left_start, size_t left_end,
  size_t right_start, size_t right_end,
  std::unordered_map<uint64_t, float> &dist_cache,
  float current_best_gap) const
{
  float min_d = std::numeric_limits<float>::infinity();

  // Only consider up to K points nearest the gap on each side
  static constexpr size_t K_NEAR = 20;
  std::vector<size_t> left_indices;
  std::vector<size_t> right_indices;
  left_indices.reserve(K_NEAR);
  right_indices.reserve(K_NEAR);

  // Collect last K valid indices on the left side (scan backward)
  size_t collected = 0;
  for (size_t i = left_end + 1; i-- > left_start; ) {
    if (valid[i]) {
      left_indices.push_back(i);
      if (++collected >= K_NEAR) break;
    }
    if (i == left_start) break; // guard against wrap due to size_t
  }

  // Collect first K valid indices on the right side (scan forward)
  collected = 0;
  for (size_t j = right_start; j <= right_end; ++j) {
    if (valid[j]) {
      right_indices.push_back(j);
      if (++collected >= K_NEAR) break;
    }
  }

  if (left_indices.empty() || right_indices.empty()) return 0.0f;

  // Evaluate distances only among the near-gap sets
  for (size_t ii = 0; ii < left_indices.size(); ++ii) {
    size_t i = left_indices[ii];
    const Vec2f &pi = xy[i];
    for (size_t jj = 0; jj < right_indices.size(); ++jj) {
      size_t j = right_indices[jj];
      const Vec2f &pj = xy[j];
      uint64_t key = pack_pair_key(static_cast<uint32_t>(i), static_cast<uint32_t>(j));
      auto it = dist_cache.find(key);
      float d;
      if (it != dist_cache.end()) {
        d = it->second;
      } else {
        float dx = pi.x - pj.x;
        float dy = pi.y - pj.y;
        d = dx*dx + dy*dy; // squared distance
        dist_cache.emplace(key, d);
      }
      if (d < min_d) {
        min_d = d;
        if (min_d <= 0.0f) {
          return 0.0f; // cannot get smaller
        }
        if (min_d <= current_best_gap) {
          return min_d; // prune: cannot beat current best
        }
      }
    }
  }

  return min_d;
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