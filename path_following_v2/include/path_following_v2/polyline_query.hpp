#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace path_following_v2::polyline_query
{

constexpr double kMinimumProjectableLengthSquared = 1e-12;

struct Segment
{
  double ax{0.0};
  double ay{0.0};
  double dx{0.0};
  double dy{0.0};
  double length_squared{0.0};
  double length{0.0};
  double yaw{0.0};
  double min_x{0.0};
  double max_x{0.0};
  double min_y{0.0};
  double max_y{0.0};
  bool projectable{false};
};

struct NearestPoint
{
  bool valid{false};
  double distance{std::numeric_limits<double>::infinity()};
  double ratio{0.0};
  double residual_x{0.0};
  double residual_y{0.0};
  std::size_t segment_index{0};
  std::size_t exact_distance_evaluations{0};
};

template<typename PointRange>
std::vector<Segment> prepareSegments(const PointRange & points, const bool closed = false)
{
  std::vector<Segment> segments;
  if (points.size() < 2) {
    return segments;
  }
  const std::size_t segment_count = closed ? points.size() : points.size() - 1;
  segments.reserve(segment_count);
  for (std::size_t index = 0; index < segment_count; ++index) {
    const auto & first = points[index];
    const auto & second = points[(index + 1) % points.size()];
    Segment segment;
    segment.ax = first.x;
    segment.ay = first.y;
    segment.dx = second.x - first.x;
    segment.dy = second.y - first.y;
    segment.length_squared =
      segment.dx * segment.dx + segment.dy * segment.dy;
    segment.projectable =
      segment.length_squared > kMinimumProjectableLengthSquared;
    if (segment.projectable) {
      segment.length = std::sqrt(segment.length_squared);
      segment.yaw = std::atan2(segment.dy, segment.dx);
    }

    // Expand every bound by one representable value. This makes the AABB a
    // conservative enclosure even at floating-point endpoints; pruning is
    // also strict, so equality always reaches the original exact formula.
    segment.min_x = std::nextafter(
      std::min(first.x, second.x), -std::numeric_limits<double>::infinity());
    segment.max_x = std::nextafter(
      std::max(first.x, second.x), std::numeric_limits<double>::infinity());
    segment.min_y = std::nextafter(
      std::min(first.y, second.y), -std::numeric_limits<double>::infinity());
    segment.max_y = std::nextafter(
      std::max(first.y, second.y), std::numeric_limits<double>::infinity());
    segments.push_back(segment);
  }
  return segments;
}

inline double aabbDistanceSquared(
  const double x, const double y, const Segment & segment)
{
  const double delta_x = x < segment.min_x ? segment.min_x - x :
    (x > segment.max_x ? x - segment.max_x : 0.0);
  const double delta_y = y < segment.min_y ? segment.min_y - y :
    (y > segment.max_y ? y - segment.max_y : 0.0);
  return delta_x * delta_x + delta_y * delta_y;
}

inline bool lowerBoundExceedsDistance(
  const double lower_bound_squared, const double distance)
{
  if (!std::isfinite(distance)) {
    return false;
  }
  const double distance_squared = distance * distance;
  if (!std::isfinite(distance_squared)) {
    return false;
  }
  // Allow several ULPs around the squared comparison. The exact hypot-based
  // calculation below still resolves all close calls and original tie order.
  constexpr double tolerance_multiplier =
    16.0 * std::numeric_limits<double>::epsilon();
  const double tolerance = tolerance_multiplier * std::max(1.0, distance_squared);
  return lower_bound_squared > distance_squared + tolerance;
}

inline void exactClosestPoint(
  const double x, const double y, const Segment & segment,
  double & ratio, double & residual_x, double & residual_y, double & distance)
{
  ratio = segment.projectable ? std::clamp(
    ((x - segment.ax) * segment.dx + (y - segment.ay) * segment.dy) /
    segment.length_squared, 0.0, 1.0) : 0.0;
  residual_x = x - (segment.ax + ratio * segment.dx);
  residual_y = y - (segment.ay + ratio * segment.dy);
  distance = std::hypot(residual_x, residual_y);
}

inline NearestPoint nearestPoint(
  const double x, const double y, const std::vector<Segment> & segments,
  const std::size_t begin = 0,
  const std::size_t requested_end = std::numeric_limits<std::size_t>::max(),
  const bool skip_unprojectable = false)
{
  NearestPoint best;
  const std::size_t end = std::min(requested_end, segments.size());
  if (begin >= end) {
    return best;
  }
  for (std::size_t index = begin; index < end; ++index) {
    const auto & segment = segments[index];
    if (skip_unprojectable && !segment.projectable) {
      continue;
    }
    if (lowerBoundExceedsDistance(
        aabbDistanceSquared(x, y, segment), best.distance))
    {
      continue;
    }
    double ratio = 0.0;
    double residual_x = 0.0;
    double residual_y = 0.0;
    double distance = std::numeric_limits<double>::infinity();
    exactClosestPoint(
      x, y, segment, ratio, residual_x, residual_y, distance);
    ++best.exact_distance_evaluations;
    // Strict comparison retains the exhaustive implementation's earliest
    // segment on exact ties.
    if (distance < best.distance) {
      best.valid = true;
      best.distance = distance;
      best.ratio = ratio;
      best.residual_x = residual_x;
      best.residual_y = residual_y;
      best.segment_index = index;
    }
  }
  return best;
}

inline bool anyWithinDistance(
  const double x, const double y, const double threshold,
  const std::vector<Segment> & segments,
  const std::size_t begin = 0,
  const std::size_t requested_end = std::numeric_limits<std::size_t>::max(),
  std::size_t * exact_distance_evaluations = nullptr)
{
  if (threshold < 0.0 || std::isnan(threshold)) {
    return false;
  }
  const std::size_t end = std::min(requested_end, segments.size());
  if (begin >= end) {
    return false;
  }
  for (std::size_t index = begin; index < end; ++index) {
    const auto & segment = segments[index];
    if (lowerBoundExceedsDistance(
        aabbDistanceSquared(x, y, segment), threshold))
    {
      continue;
    }
    double ratio = 0.0;
    double residual_x = 0.0;
    double residual_y = 0.0;
    double distance = std::numeric_limits<double>::infinity();
    exactClosestPoint(
      x, y, segment, ratio, residual_x, residual_y, distance);
    if (exact_distance_evaluations != nullptr) {
      ++(*exact_distance_evaluations);
    }
    // Keep the original hypot and inclusive threshold comparison. Squared
    // distance is intentionally used only by the conservative AABB.
    if (distance <= threshold) {
      return true;
    }
  }
  return false;
}

}  // namespace path_following_v2::polyline_query
