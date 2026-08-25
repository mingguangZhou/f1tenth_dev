#ifndef PATH_FOLLOWING_V2__PATH_SPLICE_HPP_
#define PATH_FOLLOWING_V2__PATH_SPLICE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace path_following_v2
{
namespace path_splice
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

struct Match
{
  bool valid{false};
  std::size_t raw_start_index{0};
  double connection_distance_m{std::numeric_limits<double>::infinity()};
  double maximum_heading_error_rad{std::numeric_limits<double>::infinity()};
  double maximum_curvature_inv_m{std::numeric_limits<double>::infinity()};
};

enum class ActivePathHandoffMode
{
  STRICT_SPLICE,
  FRESH_RAW_PATH,
};

inline ActivePathHandoffMode activePathHandoffMode(
  const bool handoff_started, const bool rejoin_geometry_reached,
  const std::size_t progress_index, const std::size_t rejoin_index)
{
  return handoff_started ||
    (rejoin_geometry_reached && progress_index > rejoin_index) ?
    ActivePathHandoffMode::FRESH_RAW_PATH :
    ActivePathHandoffMode::STRICT_SPLICE;
}

inline bool storedPathEndedWithoutRejoin(
  const bool handoff_started, const bool stored_path_near_end,
  const bool rejoin_geometry_reached)
{
  return !handoff_started && stored_path_near_end && !rejoin_geometry_reached;
}

inline double distance(const Point2 & first, const Point2 & second)
{
  return std::hypot(second.x - first.x, second.y - first.y);
}

inline double heading(const Point2 & first, const Point2 & second)
{
  return std::atan2(second.y - first.y, second.x - first.x);
}

inline double headingError(const double first, const double second)
{
  return std::abs(std::atan2(std::sin(first - second), std::cos(first - second)));
}

inline double threePointCurvature(
  const Point2 & first, const Point2 & middle, const Point2 & last)
{
  const double a = distance(first, middle);
  const double b = distance(middle, last);
  const double c = distance(first, last);
  if (a <= 1e-6 || b <= 1e-6 || c <= 1e-6) {
    return 0.0;
  }
  const double cross = std::abs(
    (middle.x - first.x) * (last.y - first.y) -
    (middle.y - first.y) * (last.x - first.x));
  return 2.0 * cross / (a * b * c);
}

inline Match findContinuousTailStart(
  const Point2 & previous, const Point2 & anchor,
  const std::vector<Point2> & raw_path,
  const double maximum_gap_m, const double maximum_heading_error_rad,
  const double maximum_curvature_inv_m)
{
  Match best;
  if (distance(previous, anchor) <= 1e-6 || raw_path.size() < 2) {
    return best;
  }

  const double incoming_heading = heading(previous, anchor);
  for (std::size_t raw_index = 0; raw_index + 1 < raw_path.size(); ++raw_index) {
    std::size_t candidate_index = raw_index;
    if (distance(anchor, raw_path[candidate_index]) <= 1e-6) {
      ++candidate_index;
    }
    if (candidate_index + 1 >= raw_path.size()) {
      continue;
    }

    const double connection_distance = distance(anchor, raw_path[candidate_index]);
    if (connection_distance <= 1e-6 || connection_distance > maximum_gap_m) {
      continue;
    }
    const double connection_heading = heading(anchor, raw_path[candidate_index]);
    const double outgoing_heading = heading(
      raw_path[candidate_index], raw_path[candidate_index + 1]);
    const double heading_error = std::max(
      headingError(incoming_heading, connection_heading),
      headingError(connection_heading, outgoing_heading));
    if (heading_error > maximum_heading_error_rad) {
      continue;
    }

    const double curvature = std::max(
      threePointCurvature(previous, anchor, raw_path[candidate_index]),
      threePointCurvature(anchor, raw_path[candidate_index], raw_path[candidate_index + 1]));
    if (!std::isfinite(curvature) || curvature > maximum_curvature_inv_m) {
      continue;
    }

    if (!best.valid || connection_distance < best.connection_distance_m) {
      best.valid = true;
      best.raw_start_index = candidate_index;
      best.connection_distance_m = connection_distance;
      best.maximum_heading_error_rad = heading_error;
      best.maximum_curvature_inv_m = curvature;
    }
  }
  return best;
}

}  // namespace path_splice
}  // namespace path_following_v2

#endif  // PATH_FOLLOWING_V2__PATH_SPLICE_HPP_
