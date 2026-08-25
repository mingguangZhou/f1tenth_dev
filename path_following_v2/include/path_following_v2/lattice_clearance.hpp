#ifndef PATH_FOLLOWING_V2__LATTICE_CLEARANCE_HPP_
#define PATH_FOLLOWING_V2__LATTICE_CLEARANCE_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

namespace path_following_v2
{
namespace lattice_clearance
{

// Refine a map-derived clearance with connected LiDAR evidence. Distances
// strictly beyond the preferred-clearance radius cannot affect either lattice
// feasibility or node cost, so they are rejected by an axis-aligned lower
// bound before the exact hypot calculation. Once a collision is proven, the
// precise smaller distance is likewise irrelevant because the sample is
// unreachable.
template<typename PointRange>
double refineWithTrustedPoints(
  const double sample_x, const double sample_y,
  const double initial_clearance, const double preferred_clearance,
  const double collision_clearance, const PointRange & trusted_points)
{
  double clearance = initial_clearance;
  if (std::isnan(clearance) ||
    clearance == -std::numeric_limits<double>::infinity() ||
    (std::isfinite(clearance) && clearance <= collision_clearance))
  {
    return clearance;
  }

  for (const auto & point : trusted_points) {
    const double relevant_radius = std::min(preferred_clearance, clearance);
    const double dx = sample_x - point.x;
    if (std::abs(dx) > relevant_radius) {
      continue;
    }
    const double dy = sample_y - point.y;
    if (std::abs(dy) > relevant_radius) {
      continue;
    }
    const double point_clearance = std::hypot(dx, dy);
    clearance = std::min(clearance, point_clearance);
    if (clearance <= collision_clearance) {
      break;
    }
  }
  return clearance;
}

}  // namespace lattice_clearance
}  // namespace path_following_v2

#endif  // PATH_FOLLOWING_V2__LATTICE_CLEARANCE_HPP_
