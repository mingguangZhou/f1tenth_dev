#pragma once

#include <algorithm>
#include <cmath>

namespace path_following_v2::maneuver_speed
{

inline double curvatureLimitedSpeed(
  const double configured_ceiling_mps,
  const double command_ceiling_mps,
  const double lateral_acceleration_limit_mps2,
  const double maximum_curvature_inv_m)
{
  const double ceiling = std::clamp(
    configured_ceiling_mps, 0.0, std::max(0.0, command_ceiling_mps));
  if (!std::isfinite(maximum_curvature_inv_m) || maximum_curvature_inv_m <= 1e-6) {
    return ceiling;
  }
  const double curvature_cap = std::sqrt(
    std::max(0.0, lateral_acceleration_limit_mps2) / maximum_curvature_inv_m);
  return std::min(ceiling, curvature_cap);
}

}  // namespace path_following_v2::maneuver_speed
