#ifndef PATH_FOLLOWING_V2__CANDIDATE_SELECTION_HPP_
#define PATH_FOLLOWING_V2__CANDIDATE_SELECTION_HPP_

#include <cmath>

namespace path_following_v2
{
namespace selection
{

enum class Choice
{
  NONE,
  LEFT,
  RIGHT
};

struct Metrics
{
  bool valid{false};
  double minimum_clearance{0.0};
  double objective_cost{0.0};
  double maximum_curvature{0.0};
  double peak_offset{0.0};
};

inline Choice chooseSaferCandidate(
  const Metrics & left, const Metrics & right,
  const double clearance_tie_m)
{
  if (left.valid != right.valid) {
    return left.valid ? Choice::LEFT : Choice::RIGHT;
  }
  if (!left.valid) {
    return Choice::NONE;
  }

  const bool left_clearance_finite = std::isfinite(left.minimum_clearance);
  const bool right_clearance_finite = std::isfinite(right.minimum_clearance);
  if (left_clearance_finite != right_clearance_finite) {
    // Infinite clearance means no scan return was close enough to limit the
    // passing section, so it is preferable to a finite bottleneck.
    return left_clearance_finite ? Choice::RIGHT : Choice::LEFT;
  }
  if (left_clearance_finite &&
    std::abs(left.minimum_clearance - right.minimum_clearance) > clearance_tie_m)
  {
    return left.minimum_clearance > right.minimum_clearance ?
      Choice::LEFT : Choice::RIGHT;
  }

  const bool left_cost_finite = std::isfinite(left.objective_cost);
  const bool right_cost_finite = std::isfinite(right.objective_cost);
  if (left_cost_finite != right_cost_finite) {
    return left_cost_finite ? Choice::LEFT : Choice::RIGHT;
  }
  if (left_cost_finite && std::abs(left.objective_cost - right.objective_cost) > 1e-9) {
    return left.objective_cost < right.objective_cost ? Choice::LEFT : Choice::RIGHT;
  }
  if (std::abs(left.maximum_curvature - right.maximum_curvature) > 1e-3) {
    return left.maximum_curvature < right.maximum_curvature ? Choice::LEFT : Choice::RIGHT;
  }
  return std::abs(left.peak_offset) <= std::abs(right.peak_offset) ?
    Choice::LEFT : Choice::RIGHT;
}

}  // namespace selection
}  // namespace path_following_v2

#endif  // PATH_FOLLOWING_V2__CANDIDATE_SELECTION_HPP_
