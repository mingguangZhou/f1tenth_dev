#ifndef PATH_FOLLOWING_V2__BOUNDED_CORRIDOR_SMOOTHER_HPP_
#define PATH_FOLLOWING_V2__BOUNDED_CORRIDOR_SMOOTHER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace path_following_v2
{
namespace corridor
{

struct Problem
{
  std::vector<double> s;
  std::vector<double> initial_offsets;
  std::vector<double> reference_offsets;
  std::vector<double> lower_offsets;
  std::vector<double> upper_offsets;
  std::vector<double> continuity_offsets;
};

struct Config
{
  int max_iterations{12};
  double relaxation{0.85};
  double reference_weight{1.0};
  double continuity_weight{10.0};
  double slope_weight{0.5};
  double curvature_weight{10.0};
  double curvature_rate_weight{2.0};
};

struct Result
{
  bool valid{false};
  std::vector<double> offsets;
  int iterations{0};
  double initial_cost{std::numeric_limits<double>::infinity()};
  double final_cost{std::numeric_limits<double>::infinity()};
  std::string reason{"not solved"};
};

// Solves a small convex least-squares problem directly in centerline-relative
// offsets. Every coordinate update is projected back into its local corridor,
// so smoothing can never trade away a hard lateral boundary. The dense matrix
// is intentionally used here: local plans contain only a few dozen stations,
// and fixed-count coordinate descent is predictable on modest hardware.
class Solver
{
public:
  explicit Solver(Config config)
  : config_(std::move(config))
  {
    config_.max_iterations = std::max(0, config_.max_iterations);
    config_.relaxation = std::clamp(config_.relaxation, 0.05, 1.0);
    config_.reference_weight = std::max(0.0, config_.reference_weight);
    config_.continuity_weight = std::max(0.0, config_.continuity_weight);
    config_.slope_weight = std::max(0.0, config_.slope_weight);
    config_.curvature_weight = std::max(0.0, config_.curvature_weight);
    config_.curvature_rate_weight = std::max(0.0, config_.curvature_rate_weight);
  }

  Result solve(const Problem & problem) const
  {
    Result result;
    const std::size_t size = problem.s.size();
    if (size < 3 || problem.initial_offsets.size() != size ||
      problem.reference_offsets.size() != size ||
      problem.lower_offsets.size() != size || problem.upper_offsets.size() != size ||
      (!problem.continuity_offsets.empty() && problem.continuity_offsets.size() != size))
    {
      result.reason = "corridor smoother input sizes are inconsistent";
      return result;
    }
    for (std::size_t index = 0; index < size; ++index) {
      if (!std::isfinite(problem.s[index]) ||
        !std::isfinite(problem.initial_offsets[index]) ||
        !std::isfinite(problem.reference_offsets[index]) ||
        !std::isfinite(problem.lower_offsets[index]) ||
        !std::isfinite(problem.upper_offsets[index]) ||
        problem.lower_offsets[index] > problem.upper_offsets[index])
      {
        result.reason = "corridor smoother received an invalid station";
        return result;
      }
      if (index > 0 && problem.s[index] <= problem.s[index - 1] + 1e-9) {
        result.reason = "corridor smoother stations are not strictly increasing";
        return result;
      }
    }

    std::vector<double> hessian(size * size, 0.0);
    std::vector<double> target(size, 0.0);
    const auto add_term = [&hessian, &target, size](
        const std::vector<std::pair<std::size_t, double>> & coefficients,
        const double desired, const double weight)
      {
        if (weight <= 0.0) {
          return;
        }
        for (const auto & row : coefficients) {
          target[row.first] += weight * row.second * desired;
          for (const auto & column : coefficients) {
            hessian[row.first * size + column.first] +=
              weight * row.second * column.second;
          }
        }
      };

    for (std::size_t index = 0; index < size; ++index) {
      add_term({{index, 1.0}}, problem.reference_offsets[index], config_.reference_weight);
      if (!problem.continuity_offsets.empty() &&
        std::isfinite(problem.continuity_offsets[index]))
      {
        add_term(
          {{index, 1.0}}, problem.continuity_offsets[index],
          config_.continuity_weight);
      }
    }

    for (std::size_t index = 1; index < size; ++index) {
      const double inverse_ds = 1.0 / (problem.s[index] - problem.s[index - 1]);
      add_term(
        {{index - 1, -inverse_ds}, {index, inverse_ds}},
        0.0, config_.slope_weight);
    }

    std::vector<std::vector<std::pair<std::size_t, double>>> curvature_terms;
    curvature_terms.reserve(size - 2);
    for (std::size_t index = 1; index + 1 < size; ++index) {
      const double previous_ds = problem.s[index] - problem.s[index - 1];
      const double next_ds = problem.s[index + 1] - problem.s[index];
      const double inverse_average_ds = 2.0 / (previous_ds + next_ds);
      std::vector<std::pair<std::size_t, double>> coefficients{
        {index - 1, inverse_average_ds / previous_ds},
        {index, -inverse_average_ds * (1.0 / previous_ds + 1.0 / next_ds)},
        {index + 1, inverse_average_ds / next_ds}};
      add_term(coefficients, 0.0, config_.curvature_weight);
      curvature_terms.push_back(std::move(coefficients));
    }

    for (std::size_t term = 1; term < curvature_terms.size(); ++term) {
      const double center_distance = 0.5 *
        (problem.s[term + 2] - problem.s[term]);
      std::vector<double> combined(size, 0.0);
      for (const auto & coefficient : curvature_terms[term]) {
        combined[coefficient.first] += coefficient.second / center_distance;
      }
      for (const auto & coefficient : curvature_terms[term - 1]) {
        combined[coefficient.first] -= coefficient.second / center_distance;
      }
      std::vector<std::pair<std::size_t, double>> coefficients;
      for (std::size_t index = 0; index < size; ++index) {
        if (std::abs(combined[index]) > 1e-12) {
          coefficients.emplace_back(index, combined[index]);
        }
      }
      add_term(coefficients, 0.0, config_.curvature_rate_weight);
    }

    result.offsets.resize(size);
    for (std::size_t index = 0; index < size; ++index) {
      result.offsets[index] = std::clamp(
        problem.initial_offsets[index],
        problem.lower_offsets[index], problem.upper_offsets[index]);
    }
    result.initial_cost = cost(hessian, target, result.offsets);

    for (int iteration = 0; iteration < config_.max_iterations; ++iteration) {
      double maximum_change = 0.0;
      for (std::size_t index = 0; index < size; ++index) {
        const double diagonal = hessian[index * size + index];
        if (diagonal <= 1e-12 ||
          problem.upper_offsets[index] - problem.lower_offsets[index] <= 1e-12)
        {
          result.offsets[index] = problem.lower_offsets[index];
          continue;
        }
        double coupling = 0.0;
        for (std::size_t other = 0; other < size; ++other) {
          if (other != index) {
            coupling += hessian[index * size + other] * result.offsets[other];
          }
        }
        const double unconstrained = (target[index] - coupling) / diagonal;
        const double relaxed = result.offsets[index] +
          config_.relaxation * (unconstrained - result.offsets[index]);
        const double updated = std::clamp(
          relaxed, problem.lower_offsets[index], problem.upper_offsets[index]);
        maximum_change = std::max(maximum_change, std::abs(updated - result.offsets[index]));
        result.offsets[index] = updated;
      }
      result.iterations = iteration + 1;
      if (maximum_change < 1e-5) {
        break;
      }
    }

    result.final_cost = cost(hessian, target, result.offsets);
    result.valid = std::isfinite(result.final_cost) &&
      result.final_cost <= result.initial_cost + 1e-8;
    result.reason = result.valid ?
      "bounded corridor smoothing converged" :
      "bounded corridor smoothing did not reduce its objective";
    return result;
  }

private:
  Config config_;

  static double cost(
    const std::vector<double> & hessian, const std::vector<double> & target,
    const std::vector<double> & offsets)
  {
    const std::size_t size = offsets.size();
    double value = 0.0;
    for (std::size_t row = 0; row < size; ++row) {
      value -= 2.0 * target[row] * offsets[row];
      for (std::size_t column = 0; column < size; ++column) {
        value += offsets[row] * hessian[row * size + column] * offsets[column];
      }
    }
    return value;
  }
};

}  // namespace corridor
}  // namespace path_following_v2

#endif  // PATH_FOLLOWING_V2__BOUNDED_CORRIDOR_SMOOTHER_HPP_
