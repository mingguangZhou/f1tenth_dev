#ifndef PATH_FOLLOWING_V2__LIGHTWEIGHT_FRENET_LATTICE_HPP_
#define PATH_FOLLOWING_V2__LIGHTWEIGHT_FRENET_LATTICE_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace path_following_v2
{
namespace lattice
{

struct Station
{
  double s{0.0};
  double x{0.0};
  double y{0.0};
  double normal_x{0.0};
  double normal_y{1.0};
  double reference_offset{0.0};
  double continuity_offset{std::numeric_limits<double>::quiet_NaN()};
  double lower_offset{-std::numeric_limits<double>::infinity()};
  double upper_offset{std::numeric_limits<double>::infinity()};
  std::vector<double> sample_clearances;
};

struct Problem
{
  std::vector<Station> stations;
  double start_offset{0.0};
  double start_slope{0.0};
};

struct Config
{
  double lateral_step_m{0.05};
  double max_lateral_shift_m{0.80};
  double max_abs_slope{0.55};
  double curvature_limit_inv_m{1.0};
  double collision_clearance_m{0.24};
  double preferred_clearance_m{0.45};
  double reference_weight{4.0};
  double slope_weight{1.0};
  double curvature_weight{8.0};
  double curvature_rate_weight{2.0};
  double continuity_weight{10.0};
  double clearance_weight{2.0};
  double terminal_weight_multiplier{5.0};
  double terminal_distance_m{1.0};
  int beam_width{120};
  int max_solutions{6};
  double max_compute_time_ms{12.0};
  // Internal differential-test switch. Production keeps the conservative
  // bound enabled; disabling it restores the exhaustive next-sample loop.
  bool use_slope_reachable_sample_bounds{true};
};

struct Solution
{
  std::vector<double> offsets;
  double cost{std::numeric_limits<double>::infinity()};
};

struct Result
{
  bool valid{false};
  std::vector<Solution> solutions;
  std::size_t evaluated_transitions{0};
  double compute_time_ms{0.0};
  std::string reason{"not solved"};
};

class Solver
{
public:
  explicit Solver(Config config)
  : config_(std::move(config))
  {
    config_.lateral_step_m = std::max(0.01, config_.lateral_step_m);
    config_.max_lateral_shift_m = std::max(
      config_.lateral_step_m, config_.max_lateral_shift_m);
    config_.max_abs_slope = std::max(0.01, config_.max_abs_slope);
    config_.curvature_limit_inv_m = std::max(0.01, config_.curvature_limit_inv_m);
    config_.collision_clearance_m = std::max(0.0, config_.collision_clearance_m);
    config_.preferred_clearance_m = std::max(
      config_.collision_clearance_m, config_.preferred_clearance_m);
    config_.beam_width = std::max(1, config_.beam_width);
    config_.max_solutions = std::max(1, config_.max_solutions);
    config_.max_compute_time_ms = std::max(0.1, config_.max_compute_time_ms);
    half_sample_count_ = static_cast<int>(
      std::ceil(config_.max_lateral_shift_m / config_.lateral_step_m));
  }

  int sampleCount() const
  {
    return 2 * half_sample_count_ + 1;
  }

  double sampleOffset(const Station & station, const int sample_index) const
  {
    return station.reference_offset +
           static_cast<double>(sample_index - half_sample_count_) * config_.lateral_step_m;
  }

  bool sampleGeometryAllowed(const Station & station, const double lateral) const
  {
    return !(
      !std::isfinite(lateral) ||
      std::abs(lateral) > config_.max_lateral_shift_m + 1e-9 ||
      lateral < station.lower_offset - 1e-9 ||
      lateral > station.upper_offset + 1e-9);
  }

  Result solve(const Problem & problem) const
  {
    Result result;
    const auto started = std::chrono::steady_clock::now();
    const auto finish = [&result, &started]() {
        result.compute_time_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
      };

    if (problem.stations.size() < 3) {
      result.reason = "lattice needs at least three longitudinal stations";
      finish();
      return result;
    }
    if (!std::isfinite(problem.start_offset) || !std::isfinite(problem.start_slope)) {
      result.reason = "lattice start state is not finite";
      finish();
      return result;
    }

    const int sample_count = sampleCount();
    for (const auto & station : problem.stations) {
      if (!station.sample_clearances.empty() &&
        static_cast<int>(station.sample_clearances.size()) != sample_count)
      {
        result.reason = "station clearance grid does not match lateral sample count";
        finish();
        return result;
      }
    }

    struct State
    {
      int previous_sample{-1};
      int current_sample{-1};
      int parent_state{-1};
      double cost{std::numeric_limits<double>::infinity()};
      double slope{0.0};
      double curvature{0.0};
    };

    std::vector<std::vector<State>> layers(problem.stations.size());
    std::vector<double> next_sample_offsets(static_cast<std::size_t>(sample_count));
    const Station & start_station = problem.stations.front();
    const Station & first_station = problem.stations[1];
    const double first_ds = first_station.s - start_station.s;
    if (first_ds <= 1e-6) {
      result.reason = "lattice station distances are not strictly increasing";
      finish();
      return result;
    }

    for (int sample = 0; sample < sample_count; ++sample) {
      const double lateral = sampleOffset(first_station, sample);
      if (!sampleAllowed(first_station, sample, lateral)) {
        continue;
      }
      const double slope = (lateral - problem.start_offset) / first_ds;
      if (std::abs(slope) > config_.max_abs_slope) {
        continue;
      }
      const double slope_change = slope - problem.start_slope;
      State state;
      state.current_sample = sample;
      state.cost = nodeCost(problem, 1, sample, lateral) +
        config_.slope_weight * slope * slope +
        config_.curvature_weight * slope_change * slope_change /
        std::max(first_ds * first_ds, 1e-6) +
        config_.curvature_rate_weight * slope_change * slope_change /
        std::max(first_ds * first_ds * first_ds, 1e-6);
      state.slope = slope;
      state.curvature = slope_change / std::max(first_ds, 1e-6);
      if (std::isfinite(state.cost)) {
        layers[1].push_back(state);
      }
    }
    pruneLayer(&layers[1]);
    if (layers[1].empty()) {
      result.reason = "no reachable lateral sample at the first lattice station";
      finish();
      return result;
    }

    for (std::size_t station_index = 1;
      station_index + 1 < problem.stations.size(); ++station_index)
    {
      if (elapsedMs(started) > config_.max_compute_time_ms) {
        result.reason = "lattice compute budget exceeded";
        finish();
        return result;
      }

      const Station & previous_station = problem.stations[station_index - 1];
      const Station & current_station = problem.stations[station_index];
      const Station & next_station = problem.stations[station_index + 1];
      const double next_ds = next_station.s - current_station.s;
      if (next_ds <= 1e-6) {
        result.reason = "lattice station distances are not strictly increasing";
        finish();
        return result;
      }

      std::vector<State> best_by_pair(
        static_cast<std::size_t>(sample_count * sample_count));
      std::vector<bool> pair_used(best_by_pair.size(), false);
      for (int sample = 0; sample < sample_count; ++sample) {
        next_sample_offsets[static_cast<std::size_t>(sample)] =
          sampleOffset(next_station, sample);
      }

      for (std::size_t state_index = 0;
        state_index < layers[station_index].size(); ++state_index)
      {
        const State & state = layers[station_index][state_index];
        const double current_lateral = sampleOffset(
          current_station, state.current_sample);
        const double previous_lateral = station_index == 1 ?
          problem.start_offset :
          sampleOffset(previous_station, state.previous_sample);
        const Point previous_point = shiftedPoint(previous_station, previous_lateral);
        const Point current_point = shiftedPoint(current_station, current_lateral);

        int first_next_sample = 0;
        int one_past_last_next_sample = sample_count;
        const double maximum_lateral_delta = config_.max_abs_slope * next_ds;
        if (config_.use_slope_reachable_sample_bounds &&
          std::isfinite(current_lateral) && std::isfinite(maximum_lateral_delta) &&
          maximum_lateral_delta >= 0.0 &&
          std::isfinite(next_sample_offsets.front()) &&
          std::isfinite(next_sample_offsets.back()))
        {
          const double minimum_reachable_lateral =
            current_lateral - maximum_lateral_delta;
          const double maximum_reachable_lateral =
            current_lateral + maximum_lateral_delta;
          if (minimum_reachable_lateral > next_sample_offsets.front() ||
            maximum_reachable_lateral < next_sample_offsets.back())
          {
            const auto lower = std::lower_bound(
              next_sample_offsets.begin(), next_sample_offsets.end(),
              minimum_reachable_lateral);
            const auto upper = std::upper_bound(
              next_sample_offsets.begin(), next_sample_offsets.end(),
              maximum_reachable_lateral);
            // Retain one sample beyond each computed edge. The exact slope
            // gate below remains authoritative at floating-point boundaries.
            first_next_sample = std::max(
              0, static_cast<int>(lower - next_sample_offsets.begin()) - 1);
            one_past_last_next_sample = std::min(
              sample_count,
              static_cast<int>(upper - next_sample_offsets.begin()) + 1);
          }
        }

        for (int next_sample = first_next_sample;
          next_sample < one_past_last_next_sample; ++next_sample)
        {
          ++result.evaluated_transitions;
          const double next_lateral =
            next_sample_offsets[static_cast<std::size_t>(next_sample)];
          if (!sampleAllowed(next_station, next_sample, next_lateral)) {
            continue;
          }
          const double next_slope = (next_lateral - current_lateral) / next_ds;
          if (std::abs(next_slope) > config_.max_abs_slope) {
            continue;
          }
          const Point next_point = shiftedPoint(next_station, next_lateral);
          const double curvature = threePointCurvature(
            previous_point, current_point, next_point);
          if (!std::isfinite(curvature) ||
            curvature > config_.curvature_limit_inv_m)
          {
            continue;
          }

          const double slope_change = next_slope - state.slope;
          const double approximate_curvature = slope_change / std::max(next_ds, 1e-6);
          const double curvature_change = approximate_curvature - state.curvature;
          const double transition_cost =
            config_.slope_weight * next_slope * next_slope +
            config_.curvature_weight * curvature * curvature +
            config_.curvature_weight * slope_change * slope_change /
            std::max(next_ds * next_ds, 1e-6) +
            config_.curvature_rate_weight * curvature_change * curvature_change /
            std::max(next_ds * next_ds, 1e-6);
          const double cost = state.cost + transition_cost +
            nodeCost(problem, station_index + 1, next_sample, next_lateral);
          if (!std::isfinite(cost)) {
            continue;
          }

          const std::size_t key = static_cast<std::size_t>(
            state.current_sample * sample_count + next_sample);
          if (!pair_used[key] || cost < best_by_pair[key].cost) {
            pair_used[key] = true;
            State next_state;
            next_state.previous_sample = state.current_sample;
            next_state.current_sample = next_sample;
            next_state.parent_state = static_cast<int>(state_index);
            next_state.cost = cost;
            next_state.slope = next_slope;
            next_state.curvature = approximate_curvature;
            best_by_pair[key] = next_state;
          }
        }
      }

      auto & next_layer = layers[station_index + 1];
      next_layer.reserve(best_by_pair.size());
      for (std::size_t index = 0; index < best_by_pair.size(); ++index) {
        if (pair_used[index]) {
          next_layer.push_back(best_by_pair[index]);
        }
      }
      pruneLayer(&next_layer);
      if (next_layer.empty()) {
        result.reason = "no curvature-feasible transition through the lateral corridor";
        finish();
        return result;
      }
    }

    const auto & final_layer = layers.back();
    const int solution_count = std::min(
      config_.max_solutions, static_cast<int>(final_layer.size()));
    result.solutions.reserve(static_cast<std::size_t>(solution_count));
    for (int rank = 0; rank < solution_count; ++rank) {
      Solution solution;
      solution.cost = final_layer[static_cast<std::size_t>(rank)].cost;
      solution.offsets.resize(problem.stations.size(), problem.start_offset);
      int state_index = rank;
      for (std::size_t layer_index = problem.stations.size() - 1;
        layer_index >= 1; --layer_index)
      {
        const State & state = layers[layer_index][static_cast<std::size_t>(state_index)];
        solution.offsets[layer_index] = sampleOffset(
          problem.stations[layer_index], state.current_sample);
        state_index = state.parent_state;
        if (layer_index == 1) {
          break;
        }
      }
      solution.offsets.front() = problem.start_offset;
      result.solutions.push_back(std::move(solution));
    }

    result.valid = !result.solutions.empty();
    result.reason = result.valid ?
      "bounded Frenet lattice found feasible paths" :
      "bounded Frenet lattice returned no path";
    finish();
    return result;
  }

private:
  struct Point
  {
    double x{0.0};
    double y{0.0};
  };

  Config config_;
  int half_sample_count_{1};

  static double elapsedMs(const std::chrono::steady_clock::time_point & started)
  {
    return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  }

  bool sampleAllowed(
    const Station & station, const int sample_index, const double lateral) const
  {
    if (!sampleGeometryAllowed(station, lateral)) {
      return false;
    }
    if (!station.sample_clearances.empty()) {
      const double clearance = station.sample_clearances[static_cast<std::size_t>(sample_index)];
      // +infinity means no observed/map limitation. NaN and -infinity mean
      // unavailable or out-of-map evidence and must fail closed.
      if (std::isnan(clearance) ||
        clearance == -std::numeric_limits<double>::infinity() ||
        (std::isfinite(clearance) && clearance <= config_.collision_clearance_m))
      {
        return false;
      }
    }
    return true;
  }

  double nodeCost(
    const Problem & problem, const std::size_t station_index,
    const int sample_index, const double lateral) const
  {
    const Station & station = problem.stations[station_index];
    const double remaining = problem.stations.back().s - station.s;
    const double terminal_multiplier = remaining <= config_.terminal_distance_m ?
      config_.terminal_weight_multiplier : 1.0;
    const double reference_error = lateral - station.reference_offset;
    double cost = config_.reference_weight * terminal_multiplier *
      reference_error * reference_error;
    if (std::isfinite(station.continuity_offset)) {
      const double continuity_error = lateral - station.continuity_offset;
      cost += config_.continuity_weight * continuity_error * continuity_error;
    }
    if (!station.sample_clearances.empty()) {
      const double clearance = station.sample_clearances[static_cast<std::size_t>(sample_index)];
      if (std::isfinite(clearance) && clearance < config_.preferred_clearance_m) {
        const double shortfall = config_.preferred_clearance_m - clearance;
        cost += config_.clearance_weight * shortfall * shortfall;
      }
    }
    return cost;
  }

  Point shiftedPoint(const Station & station, const double lateral) const
  {
    return Point{
      station.x + station.normal_x * lateral,
      station.y + station.normal_y * lateral};
  }

  static double threePointCurvature(
    const Point & a, const Point & b, const Point & c)
  {
    const double ab = std::hypot(b.x - a.x, b.y - a.y);
    const double bc = std::hypot(c.x - b.x, c.y - b.y);
    const double ac = std::hypot(c.x - a.x, c.y - a.y);
    if (ab <= 1e-6 || bc <= 1e-6 || ac <= 1e-6) {
      return std::numeric_limits<double>::infinity();
    }
    const double cross = std::abs(
      (b.x - a.x) * (c.y - a.y) -
      (b.y - a.y) * (c.x - a.x));
    return 2.0 * cross / (ab * bc * ac);
  }

  template<typename StateT>
  void pruneLayer(std::vector<StateT> * const states) const
  {
    std::sort(
      states->begin(), states->end(),
      [](const StateT & left, const StateT & right) {
        return left.cost < right.cost;
      });
    if (static_cast<int>(states->size()) > config_.beam_width) {
      states->resize(static_cast<std::size_t>(config_.beam_width));
    }
  }
};

}  // namespace lattice
}  // namespace path_following_v2

#endif  // PATH_FOLLOWING_V2__LIGHTWEIGHT_FRENET_LATTICE_HPP_
