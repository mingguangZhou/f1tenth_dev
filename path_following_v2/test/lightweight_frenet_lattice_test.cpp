#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

#include "path_following_v2/lightweight_frenet_lattice.hpp"

namespace lattice = path_following_v2::lattice;

namespace
{
lattice::Problem straightProblem(const int station_count, const double station_step)
{
  lattice::Problem problem;
  for (int index = 0; index < station_count; ++index) {
    lattice::Station station;
    station.s = static_cast<double>(index) * station_step;
    station.x = station.s;
    station.y = 0.0;
    station.normal_x = 0.0;
    station.normal_y = 1.0;
    station.reference_offset = 0.0;
    station.lower_offset = -0.8;
    station.upper_offset = 0.8;
    problem.stations.push_back(station);
  }
  return problem;
}

lattice::Config testConfig()
{
  lattice::Config config;
  config.lateral_step_m = 0.05;
  config.max_lateral_shift_m = 0.8;
  config.max_abs_slope = 1.0;
  config.curvature_limit_inv_m = 3.0;
  config.beam_width = 120;
  config.max_solutions = 4;
  config.max_compute_time_ms = 100.0;
  return config;
}

void expectEquivalentSearchResults(
  const lattice::Result & bounded, const lattice::Result & exhaustive)
{
  ASSERT_EQ(bounded.valid, exhaustive.valid);
  EXPECT_EQ(bounded.reason, exhaustive.reason);
  ASSERT_EQ(bounded.solutions.size(), exhaustive.solutions.size());
  for (std::size_t rank = 0; rank < bounded.solutions.size(); ++rank) {
    EXPECT_DOUBLE_EQ(bounded.solutions[rank].cost, exhaustive.solutions[rank].cost);
    ASSERT_EQ(
      bounded.solutions[rank].offsets.size(),
      exhaustive.solutions[rank].offsets.size());
    for (std::size_t station = 0;
      station < bounded.solutions[rank].offsets.size(); ++station)
    {
      EXPECT_DOUBLE_EQ(
        bounded.solutions[rank].offsets[station],
        exhaustive.solutions[rank].offsets[station]);
    }
  }
}
}  // namespace

TEST(LightweightFrenetLattice, KeepsStraightReferenceWhenItIsOpen)
{
  const lattice::Solver solver(testConfig());
  const auto result = solver.solve(straightProblem(13, 0.5));

  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_FALSE(result.solutions.empty());
  for (const double offset : result.solutions.front().offsets) {
    EXPECT_NEAR(offset, 0.0, 1e-9);
  }
}

TEST(LightweightFrenetLattice, SampleGeometryUsesExactCorridorTolerance)
{
  auto config = testConfig();
  config.max_lateral_shift_m = 0.50;
  const lattice::Solver solver(config);
  lattice::Station station;
  station.lower_offset = -0.20;
  station.upper_offset = 0.30;

  EXPECT_TRUE(solver.sampleGeometryAllowed(station, -0.20 - 1e-9));
  EXPECT_FALSE(solver.sampleGeometryAllowed(station, -0.20 - 2e-9));
  EXPECT_TRUE(solver.sampleGeometryAllowed(station, 0.30 + 1e-9));
  EXPECT_FALSE(solver.sampleGeometryAllowed(station, 0.30 + 2e-9));
}

TEST(LightweightFrenetLattice, SampleGeometryUsesExactMaximumShiftTolerance)
{
  auto config = testConfig();
  config.max_lateral_shift_m = 0.50;
  const lattice::Solver solver(config);
  lattice::Station station;

  EXPECT_TRUE(solver.sampleGeometryAllowed(station, -0.50 - 1e-9));
  EXPECT_FALSE(solver.sampleGeometryAllowed(station, -0.50 - 2e-9));
  EXPECT_TRUE(solver.sampleGeometryAllowed(station, 0.50 + 1e-9));
  EXPECT_FALSE(solver.sampleGeometryAllowed(station, 0.50 + 2e-9));
}

TEST(LightweightFrenetLattice, SampleGeometryRejectsNonFiniteOffsets)
{
  const lattice::Solver solver(testConfig());
  const lattice::Station station;

  EXPECT_FALSE(solver.sampleGeometryAllowed(
    station, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(solver.sampleGeometryAllowed(
    station, std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(solver.sampleGeometryAllowed(
    station, -std::numeric_limits<double>::infinity()));
}

TEST(LightweightFrenetLattice, KeepsRacelineZeroInsideAsymmetricMapCorridor)
{
  auto problem = straightProblem(13, 0.5);
  for (auto & station : problem.stations) {
    station.lower_offset = -0.10;
    station.upper_offset = 0.70;
  }

  const lattice::Solver solver(testConfig());
  const auto result = solver.solve(problem);

  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_FALSE(result.solutions.empty());
  for (const double offset : result.solutions.front().offsets) {
    EXPECT_NEAR(offset, 0.0, 1e-9);
  }
}

TEST(LightweightFrenetLattice, RejectsMapBlockedSamplesDuringSearch)
{
  auto problem = straightProblem(17, 0.5);
  const lattice::Solver solver(testConfig());
  for (auto & station : problem.stations) {
    station.sample_clearances.assign(
      static_cast<std::size_t>(solver.sampleCount()),
      std::numeric_limits<double>::infinity());
    if (station.s < 2.0 || station.s > 3.0) {
      continue;
    }
    for (int sample = 0; sample < solver.sampleCount(); ++sample) {
      if (solver.sampleOffset(station, sample) < 0.30 - 1e-9) {
        station.sample_clearances[static_cast<std::size_t>(sample)] = 0.0;
      }
    }
  }

  const auto result = solver.solve(problem);

  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_FALSE(result.solutions.empty());
  for (std::size_t index = 0; index < problem.stations.size(); ++index) {
    if (problem.stations[index].s >= 2.0 && problem.stations[index].s <= 3.0) {
      EXPECT_GE(result.solutions.front().offsets[index], 0.30 - 1e-9);
    }
  }
}

TEST(LightweightFrenetLattice, RejectsNegativeInfinityClearance)
{
  auto problem = straightProblem(7, 0.5);
  const lattice::Solver solver(testConfig());
  auto & required = problem.stations[1];
  required.lower_offset = 0.0;
  required.upper_offset = 0.0;
  required.sample_clearances.assign(
    static_cast<std::size_t>(solver.sampleCount()),
    std::numeric_limits<double>::infinity());
  for (int sample = 0; sample < solver.sampleCount(); ++sample) {
    if (std::abs(solver.sampleOffset(required, sample)) <= 1e-9) {
      required.sample_clearances[static_cast<std::size_t>(sample)] =
        -std::numeric_limits<double>::infinity();
    }
  }

  const auto result = solver.solve(problem);
  EXPECT_FALSE(result.valid);
}

TEST(LightweightFrenetLattice, AllowsPositiveInfinityAsUnboundedClearance)
{
  auto problem = straightProblem(7, 0.5);
  const lattice::Solver solver(testConfig());
  for (auto & station : problem.stations) {
    station.sample_clearances.assign(
      static_cast<std::size_t>(solver.sampleCount()),
      std::numeric_limits<double>::infinity());
  }

  const auto result = solver.solve(problem);

  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_FALSE(result.solutions.empty());
  for (const double offset : result.solutions.front().offsets) {
    EXPECT_NEAR(offset, 0.0, 1e-9);
  }
}

TEST(LightweightFrenetLattice, FindsBoundedLeftPassageAndReturns)
{
  auto problem = straightProblem(17, 0.5);
  for (auto & station : problem.stations) {
    if (station.s >= 2.0 && station.s <= 3.0) {
      station.lower_offset = 0.40;
    }
  }
  problem.stations.back().lower_offset = 0.0;
  problem.stations.back().upper_offset = 0.0;

  const lattice::Solver solver(testConfig());
  const auto result = solver.solve(problem);

  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_FALSE(result.solutions.empty());
  const auto & offsets = result.solutions.front().offsets;
  EXPECT_GE(*std::max_element(offsets.begin(), offsets.end()), 0.40 - 1e-9);
  EXPECT_NEAR(offsets.front(), 0.0, 1e-9);
  EXPECT_NEAR(offsets.back(), 0.0, 1e-9);
}

TEST(LightweightFrenetLattice, RejectsAnImpossibleCorridor)
{
  auto problem = straightProblem(7, 0.25);
  problem.stations[1].lower_offset = 0.80;
  problem.stations[1].upper_offset = 0.80;

  auto config = testConfig();
  config.max_abs_slope = 0.5;
  const lattice::Solver solver(config);
  const auto result = solver.solve(problem);

  EXPECT_FALSE(result.valid);
}

TEST(LightweightFrenetLattice, SmoothGuideWorksWithNarrowBeam)
{
  auto problem = straightProblem(25, 0.25);
  const auto smoothstep = [](const double value) {
      const double u = std::clamp(value, 0.0, 1.0);
      return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
    };
  for (auto & station : problem.stations) {
    if (station.s <= 2.0) {
      station.reference_offset = 0.45 * smoothstep(station.s / 2.0);
    } else if (station.s <= 3.0) {
      station.reference_offset = 0.45;
      station.lower_offset = 0.40;
    } else {
      station.reference_offset = 0.45 *
        (1.0 - smoothstep((station.s - 3.0) / 3.0));
    }
  }
  problem.stations.back().reference_offset = 0.0;
  problem.stations.back().lower_offset = 0.0;
  problem.stations.back().upper_offset = 0.0;

  auto config = testConfig();
  config.beam_width = 10;
  config.curvature_limit_inv_m = 1.2;
  const lattice::Solver solver(config);
  const auto result = solver.solve(problem);

  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_GE(
    *std::max_element(
      result.solutions.front().offsets.begin(),
      result.solutions.front().offsets.end()),
    0.40 - 1e-9);
  EXPECT_NEAR(result.solutions.front().offsets.back(), 0.0, 1e-9);
}

TEST(LightweightFrenetLattice, SlopeReachabilityBoundsMatchExhaustiveSearch)
{
  auto bounded_config = testConfig();
  bounded_config.max_lateral_shift_m = 0.90;
  bounded_config.max_abs_slope = 0.55;
  bounded_config.curvature_limit_inv_m = 5.0;
  bounded_config.beam_width = 70;
  bounded_config.max_compute_time_ms = 1000.0;
  bounded_config.use_slope_reachable_sample_bounds = true;
  auto exhaustive_config = bounded_config;
  exhaustive_config.use_slope_reachable_sample_bounds = false;

  const lattice::Solver bounded_solver(bounded_config);
  const lattice::Solver exhaustive_solver(exhaustive_config);
  std::size_t bounded_transitions = 0;
  std::size_t exhaustive_transitions = 0;
  int valid_scenarios = 0;

  for (int scenario = 0; scenario < 16; ++scenario) {
    SCOPED_TRACE(scenario);
    lattice::Problem problem;
    problem.start_offset = 0.0;
    problem.start_slope = 0.0;
    double s = 0.0;
    const double direction = scenario % 2 == 0 ? 1.0 : -1.0;
    for (int index = 0; index < 41; ++index) {
      if (index > 0) {
        s += 0.22 + 0.01 * static_cast<double>((index + scenario) % 7);
      }
      const double phase = 0.15 * static_cast<double>(scenario);
      const double path_slope = 0.032 * std::cos(0.4 * s + phase);
      const double normalizer = std::hypot(1.0, path_slope);
      const double progress = static_cast<double>(index) / 40.0;

      lattice::Station station;
      station.s = s;
      station.x = s;
      station.y = 0.08 * std::sin(0.4 * s + phase);
      station.normal_x = -path_slope / normalizer;
      station.normal_y = 1.0 / normalizer;
      station.reference_offset =
        direction * 0.35 * std::sin(M_PI * progress);
      station.continuity_offset = station.reference_offset +
        0.02 * std::sin(0.7 * s + phase);
      station.lower_offset = -0.90;
      station.upper_offset = 0.90;
      if (index >= 15 && index <= 24) {
        if (direction > 0.0) {
          station.lower_offset = 0.25;
        } else {
          station.upper_offset = -0.25;
        }
      }
      if (index == 40) {
        station.reference_offset = 0.0;
        station.continuity_offset = 0.0;
        station.lower_offset = 0.0;
        station.upper_offset = 0.0;
      }
      station.sample_clearances.assign(
        static_cast<std::size_t>(bounded_solver.sampleCount()),
        std::numeric_limits<double>::infinity());
      for (int sample = 0; sample < bounded_solver.sampleCount(); ++sample) {
        const double offset = bounded_solver.sampleOffset(station, sample);
        if (index >= 14 && index <= 25 && direction * offset < 0.20) {
          station.sample_clearances[static_cast<std::size_t>(sample)] = 0.0;
        }
      }
      problem.stations.push_back(std::move(station));
    }

    const auto bounded = bounded_solver.solve(problem);
    const auto exhaustive = exhaustive_solver.solve(problem);
    expectEquivalentSearchResults(bounded, exhaustive);
    bounded_transitions += bounded.evaluated_transitions;
    exhaustive_transitions += exhaustive.evaluated_transitions;
    valid_scenarios += bounded.valid ? 1 : 0;
  }

  EXPECT_EQ(valid_scenarios, 16);
  EXPECT_LT(bounded_transitions * 2, exhaustive_transitions);
}

TEST(LightweightFrenetLattice, SlopeReachabilityBoundsKeepBoundarySamples)
{
  auto problem = straightProblem(7, 0.25);
  const std::vector<double> required_offsets{
    0.0, 0.05, 0.10, 0.15, 0.10, 0.05, 0.0};
  for (std::size_t index = 0; index < problem.stations.size(); ++index) {
    problem.stations[index].lower_offset = required_offsets[index];
    problem.stations[index].upper_offset = required_offsets[index];
  }

  auto bounded_config = testConfig();
  bounded_config.max_lateral_shift_m = 0.30;
  // Stay just inside the existing floating-point slope gate. The optimized
  // range is conservative, but the original exact gate remains authoritative.
  bounded_config.max_abs_slope = 0.200001;
  bounded_config.curvature_limit_inv_m = 100.0;
  bounded_config.max_compute_time_ms = 1000.0;
  bounded_config.use_slope_reachable_sample_bounds = true;
  auto exhaustive_config = bounded_config;
  exhaustive_config.use_slope_reachable_sample_bounds = false;

  const auto bounded = lattice::Solver(bounded_config).solve(problem);
  const auto exhaustive = lattice::Solver(exhaustive_config).solve(problem);

  expectEquivalentSearchResults(bounded, exhaustive);
  ASSERT_TRUE(bounded.valid) << bounded.reason;
  ASSERT_FALSE(bounded.solutions.empty());
  ASSERT_EQ(bounded.solutions.front().offsets.size(), required_offsets.size());
  for (std::size_t index = 0; index < required_offsets.size(); ++index) {
    EXPECT_NEAR(bounded.solutions.front().offsets[index], required_offsets[index], 1e-12);
  }
  EXPECT_LT(bounded.evaluated_transitions, exhaustive.evaluated_transitions);
}

TEST(LightweightFrenetLattice, KeepsSampleOutsideRoundedSlopeProduct)
{
  constexpr double station_spacing = 0.21;
  constexpr double required_offset = 0.007;
  const double boundary_slope = required_offset / station_spacing;
  ASSERT_LT(boundary_slope * station_spacing, required_offset);

  auto problem = straightProblem(3, station_spacing);
  problem.stations[1].lower_offset = 0.0;
  problem.stations[1].upper_offset = 0.0;
  problem.stations[2].reference_offset = required_offset;
  problem.stations[2].lower_offset = required_offset;
  problem.stations[2].upper_offset = required_offset;

  auto bounded_config = testConfig();
  bounded_config.max_lateral_shift_m = 0.30;
  bounded_config.max_abs_slope = boundary_slope;
  bounded_config.curvature_limit_inv_m = 100.0;
  bounded_config.max_compute_time_ms = 1000.0;
  bounded_config.use_slope_reachable_sample_bounds = true;
  auto exhaustive_config = bounded_config;
  exhaustive_config.use_slope_reachable_sample_bounds = false;

  const auto bounded = lattice::Solver(bounded_config).solve(problem);
  const auto exhaustive = lattice::Solver(exhaustive_config).solve(problem);

  expectEquivalentSearchResults(bounded, exhaustive);
  ASSERT_TRUE(bounded.valid) << bounded.reason;
  ASSERT_FALSE(bounded.solutions.empty());
  EXPECT_DOUBLE_EQ(bounded.solutions.front().offsets.back(), required_offset);
  EXPECT_LT(bounded.evaluated_transitions, exhaustive.evaluated_transitions);
}
