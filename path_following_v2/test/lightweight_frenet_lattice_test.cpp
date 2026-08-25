#include <algorithm>
#include <cmath>
#include <limits>

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
