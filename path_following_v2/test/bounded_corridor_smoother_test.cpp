#include <algorithm>
#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "path_following_v2/bounded_corridor_smoother.hpp"

namespace corridor = path_following_v2::corridor;

namespace
{
corridor::Problem roughPassage()
{
  corridor::Problem problem;
  constexpr int count = 17;
  for (int index = 0; index < count; ++index) {
    const double s = 0.5 * static_cast<double>(index);
    problem.s.push_back(s);
    problem.reference_offsets.push_back(0.0);
    problem.lower_offsets.push_back(-0.8);
    problem.upper_offsets.push_back(0.8);
    problem.continuity_offsets.push_back(std::numeric_limits<double>::quiet_NaN());
    problem.initial_offsets.push_back(
      index == 0 || index == count - 1 ? 0.0 :
      (index % 2 == 0 ? 0.55 : 0.40));
  }
  problem.lower_offsets.front() = 0.0;
  problem.upper_offsets.front() = 0.0;
  problem.lower_offsets.back() = 0.0;
  problem.upper_offsets.back() = 0.0;
  for (int index = 5; index <= 9; ++index) {
    problem.lower_offsets[static_cast<std::size_t>(index)] = 0.38;
  }
  return problem;
}

double squaredSecondDifference(const std::vector<double> & values)
{
  double result = 0.0;
  for (std::size_t index = 1; index + 1 < values.size(); ++index) {
    const double second = values[index - 1] - 2.0 * values[index] + values[index + 1];
    result += second * second;
  }
  return result;
}
}  // namespace

TEST(BoundedCorridorSmoother, ReducesBendingWithoutLeavingBounds)
{
  const auto problem = roughPassage();
  corridor::Config config;
  config.reference_weight = 0.2;
  config.curvature_weight = 15.0;
  config.curvature_rate_weight = 3.0;
  const auto result = corridor::Solver(config).solve(problem);

  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_EQ(result.offsets.size(), problem.s.size());
  EXPECT_LT(squaredSecondDifference(result.offsets),
    squaredSecondDifference(problem.initial_offsets));
  for (std::size_t index = 0; index < result.offsets.size(); ++index) {
    EXPECT_GE(result.offsets[index], problem.lower_offsets[index] - 1e-9);
    EXPECT_LE(result.offsets[index], problem.upper_offsets[index] + 1e-9);
  }
}

TEST(BoundedCorridorSmoother, ContinuityReferenceStabilizesAReplan)
{
  auto problem = roughPassage();
  problem.continuity_offsets.assign(problem.s.size(), 0.45);
  problem.continuity_offsets.front() = 0.0;
  problem.continuity_offsets.back() = 0.0;

  corridor::Config config;
  config.reference_weight = 0.2;
  config.continuity_weight = 20.0;
  const auto result = corridor::Solver(config).solve(problem);

  ASSERT_TRUE(result.valid) << result.reason;
  for (std::size_t index = 5; index <= 9; ++index) {
    EXPECT_NEAR(result.offsets[index], 0.45, 0.08);
  }
}

TEST(BoundedCorridorSmoother, RejectsInconsistentBounds)
{
  auto problem = roughPassage();
  problem.lower_offsets[4] = 0.5;
  problem.upper_offsets[4] = 0.4;

  const auto result = corridor::Solver(corridor::Config()).solve(problem);
  EXPECT_FALSE(result.valid);
}
