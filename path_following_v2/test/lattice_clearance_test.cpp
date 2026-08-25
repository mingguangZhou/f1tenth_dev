#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "gtest/gtest.h"

#include "path_following_v2/lattice_clearance.hpp"

namespace
{
struct Point
{
  double x{0.0};
  double y{0.0};
};

double exhaustiveClearance(
  const double sample_x, const double sample_y, const double initial_clearance,
  const std::vector<Point> & points)
{
  double clearance = initial_clearance;
  for (const auto & point : points) {
    clearance = std::min(
      clearance, std::hypot(sample_x - point.x, sample_y - point.y));
  }
  return clearance;
}

bool blocked(const double clearance, const double collision_clearance)
{
  return std::isnan(clearance) ||
         clearance == -std::numeric_limits<double>::infinity() ||
         (std::isfinite(clearance) && clearance <= collision_clearance);
}

double clearanceCost(const double clearance, const double preferred_clearance)
{
  if (!std::isfinite(clearance) || clearance >= preferred_clearance) {
    return 0.0;
  }
  const double shortfall = preferred_clearance - clearance;
  return shortfall * shortfall;
}
}  // namespace

TEST(LatticeClearance, PreservesFeasibilityAndClearanceCostAcrossRandomClouds)
{
  constexpr double collision_clearance = 0.29;
  constexpr double preferred_clearance = 0.45;
  std::mt19937 generator(20260825U);
  std::uniform_real_distribution<double> coordinate(-1.5, 1.5);
  std::uniform_real_distribution<double> map_clearance(0.30, 1.0);

  for (int trial = 0; trial < 2000; ++trial) {
    const double sample_x = coordinate(generator);
    const double sample_y = coordinate(generator);
    const double initial_clearance = trial % 3 == 0 ?
      std::numeric_limits<double>::infinity() : map_clearance(generator);
    std::vector<Point> points;
    points.reserve(128);
    for (int index = 0; index < 128; ++index) {
      points.push_back(Point{coordinate(generator), coordinate(generator)});
    }

    const double exhaustive = exhaustiveClearance(
      sample_x, sample_y, initial_clearance, points);
    const double optimized =
      path_following_v2::lattice_clearance::refineWithTrustedPoints(
      sample_x, sample_y, initial_clearance, preferred_clearance,
      collision_clearance, points);

    EXPECT_EQ(blocked(optimized, collision_clearance),
      blocked(exhaustive, collision_clearance));
    if (!blocked(exhaustive, collision_clearance)) {
      EXPECT_DOUBLE_EQ(
        clearanceCost(optimized, preferred_clearance),
        clearanceCost(exhaustive, preferred_clearance));
    }
  }
}

TEST(LatticeClearance, IncludesCollisionAndPreferredBoundaryEquality)
{
  constexpr double collision_clearance = 0.29;
  constexpr double preferred_clearance = 0.45;
  const double below_collision = std::nextafter(
    collision_clearance, -std::numeric_limits<double>::infinity());
  const double above_preferred = std::nextafter(
    preferred_clearance, std::numeric_limits<double>::infinity());
  const std::vector<Point> points{
    Point{preferred_clearance, 0.0},
    Point{above_preferred, 0.0},
    Point{collision_clearance, 0.0},
    Point{below_collision, 0.0}};

  const double optimized =
    path_following_v2::lattice_clearance::refineWithTrustedPoints(
    0.0, 0.0, std::numeric_limits<double>::infinity(),
    preferred_clearance, collision_clearance, points);
  EXPECT_LE(optimized, collision_clearance);
}

TEST(LatticeClearance, PreferredEqualToCollisionDoesNotBlockClearSpace)
{
  constexpr double clearance_threshold = 0.29;
  const std::vector<Point> points{Point{0.50, 0.0}};
  const double optimized =
    path_following_v2::lattice_clearance::refineWithTrustedPoints(
    0.0, 0.0, std::numeric_limits<double>::infinity(),
    clearance_threshold, clearance_threshold, points);

  EXPECT_TRUE(std::isinf(optimized));
  EXPECT_FALSE(blocked(optimized, clearance_threshold));
}

TEST(LatticeClearance, MapCollisionShortCircuitsWithoutChangingDecision)
{
  constexpr double collision_clearance = 0.29;
  constexpr double preferred_clearance = 0.45;
  const std::vector<Point> points{Point{0.01, 0.0}};
  const double optimized =
    path_following_v2::lattice_clearance::refineWithTrustedPoints(
    0.0, 0.0, 0.20, preferred_clearance, collision_clearance, points);

  EXPECT_DOUBLE_EQ(optimized, 0.20);
  EXPECT_TRUE(blocked(optimized, collision_clearance));
}

TEST(LatticeClearance, PreservesFailClosedNonFiniteMapValues)
{
  constexpr double collision_clearance = 0.29;
  constexpr double preferred_clearance = 0.45;
  const std::vector<Point> points{Point{1.0, 1.0}};

  for (const double initial_clearance : {
      std::numeric_limits<double>::quiet_NaN(),
      -std::numeric_limits<double>::infinity()})
  {
    const double optimized =
      path_following_v2::lattice_clearance::refineWithTrustedPoints(
      0.0, 0.0, initial_clearance, preferred_clearance,
      collision_clearance, points);
    EXPECT_TRUE(blocked(optimized, collision_clearance));
  }
}

TEST(LatticeClearance, FiniteMapClearanceTighterThanPreferredIsPreserved)
{
  constexpr double collision_clearance = 0.29;
  constexpr double preferred_clearance = 0.45;
  constexpr double map_clearance = 0.35;
  const std::vector<Point> points{
    Point{0.36, 0.0},
    Point{preferred_clearance, preferred_clearance}};

  const double optimized =
    path_following_v2::lattice_clearance::refineWithTrustedPoints(
    0.0, 0.0, map_clearance, preferred_clearance,
    collision_clearance, points);
  EXPECT_DOUBLE_EQ(optimized, map_clearance);
}

TEST(LatticeClearance, DiagonalInsideAxisBoundsStillUsesExactDistance)
{
  constexpr double collision_clearance = 0.29;
  constexpr double preferred_clearance = 0.45;
  const std::vector<Point> points{Point{0.30, 0.30}};

  const double optimized =
    path_following_v2::lattice_clearance::refineWithTrustedPoints(
    0.0, 0.0, std::numeric_limits<double>::infinity(),
    preferred_clearance, collision_clearance, points);
  EXPECT_DOUBLE_EQ(optimized, std::hypot(0.30, 0.30));
}
