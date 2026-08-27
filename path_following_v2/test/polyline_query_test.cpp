#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <vector>

#include "path_following_v2/polyline_query.hpp"

namespace
{

namespace query = path_following_v2::polyline_query;

struct Point
{
  double x{0.0};
  double y{0.0};
};

struct LegacyNearest
{
  bool valid{false};
  double distance{std::numeric_limits<double>::infinity()};
  double ratio{0.0};
  double residual_x{0.0};
  double residual_y{0.0};
  std::size_t segment_index{0};
};

LegacyNearest exhaustiveNearest(
  const Point & point, const std::vector<Point> & points,
  const std::size_t begin, const std::size_t requested_end,
  const bool skip_unprojectable)
{
  LegacyNearest best;
  if (points.size() < 2) {
    return best;
  }
  const std::size_t end = std::min(requested_end, points.size() - 1);
  for (std::size_t index = begin; index < end; ++index) {
    const auto & first = points[index];
    const auto & second = points[index + 1];
    const double dx = second.x - first.x;
    const double dy = second.y - first.y;
    const double length_squared = dx * dx + dy * dy;
    if (skip_unprojectable &&
      length_squared <= query::kMinimumProjectableLengthSquared)
    {
      continue;
    }
    const double ratio = length_squared > query::kMinimumProjectableLengthSquared ?
      std::clamp(
      ((point.x - first.x) * dx + (point.y - first.y) * dy) /
      length_squared, 0.0, 1.0) : 0.0;
    const double residual_x = point.x - (first.x + ratio * dx);
    const double residual_y = point.y - (first.y + ratio * dy);
    const double distance = std::hypot(residual_x, residual_y);
    if (distance < best.distance) {
      best.valid = true;
      best.distance = distance;
      best.ratio = ratio;
      best.residual_x = residual_x;
      best.residual_y = residual_y;
      best.segment_index = index;
    }
  }
  return best;
}

bool exhaustiveAnyWithinDistance(
  const Point & point, const std::vector<Point> & points,
  const double threshold, const std::size_t begin,
  const std::size_t requested_end)
{
  const auto nearest = exhaustiveNearest(
    point, points, begin, requested_end, false);
  return nearest.valid && nearest.distance <= threshold;
}

void expectSameNearest(
  const LegacyNearest & expected, const query::NearestPoint & actual)
{
  ASSERT_EQ(actual.valid, expected.valid);
  if (!expected.valid) {
    EXPECT_TRUE(std::isinf(actual.distance));
    return;
  }
  EXPECT_EQ(actual.segment_index, expected.segment_index);
  EXPECT_DOUBLE_EQ(actual.distance, expected.distance);
  EXPECT_DOUBLE_EQ(actual.ratio, expected.ratio);
  EXPECT_DOUBLE_EQ(actual.residual_x, expected.residual_x);
  EXPECT_DOUBLE_EQ(actual.residual_y, expected.residual_y);
}

TEST(PolylineQueryTest, PreparedSegmentsRetainOriginalGeometryAndDegenerateRule)
{
  const std::vector<Point> points{
    {0.0, 0.0}, {3.0, 4.0}, {3.0, 4.0}, {3.000001, 4.0}};
  const auto segments = query::prepareSegments(points);
  ASSERT_EQ(segments.size(), 3U);
  EXPECT_DOUBLE_EQ(segments[0].dx, 3.0);
  EXPECT_DOUBLE_EQ(segments[0].dy, 4.0);
  EXPECT_DOUBLE_EQ(segments[0].length_squared, 25.0);
  EXPECT_DOUBLE_EQ(segments[0].length, 5.0);
  EXPECT_TRUE(segments[0].projectable);
  EXPECT_FALSE(segments[1].projectable);

  const double boundary = std::sqrt(query::kMinimumProjectableLengthSquared);
  const std::vector<Point> boundary_points{{0.0, 0.0}, {boundary, 0.0}};
  const auto boundary_segments = query::prepareSegments(boundary_points);
  ASSERT_EQ(boundary_segments.size(), 1U);
  EXPECT_EQ(
    boundary_segments[0].projectable,
    boundary_segments[0].length_squared >
    query::kMinimumProjectableLengthSquared);

  double below_boundary = boundary;
  while (below_boundary * below_boundary >=
    query::kMinimumProjectableLengthSquared)
  {
    below_boundary = std::nextafter(
      below_boundary, -std::numeric_limits<double>::infinity());
  }
  double above_boundary = boundary;
  while (above_boundary * above_boundary <=
    query::kMinimumProjectableLengthSquared)
  {
    above_boundary = std::nextafter(
      above_boundary, std::numeric_limits<double>::infinity());
  }
  const auto below_segments = query::prepareSegments(
    std::vector<Point>{{0.0, 0.0}, {below_boundary, 0.0}});
  const auto above_segments = query::prepareSegments(
    std::vector<Point>{{0.0, 0.0}, {above_boundary, 0.0}});
  ASSERT_EQ(below_segments.size(), 1U);
  ASSERT_EQ(above_segments.size(), 1U);
  ASSERT_LT(
    below_segments[0].length_squared,
    query::kMinimumProjectableLengthSquared);
  ASSERT_GT(
    above_segments[0].length_squared,
    query::kMinimumProjectableLengthSquared);
  EXPECT_FALSE(below_segments[0].projectable);
  EXPECT_TRUE(above_segments[0].projectable);

  const auto below_included = query::nearestPoint(
    below_boundary, 0.0, below_segments);
  ASSERT_TRUE(below_included.valid);
  EXPECT_DOUBLE_EQ(below_included.ratio, 0.0);
  EXPECT_DOUBLE_EQ(below_included.distance, below_boundary);
  EXPECT_FALSE(query::nearestPoint(
    below_boundary, 0.0, below_segments, 0,
    below_segments.size(), true).valid);
  const auto above_projected = query::nearestPoint(
    above_boundary, 0.0, above_segments, 0,
    above_segments.size(), true);
  ASSERT_TRUE(above_projected.valid);
  EXPECT_DOUBLE_EQ(above_projected.ratio, 1.0);
  EXPECT_DOUBLE_EQ(above_projected.distance, 0.0);
}

TEST(PolylineQueryTest, HandlesTiesDegeneratesAndSubrangesLikeExhaustiveSearch)
{
  const std::vector<Point> hairpin{
    {0.0, 0.0}, {2.0, 0.0}, {2.0, 1.0}, {0.0, 1.0},
    {0.0, 1.0}, {-1.0, 1.0}};
  const auto segments = query::prepareSegments(hairpin);

  for (const Point point : std::vector<Point>{{1.0, 0.5}, {2.0, 0.0}, {0.0, 1.0}}) {
    expectSameNearest(
      exhaustiveNearest(point, hairpin, 0, segments.size(), false),
      query::nearestPoint(point.x, point.y, segments));
    expectSameNearest(
      exhaustiveNearest(point, hairpin, 2, 5, false),
      query::nearestPoint(point.x, point.y, segments, 2, 5));
  }

  const Point on_degenerate{0.0, 1.0};
  expectSameNearest(
    exhaustiveNearest(on_degenerate, hairpin, 0, segments.size(), true),
    query::nearestPoint(
      on_degenerate.x, on_degenerate.y, segments, 0, segments.size(), true));
}

TEST(PolylineQueryTest, ClosedPolylineIncludesSeamWithoutChangingTieOrder)
{
  const std::vector<Point> square{
    {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
  const auto segments = query::prepareSegments(square, true);
  auto explicitly_closed_square = square;
  explicitly_closed_square.push_back(square.front());
  ASSERT_EQ(segments.size(), 4U);

  const auto beside_seam = query::nearestPoint(-0.2, 0.5, segments);
  expectSameNearest(
    exhaustiveNearest(
      Point{-0.2, 0.5}, explicitly_closed_square, 0,
      explicitly_closed_square.size() - 1, false),
    beside_seam);
  ASSERT_TRUE(beside_seam.valid);
  EXPECT_EQ(beside_seam.segment_index, 3U);
  EXPECT_DOUBLE_EQ(beside_seam.distance, 0.2);
  EXPECT_TRUE(query::anyWithinDistance(-0.2, 0.5, 0.2, segments));
  EXPECT_FALSE(query::anyWithinDistance(
    -0.2, 0.5,
    std::nextafter(0.2, -std::numeric_limits<double>::infinity()), segments));

  const auto center_tie = query::nearestPoint(0.5, 0.5, segments);
  expectSameNearest(
    exhaustiveNearest(
      Point{0.5, 0.5}, explicitly_closed_square, 0,
      explicitly_closed_square.size() - 1, false),
    center_tie);
  ASSERT_TRUE(center_tie.valid);
  EXPECT_EQ(center_tie.segment_index, 0U);
  EXPECT_DOUBLE_EQ(center_tie.distance, 0.5);
}

TEST(PolylineQueryTest, DoesNotPruneALaterSegmentCloserByOneUlp)
{
  const double first_distance = 1.0;
  const double later_distance = std::nextafter(
    first_distance, -std::numeric_limits<double>::infinity());
  const auto first = query::prepareSegments(
    std::vector<Point>{{first_distance, -1.0}, {first_distance, 1.0}});
  const auto later = query::prepareSegments(
    std::vector<Point>{{later_distance, -1.0}, {later_distance, 1.0}});
  ASSERT_EQ(first.size(), 1U);
  ASSERT_EQ(later.size(), 1U);
  const std::vector<query::Segment> segments{first.front(), later.front()};

  const auto nearest = query::nearestPoint(0.0, 0.0, segments);
  ASSERT_TRUE(nearest.valid);
  EXPECT_EQ(nearest.segment_index, 1U);
  EXPECT_DOUBLE_EQ(nearest.distance, later_distance);
  EXPECT_EQ(nearest.exact_distance_evaluations, 2U);
}

TEST(PolylineQueryTest, DiagonalAabbCornerPreservesAdjacentThresholdDecisions)
{
  const Point point{0.0, 0.0};
  const std::vector<Point> path{{3.0, 4.0}, {6.0, 7.0}};
  const auto segments = query::prepareSegments(path);
  ASSERT_EQ(segments.size(), 1U);
  const double exact_distance = 5.0;
  const std::vector<double> thresholds{
    std::nextafter(
      exact_distance, -std::numeric_limits<double>::infinity()),
    exact_distance,
    std::nextafter(
      exact_distance, std::numeric_limits<double>::infinity())};
  for (const double threshold : thresholds) {
    EXPECT_EQ(
      query::anyWithinDistance(
        point.x, point.y, threshold, segments),
      exhaustiveAnyWithinDistance(
        point, path, threshold, 0, segments.size()));
  }
}

TEST(PolylineQueryTest, LargeCoordinateTranslationPreservesExactResults)
{
  constexpr double base_x = 1e12;
  constexpr double base_y = -1e12;
  const Point point{base_x, base_y};
  const std::vector<Point> path{
    {base_x + 3.0, base_y + 4.0},
    {base_x + 6.0, base_y + 7.0}};
  const auto segments = query::prepareSegments(path);
  const auto expected = exhaustiveNearest(
    point, path, 0, segments.size(), false);
  const auto actual = query::nearestPoint(point.x, point.y, segments);
  expectSameNearest(expected, actual);

  for (const double threshold : std::vector<double>{
      std::nextafter(
        expected.distance, -std::numeric_limits<double>::infinity()),
      expected.distance,
      std::nextafter(
        expected.distance, std::numeric_limits<double>::infinity())})
  {
    EXPECT_EQ(
      query::anyWithinDistance(
        point.x, point.y, threshold, segments),
      exhaustiveAnyWithinDistance(
        point, path, threshold, 0, segments.size()));
  }
}

TEST(PolylineQueryTest, RandomizedNearestQueriesExactlyMatchLegacyFormula)
{
  std::mt19937_64 generator(0x5e67c0deULL);
  std::uniform_real_distribution<double> step(-0.30, 0.30);
  std::uniform_real_distribution<double> query_offset(-1.0, 1.0);
  std::bernoulli_distribution duplicate(0.08);

  for (int path_trial = 0; path_trial < 80; ++path_trial) {
    std::vector<Point> points;
    points.push_back(Point{step(generator), step(generator)});
    for (int index = 1; index < 90; ++index) {
      if (duplicate(generator)) {
        points.push_back(points.back());
      } else {
        points.push_back(Point{
          points.back().x + 0.12 + step(generator),
          points.back().y + step(generator)});
      }
    }
    const auto segments = query::prepareSegments(points);
    ASSERT_EQ(segments.size(), points.size() - 1);

    std::uniform_int_distribution<std::size_t> segment_index(0, segments.size() - 1);
    for (int query_trial = 0; query_trial < 120; ++query_trial) {
      const std::size_t anchor = segment_index(generator);
      const Point point{
        points[anchor].x + query_offset(generator),
        points[anchor].y + query_offset(generator)};
      std::size_t begin = segment_index(generator);
      std::size_t end = segment_index(generator) + 1;
      if (begin > end) {
        std::swap(begin, end);
      }
      end = std::min(end, segments.size());
      const bool skip_unprojectable = duplicate(generator);
      const auto expected = exhaustiveNearest(
        point, points, begin, end, skip_unprojectable);
      const auto actual = query::nearestPoint(
        point.x, point.y, segments, begin, end, skip_unprojectable);
      expectSameNearest(expected, actual);
      EXPECT_LE(actual.exact_distance_evaluations, end - begin);
    }
  }
}

TEST(PolylineQueryTest, ThresholdDecisionsMatchAtExactAndAdjacentValues)
{
  std::mt19937_64 generator(0xc0111510ULL);
  std::uniform_real_distribution<double> coordinate(-8.0, 8.0);
  std::uniform_real_distribution<double> step(-0.4, 0.4);
  const std::vector<double> fixed_thresholds{0.0, 0.24, 0.29, 0.50};

  for (int path_trial = 0; path_trial < 40; ++path_trial) {
    std::vector<Point> points;
    points.push_back(Point{coordinate(generator), coordinate(generator)});
    for (int index = 1; index < 120; ++index) {
      points.push_back(Point{
        points.back().x + 0.10 + step(generator),
        points.back().y + step(generator)});
    }
    if (path_trial % 3 == 0) {
      points[47] = points[46];
    }
    const auto segments = query::prepareSegments(points);
    std::uniform_int_distribution<std::size_t> segment_index(
      0, segments.size() - 1);
    for (int query_trial = 0; query_trial < 80; ++query_trial) {
      const Point point{coordinate(generator), coordinate(generator)};
      std::size_t begin = segment_index(generator);
      std::size_t end = segment_index(generator) + 1;
      if (begin > end) {
        std::swap(begin, end);
      }
      end = std::min(end, segments.size());
      const auto nearest = exhaustiveNearest(
        point, points, begin, end, false);
      std::vector<double> thresholds = fixed_thresholds;
      thresholds.push_back(nearest.distance);
      thresholds.push_back(std::nextafter(
        nearest.distance, -std::numeric_limits<double>::infinity()));
      thresholds.push_back(std::nextafter(
        nearest.distance, std::numeric_limits<double>::infinity()));
      for (const double threshold : thresholds) {
        EXPECT_EQ(
          query::anyWithinDistance(
            point.x, point.y, threshold, segments, begin, end),
          exhaustiveAnyWithinDistance(
            point, points, threshold, begin, end));
      }
    }
  }
}

TEST(PolylineQueryTest, PruningReducesExactWorkOnRepresentativeScanAndPath)
{
  std::vector<Point> path;
  path.reserve(334);
  for (std::size_t index = 0; index < 334; ++index) {
    const double x = 0.03 * static_cast<double>(index);
    path.push_back(Point{x, 0.35 * std::sin(0.45 * x)});
  }
  const auto segments = query::prepareSegments(path);
  std::size_t exact_evaluations = 0;
  std::size_t threshold_exact_evaluations = 0;
  std::size_t exhaustive_evaluations = 0;
  std::size_t threshold_matches = 0;
  constexpr double collision_threshold = 0.29;
  for (std::size_t beam = 0; beam < 1080; ++beam) {
    const double angle = -2.35 + 4.70 * static_cast<double>(beam) / 1079.0;
    const double range = 1.5 + 6.0 * static_cast<double>((beam * 37) % 1080) / 1079.0;
    const Point point{range * std::cos(angle), range * std::sin(angle)};
    const auto expected = exhaustiveNearest(point, path, 0, segments.size(), false);
    const auto actual = query::nearestPoint(point.x, point.y, segments);
    expectSameNearest(expected, actual);
    exact_evaluations += actual.exact_distance_evaluations;
    exhaustive_evaluations += segments.size();

    std::size_t threshold_evaluations = 0;
    const bool expected_within = exhaustiveAnyWithinDistance(
      point, path, collision_threshold, 0, segments.size());
    const bool actual_within = query::anyWithinDistance(
      point.x, point.y, collision_threshold, segments, 0,
      segments.size(), &threshold_evaluations);
    EXPECT_EQ(actual_within, expected_within);
    threshold_exact_evaluations += threshold_evaluations;
    threshold_matches += actual_within ? 1U : 0U;
  }
  RecordProperty("exact_distance_evaluations", exact_evaluations);
  RecordProperty("exhaustive_distance_evaluations", exhaustive_evaluations);
  RecordProperty(
    "exact_evaluation_ratio_ppm",
    static_cast<int>(
      1000000ULL * exact_evaluations / exhaustive_evaluations));
  RecordProperty(
    "threshold_exact_distance_evaluations",
    threshold_exact_evaluations);
  RecordProperty("threshold_positive_queries", threshold_matches);
  EXPECT_LT(exact_evaluations, exhaustive_evaluations * 3 / 4);
  EXPECT_LT(threshold_exact_evaluations, exhaustive_evaluations * 3 / 4);
  EXPECT_GT(threshold_matches, 0U);
  EXPECT_LT(threshold_matches, 1080U);
}

}  // namespace
