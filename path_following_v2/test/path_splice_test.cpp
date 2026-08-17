#include <cmath>
#include <vector>

#include "gtest/gtest.h"

#include "path_following_v2/path_splice.hpp"

namespace splice = path_following_v2::path_splice;

TEST(PathSplice, AcceptsAContinuousRacelineTail)
{
  const splice::Point2 previous{-0.05, 0.0};
  const splice::Point2 anchor{0.0, 0.0};
  const std::vector<splice::Point2> raw{{0.0, 0.0}, {0.03, 0.0}, {0.06, 0.0}};

  const auto match = splice::findContinuousTailStart(
    previous, anchor, raw, 0.15, 15.0 * M_PI / 180.0, 1.08);

  ASSERT_TRUE(match.valid);
  EXPECT_EQ(match.raw_start_index, 1U);
  EXPECT_NEAR(match.connection_distance_m, 0.03, 1e-9);
  EXPECT_NEAR(match.maximum_curvature_inv_m, 0.0, 1e-9);
}

TEST(PathSplice, RejectsAVisibleGapToAnUnavailableTail)
{
  const splice::Point2 previous{-0.05, 0.0};
  const splice::Point2 anchor{0.0, 0.0};
  const std::vector<splice::Point2> raw{{0.49, 0.10}, {0.52, 0.10}, {0.55, 0.10}};

  const auto match = splice::findContinuousTailStart(
    previous, anchor, raw, 0.15, 15.0 * M_PI / 180.0, 1.08);

  EXPECT_FALSE(match.valid);
}

TEST(PathSplice, RejectsAHeadingDiscontinuityAtANearbyBranch)
{
  const splice::Point2 previous{-0.05, 0.0};
  const splice::Point2 anchor{0.0, 0.0};
  const std::vector<splice::Point2> raw{{0.0, 0.04}, {0.0, 0.07}, {0.0, 0.10}};

  const auto match = splice::findContinuousTailStart(
    previous, anchor, raw, 0.15, 15.0 * M_PI / 180.0, 1.08);

  EXPECT_FALSE(match.valid);
}
