#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "path_following_v2/scan_pooling.hpp"

namespace
{

using path_following_v2::scan_pooling::RangeLimits;
using path_following_v2::scan_pooling::minimumRangePool;
using path_following_v2::scan_pooling::minimumRangePoolProjectedHits;

struct ProjectedHit
{
  std::size_t beam_index{0};
  std::size_t support_count{1};
  double range{0.0};
  int payload{0};
};

TEST(ScanPooling, FullResolutionPreservesEveryUsableSampleExactly)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();
  const std::vector<float> ranges{1.0F, infinity, nan, 0.09F, 2.5F, 5.1F, 0.1F, 5.0F};

  const auto result = minimumRangePool(ranges, RangeLimits{0.1, 5.0}, 1);

  ASSERT_EQ(result.raw_beam_count, ranges.size());
  EXPECT_EQ(result.valid_beam_count, 6U);
  EXPECT_EQ(result.pool_size, 1U);
  ASSERT_EQ(result.beams.size(), 4U);
  const std::vector<std::size_t> expected_indices{0, 4, 6, 7};
  for (std::size_t output = 0; output < result.beams.size(); ++output) {
    const auto raw = expected_indices[output];
    EXPECT_EQ(result.beams[output].bin_start_index, raw);
    EXPECT_EQ(result.beams[output].source_index, raw);
    EXPECT_EQ(result.beams[output].support_count, 1U);
    EXPECT_DOUBLE_EQ(result.beams[output].range_m, static_cast<double>(ranges[raw]));
  }
}

TEST(ScanPooling, SelectsMinimumUsableBeamAndReportsRawSupport)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();
  const std::vector<float> ranges{3.0F, 2.0F, infinity, 1.0F, 0.2F, nan, 5.0F, 4.0F};

  const auto result = minimumRangePool(ranges, RangeLimits{0.1, 4.5}, 2);

  EXPECT_EQ(result.valid_beam_count, 7U);
  ASSERT_EQ(result.beams.size(), 4U);

  EXPECT_EQ(result.beams[0].bin_start_index, 0U);
  EXPECT_EQ(result.beams[0].source_index, 1U);
  EXPECT_EQ(result.beams[0].support_count, 2U);
  EXPECT_DOUBLE_EQ(result.beams[0].range_m, 2.0);

  EXPECT_EQ(result.beams[1].bin_start_index, 2U);
  EXPECT_EQ(result.beams[1].source_index, 3U);
  EXPECT_EQ(result.beams[1].support_count, 1U);
  EXPECT_DOUBLE_EQ(result.beams[1].range_m, 1.0);

  EXPECT_EQ(result.beams[2].bin_start_index, 4U);
  EXPECT_EQ(result.beams[2].source_index, 4U);
  EXPECT_EQ(result.beams[2].support_count, 1U);
  EXPECT_DOUBLE_EQ(result.beams[2].range_m, static_cast<double>(ranges[4]));

  EXPECT_EQ(result.beams[3].bin_start_index, 6U);
  EXPECT_EQ(result.beams[3].source_index, 7U);
  EXPECT_EQ(result.beams[3].support_count, 1U);
  EXPECT_DOUBLE_EQ(result.beams[3].range_m, 4.0);
}

TEST(ScanPooling, KeepsEarliestRawIndexWhenMinimumRangesTie)
{
  const std::vector<double> ranges{2.0, 1.0, 1.0, 3.0};

  const auto result = minimumRangePool(ranges, RangeLimits{0.0, 5.0}, 4);

  ASSERT_EQ(result.beams.size(), 1U);
  EXPECT_EQ(result.beams.front().bin_start_index, 0U);
  EXPECT_EQ(result.beams.front().source_index, 1U);
  EXPECT_EQ(result.beams.front().support_count, 4U);
  EXPECT_DOUBLE_EQ(result.beams.front().range_m, 1.0);
}

TEST(ScanPooling, HandlesPartialFinalBinAndBinsWithoutUsableReturns)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const std::vector<double> ranges{nan, infinity, 8.0, 4.0, 3.0};

  const auto result = minimumRangePool(ranges, RangeLimits{0.1, 5.0}, 3);

  EXPECT_EQ(result.raw_beam_count, 5U);
  EXPECT_EQ(result.valid_beam_count, 4U);
  ASSERT_EQ(result.beams.size(), 1U);
  EXPECT_EQ(result.beams.front().bin_start_index, 3U);
  EXPECT_EQ(result.beams.front().source_index, 4U);
  EXPECT_EQ(result.beams.front().support_count, 2U);
  EXPECT_DOUBLE_EQ(result.beams.front().range_m, 3.0);
  EXPECT_DOUBLE_EQ(result.validBeamRatio(), 0.8);
}

TEST(ScanPooling, ValidBeamCountDoesNotDependOnPoolSize)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();
  const std::vector<float> ranges{nan, infinity, -1.0F, 0.1F, 1.0F, 10.0F, 2.0F};
  const RangeLimits limits{0.1, 5.0};

  const auto full = minimumRangePool(ranges, limits, 1);
  const auto paired = minimumRangePool(ranges, limits, 2);
  const auto wide = minimumRangePool(ranges, limits, 16);

  EXPECT_EQ(full.valid_beam_count, 5U);
  EXPECT_EQ(paired.valid_beam_count, full.valid_beam_count);
  EXPECT_EQ(wide.valid_beam_count, full.valid_beam_count);
  EXPECT_DOUBLE_EQ(paired.validBeamRatio(), full.validBeamRatio());
  EXPECT_DOUBLE_EQ(wide.validBeamRatio(), full.validBeamRatio());
}

TEST(ScanPooling, ZeroPoolSizeFallsBackToFullResolution)
{
  const std::vector<double> ranges{3.0, 2.0, 1.0};

  const auto result = minimumRangePool(ranges, RangeLimits{0.0, 5.0}, 0);

  EXPECT_EQ(result.pool_size, 1U);
  ASSERT_EQ(result.beams.size(), ranges.size());
  for (std::size_t index = 0; index < ranges.size(); ++index) {
    EXPECT_EQ(result.beams[index].source_index, index);
    EXPECT_DOUBLE_EQ(result.beams[index].range_m, ranges[index]);
  }
}

TEST(ScanPooling, EmptyInputHasZeroValidityRatio)
{
  const std::vector<float> ranges;

  const auto result = minimumRangePool(ranges, RangeLimits{0.0, 5.0}, 2);

  EXPECT_TRUE(result.beams.empty());
  EXPECT_EQ(result.raw_beam_count, 0U);
  EXPECT_EQ(result.valid_beam_count, 0U);
  EXPECT_DOUBLE_EQ(result.validBeamRatio(), 0.0);
}

TEST(ScanPooling, ProjectedHitsGroupByRawBinAndAccumulateSupportAtMinimum)
{
  const std::vector<ProjectedHit> hits{
    {0U, 1U, 3.0, 10},
    {1U, 2U, 2.0, 11},
    {2U, 4U, 5.0, 12},
    {3U, 1U, 1.0, 13},
  };

  const auto pooled = minimumRangePoolProjectedHits(hits, 2);

  ASSERT_EQ(pooled.size(), 2U);
  EXPECT_EQ(pooled[0].beam_index, 0U);
  EXPECT_EQ(pooled[0].support_count, 3U);
  EXPECT_DOUBLE_EQ(pooled[0].range, 2.0);
  EXPECT_EQ(pooled[0].payload, 11);
  EXPECT_EQ(pooled[1].beam_index, 1U);
  EXPECT_EQ(pooled[1].support_count, 5U);
  EXPECT_DOUBLE_EQ(pooled[1].range, 1.0);
  EXPECT_EQ(pooled[1].payload, 13);
}

TEST(ScanPooling, ProjectedHitsPreserveGapsAsPooledBinOrdinals)
{
  const std::vector<ProjectedHit> hits{
    {1U, 1U, 2.0, 20},
    {6U, 1U, 3.0, 21},
    {7U, 1U, 2.5, 22},
  };

  const auto pooled = minimumRangePoolProjectedHits(hits, 3);

  ASSERT_EQ(pooled.size(), 2U);
  EXPECT_EQ(pooled[0].beam_index, 0U);
  EXPECT_EQ(pooled[0].payload, 20);
  EXPECT_EQ(pooled[1].beam_index, 2U);
  EXPECT_EQ(pooled[1].support_count, 2U);
  EXPECT_DOUBLE_EQ(pooled[1].range, 2.5);
  EXPECT_EQ(pooled[1].payload, 22);
}

TEST(ScanPooling, ProjectedHitsKeepEarliestPayloadOnTiedMinimum)
{
  const std::vector<ProjectedHit> hits{
    {4U, 2U, 1.0, 30},
    {5U, 3U, 1.0, 31},
  };

  const auto pooled = minimumRangePoolProjectedHits(hits, 2);

  ASSERT_EQ(pooled.size(), 1U);
  EXPECT_EQ(pooled.front().beam_index, 2U);
  EXPECT_EQ(pooled.front().support_count, 5U);
  EXPECT_DOUBLE_EQ(pooled.front().range, 1.0);
  EXPECT_EQ(pooled.front().payload, 30);
}

}  // namespace
