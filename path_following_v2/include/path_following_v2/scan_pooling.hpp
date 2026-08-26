#ifndef PATH_FOLLOWING_V2__SCAN_POOLING_HPP_
#define PATH_FOLLOWING_V2__SCAN_POOLING_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace path_following_v2
{
namespace scan_pooling
{

// ROS-independent limits for deciding whether a raw range can represent an
// obstacle hit. Callers can resolve sensor-specific range caps before invoking
// this helper.
struct RangeLimits
{
  double minimum_m{0.0};
  double maximum_m{std::numeric_limits<double>::infinity()};
};

struct PooledBeam
{
  // First raw beam covered by this fixed-size bin.
  std::size_t bin_start_index{0};
  // Raw index of the minimum usable range. This preserves the exact angle
  // needed to reconstruct the selected endpoint.
  std::size_t source_index{0};
  // Number of finite, in-range raw beams in the bin. Consumers can retain
  // raw evidence-count semantics even though only one endpoint is projected.
  std::size_t support_count{0};
  double range_m{0.0};
};

struct Result
{
  std::vector<PooledBeam> beams;
  std::size_t raw_beam_count{0};
  std::size_t valid_beam_count{0};
  std::size_t pool_size{1};

  double validBeamRatio() const
  {
    return raw_beam_count == 0 ? 0.0 :
           static_cast<double>(valid_beam_count) /
           static_cast<double>(raw_beam_count);
  }
};

// Select the nearest finite, usable return from every fixed-size raw bin.
//
// The validity count deliberately matches the local trajectory planner's
// existing LaserScan health rule: non-NaN readings at or above minimum_m are
// valid, including +infinity and finite readings beyond maximum_m. Such
// readings remain unavailable as obstacle endpoints. A zero pool size is
// normalized to one so a misconfigured caller cannot create a non-progressing
// real-time loop.
template<typename RangeContainer>
Result minimumRangePool(
  const RangeContainer & ranges, const RangeLimits & limits,
  const std::size_t requested_pool_size)
{
  Result result;
  result.raw_beam_count = ranges.size();
  result.pool_size = std::max<std::size_t>(1, requested_pool_size);
  result.beams.reserve(
    (result.raw_beam_count + result.pool_size - 1) / result.pool_size);

  for (std::size_t bin_start = 0;
    bin_start < result.raw_beam_count; bin_start += result.pool_size)
  {
    const std::size_t bin_end = std::min(
      result.raw_beam_count, bin_start + result.pool_size);
    PooledBeam pooled;
    pooled.bin_start_index = bin_start;
    double minimum_range = std::numeric_limits<double>::infinity();

    for (std::size_t index = bin_start; index < bin_end; ++index) {
      const double range = static_cast<double>(ranges[index]);
      if (!std::isnan(range) && range >= limits.minimum_m) {
        ++result.valid_beam_count;
      }
      if (!std::isfinite(range) || range < limits.minimum_m ||
        range > limits.maximum_m)
      {
        continue;
      }

      ++pooled.support_count;
      // Strict comparison retains the earliest raw beam when ranges tie.
      if (range < minimum_range) {
        minimum_range = range;
        pooled.source_index = index;
        pooled.range_m = range;
      }
    }

    if (pooled.support_count > 0) {
      result.beams.push_back(pooled);
    }
  }
  return result;
}

// Pool endpoints that have already been transformed/projected by the caller.
// HitContainer::value_type must expose beam_index, support_count, and range.
// Input hits are expected in ascending raw beam order. The selected endpoint
// retains every other field from the minimum-range hit, while beam_index is
// rewritten to the pooled-bin ordinal and support_count accumulates all usable
// evidence represented by that bin.
template<typename HitContainer>
std::vector<typename HitContainer::value_type> minimumRangePoolProjectedHits(
  const HitContainer & full_resolution_hits,
  const std::size_t requested_pool_size)
{
  using Hit = typename HitContainer::value_type;
  const std::size_t pool_size = std::max<std::size_t>(1, requested_pool_size);
  if (pool_size == 1 || full_resolution_hits.empty()) {
    return std::vector<Hit>(full_resolution_hits.begin(), full_resolution_hits.end());
  }

  std::vector<Hit> pooled;
  pooled.reserve((full_resolution_hits.size() + pool_size - 1) / pool_size);
  std::size_t current_bin = std::numeric_limits<std::size_t>::max();
  for (const auto & hit : full_resolution_hits) {
    const std::size_t bin = hit.beam_index / pool_size;
    if (pooled.empty() || bin != current_bin) {
      pooled.push_back(hit);
      pooled.back().beam_index = bin;
      current_bin = bin;
      continue;
    }

    auto & selected = pooled.back();
    const std::size_t accumulated_support =
      selected.support_count + hit.support_count;
    // Strict comparison preserves the earliest transformed endpoint on ties,
    // matching minimumRangePool's raw-range behavior.
    if (hit.range < selected.range) {
      selected = hit;
      selected.beam_index = bin;
    }
    selected.support_count = accumulated_support;
  }
  return pooled;
}

}  // namespace scan_pooling
}  // namespace path_following_v2

#endif  // PATH_FOLLOWING_V2__SCAN_POOLING_HPP_
