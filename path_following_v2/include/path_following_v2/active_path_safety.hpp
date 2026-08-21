#pragma once

#include <algorithm>
#include <cmath>

namespace path_following_v2::active_path_safety
{

enum class ReplacementFailureDisposition
{
  HOLD_REDUCED_SPEED,
  CONFIRM_PRIMARY_FAILURE,
};

struct SafeYieldConfirmation
{
  int consecutive_terminal_scans{0};
  bool primary_failure_confirmed{false};
};

inline ReplacementFailureDisposition replacementFailureDisposition(
  const bool physical_blockage_confirmed,
  const bool path_ended_without_rejoin)
{
  return physical_blockage_confirmed || path_ended_without_rejoin ?
    ReplacementFailureDisposition::CONFIRM_PRIMARY_FAILURE :
    ReplacementFailureDisposition::HOLD_REDUCED_SPEED;
}

inline SafeYieldConfirmation updateSafeYieldConfirmation(
  const int previous_consecutive_terminal_scans,
  const bool fresh_scan,
  const double speed_cap_mps,
  const double terminal_speed_threshold_mps,
  const int required_confirmation_scans)
{
  const int required = std::max(1, required_confirmation_scans);
  const bool terminal = !std::isfinite(speed_cap_mps) ||
    speed_cap_mps <= std::max(0.0, terminal_speed_threshold_mps);
  if (!terminal) {
    return {};
  }

  SafeYieldConfirmation result;
  result.consecutive_terminal_scans = std::clamp(
    previous_consecutive_terminal_scans, 0, required);
  if (fresh_scan) {
    result.consecutive_terminal_scans = std::min(
      result.consecutive_terminal_scans + 1, required);
  }
  result.primary_failure_confirmed =
    result.consecutive_terminal_scans >= required;
  return result;
}

}  // namespace path_following_v2::active_path_safety
