#pragma once

namespace path_following_v2::active_path_safety
{

enum class ReplacementFailureDisposition
{
  HOLD_REDUCED_SPEED,
  CONFIRM_PRIMARY_FAILURE,
};

inline ReplacementFailureDisposition replacementFailureDisposition(
  const bool physical_blockage_confirmed,
  const bool path_ended_without_rejoin)
{
  return physical_blockage_confirmed || path_ended_without_rejoin ?
    ReplacementFailureDisposition::CONFIRM_PRIMARY_FAILURE :
    ReplacementFailureDisposition::HOLD_REDUCED_SPEED;
}

}  // namespace path_following_v2::active_path_safety
