#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace path_following_v2::active_path_safety
{

enum class ReplacementFailureDisposition
{
  HOLD_REDUCED_SPEED,
  CONFIRM_PRIMARY_FAILURE,
};

enum class CandidateValidationOutcome
{
  CLEAR,
  COLLISION,
  INPUT_FAILURE,
};

enum class ActiveCandidateDisposition
{
  ACTIVATE_REPLACEMENT,
  RETAIN_ACTIVE_PATH,
  CONTINUE_FAILURE_HANDLING,
  FAIL_PRIMARY,
};

struct SafeYieldConfirmation
{
  int consecutive_terminal_scans{0};
  bool primary_failure_confirmed{false};
};

struct ScanConfirmationState
{
  int consecutive_scans{0};
  std::int64_t last_evidence_scan_id{0};
};

struct ScanConfirmationUpdate
{
  ScanConfirmationState state;
  bool consumed{false};
  bool confirmed{false};
};

inline ReplacementFailureDisposition replacementFailureDisposition(
  const bool physical_blockage_confirmed,
  const bool path_ended_without_rejoin)
{
  return physical_blockage_confirmed || path_ended_without_rejoin ?
    ReplacementFailureDisposition::CONFIRM_PRIMARY_FAILURE :
    ReplacementFailureDisposition::HOLD_REDUCED_SPEED;
}

inline ActiveCandidateDisposition activeCandidateDisposition(
  const CandidateValidationOutcome candidate_outcome,
  const bool held_path_physically_clear,
  const bool held_path_ended_without_rejoin)
{
  if (candidate_outcome == CandidateValidationOutcome::CLEAR) {
    return ActiveCandidateDisposition::ACTIVATE_REPLACEMENT;
  }
  if (candidate_outcome == CandidateValidationOutcome::COLLISION &&
    held_path_physically_clear && !held_path_ended_without_rejoin)
  {
    return ActiveCandidateDisposition::RETAIN_ACTIVE_PATH;
  }
  if (candidate_outcome == CandidateValidationOutcome::COLLISION) {
    return ActiveCandidateDisposition::CONTINUE_FAILURE_HANDLING;
  }
  return ActiveCandidateDisposition::FAIL_PRIMARY;
}

inline ScanConfirmationUpdate updateDistinctScanConfirmation(
  const ScanConfirmationState & previous,
  const std::int64_t evidence_scan_id,
  const bool condition_present,
  const int required_confirmation_scans)
{
  const int required = std::max(1, required_confirmation_scans);
  ScanConfirmationUpdate result;
  result.state.consecutive_scans = std::clamp(
    previous.consecutive_scans, 0, required);
  result.state.last_evidence_scan_id = previous.last_evidence_scan_id;
  if (evidence_scan_id <= 0 ||
    evidence_scan_id <= previous.last_evidence_scan_id)
  {
    result.confirmed = result.state.consecutive_scans >= required;
    return result;
  }

  result.consumed = true;
  result.state.last_evidence_scan_id = evidence_scan_id;
  result.state.consecutive_scans = condition_present ?
    std::min(result.state.consecutive_scans + 1, required) : 0;
  result.confirmed = result.state.consecutive_scans >= required;
  return result;
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
