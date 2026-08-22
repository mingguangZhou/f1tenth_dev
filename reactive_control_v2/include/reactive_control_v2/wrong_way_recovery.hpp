#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace reactive_control_v2::wrong_way_recovery
{

constexpr uint8_t kRacelineMode = 1;
constexpr uint8_t kReactiveMode = 2;

struct Evidence
{
  bool enabled{false};
  bool arbitration_required{false};
  bool arbitration_mode_healthy{false};
  uint8_t arbitration_mode{0};
  bool heading_sample_fresh{false};
  bool path_association_valid{false};
  bool scan_valid{false};
  bool odom_healthy{false};
  bool positive_forward_request{false};
  double heading_error_rad{0.0};
  double entry_angle_rad{0.0};
};

inline bool currentGeometryValid(const Evidence & evidence)
{
  return evidence.enabled && evidence.arbitration_required &&
         evidence.arbitration_mode_healthy &&
         (evidence.arbitration_mode == kRacelineMode ||
         evidence.arbitration_mode == kReactiveMode) &&
         evidence.heading_sample_fresh && evidence.path_association_valid &&
         evidence.scan_valid &&
         evidence.odom_healthy && std::isfinite(evidence.heading_error_rad);
}

inline bool reverseAuthorityValid(const Evidence & evidence)
{
  return currentGeometryValid(evidence) &&
         evidence.arbitration_mode == kReactiveMode;
}

inline bool reverseHeadingPresent(
  const Evidence & evidence, const double threshold_rad)
{
  return currentGeometryValid(evidence) &&
         std::abs(evidence.heading_error_rad) >= std::max(0.0, threshold_rad);
}

inline bool entryPresent(const Evidence & evidence)
{
  return evidence.positive_forward_request &&
         reverseHeadingPresent(evidence, evidence.entry_angle_rad);
}

inline bool suspicionConfirmationPresent(
  const bool suspicion_latched, const Evidence & evidence,
  const double exit_angle_rad)
{
  return suspicion_latched && currentGeometryValid(evidence) &&
         std::abs(evidence.heading_error_rad) > std::max(0.0, exit_angle_rad);
}

inline bool forwardDirectionUnverified(const Evidence & evidence)
{
  const bool selected_forward_mode =
    evidence.arbitration_mode == kRacelineMode ||
    evidence.arbitration_mode == kReactiveMode;
  return evidence.enabled && evidence.arbitration_required &&
         evidence.arbitration_mode_healthy && selected_forward_mode &&
         evidence.scan_valid && evidence.positive_forward_request &&
         (!evidence.odom_healthy || !evidence.heading_sample_fresh ||
         !evidence.path_association_valid ||
         !std::isfinite(evidence.heading_error_rad));
}

inline bool correctionStillRequired(
  const bool latched, const Evidence & evidence, const double exit_angle_rad)
{
  return latched && currentGeometryValid(evidence) &&
         std::abs(evidence.heading_error_rad) > std::max(0.0, exit_angle_rad);
}

inline bool alignmentRecovered(
  const Evidence & evidence, const double exit_angle_rad)
{
  return currentGeometryValid(evidence) &&
         std::abs(evidence.heading_error_rad) <= std::max(0.0, exit_angle_rad);
}

inline bool alignmentReleaseWindow(
  const bool recovery_latched, const bool alignment_confirmed,
  const Evidence & evidence, const double reentry_angle_rad)
{
  return recovery_latched && alignment_confirmed && currentGeometryValid(evidence) &&
         std::abs(evidence.heading_error_rad) < std::max(0.0, reentry_angle_rad);
}

inline bool confirmationReached(
  const bool timer_active, const double elapsed_sec,
  const double confirmation_sec, const int distinct_sample_count,
  const int required_distinct_samples)
{
  return timer_active && std::isfinite(elapsed_sec) &&
         elapsed_sec >= std::max(0.0, confirmation_sec) &&
         distinct_sample_count >= std::max(1, required_distinct_samples);
}

inline int chooseTurnSign(
  const double heading_error_rad, const double left_clearance_m,
  const double right_clearance_m, const double ambiguity_angle_rad)
{
  if (!std::isfinite(heading_error_rad)) {
    return 0;
  }
  if (std::abs(heading_error_rad) >= std::max(0.0, ambiguity_angle_rad)) {
    const bool left_valid = !std::isnan(left_clearance_m) && left_clearance_m >= 0.0;
    const bool right_valid = !std::isnan(right_clearance_m) && right_clearance_m >= 0.0;
    if (left_valid && right_valid) {
      if (left_clearance_m > right_clearance_m + 1e-6) {
        return 1;
      }
      if (right_clearance_m > left_clearance_m + 1e-6) {
        return -1;
      }
    }
    // Make the exact +/-pi representation deterministic when both observed
    // sides are equivalently clear.
    return 1;
  }
  return heading_error_rad >= 0.0 ? 1 : -1;
}

inline int retainEpisodeTurnSign(
  const int existing_turn_sign, const double heading_error_rad,
  const double left_clearance_m, const double right_clearance_m,
  const double ambiguity_angle_rad)
{
  if (existing_turn_sign == -1 || existing_turn_sign == 1) {
    return existing_turn_sign;
  }
  return chooseTurnSign(
    heading_error_rad, left_clearance_m, right_clearance_m,
    ambiguity_angle_rad);
}

inline double reverseSteeringAngle(
  const int turn_sign, const double steering_limit_rad)
{
  if (turn_sign == 0 || !std::isfinite(steering_limit_rad)) {
    return 0.0;
  }
  return static_cast<double>(turn_sign) * std::max(0.0, steering_limit_rad);
}

inline bool selectedReverseSideClear(
  const int turn_sign, const double left_valid_ratio,
  const double right_valid_ratio, const double minimum_valid_ratio,
  const double left_clearance_m, const double right_clearance_m,
  const double minimum_clearance_m)
{
  if (turn_sign != -1 && turn_sign != 1) {
    return false;
  }
  const double selected_valid_ratio = turn_sign > 0 ?
    left_valid_ratio : right_valid_ratio;
  const double selected_clearance_m = turn_sign > 0 ?
    left_clearance_m : right_clearance_m;
  return std::isfinite(selected_valid_ratio) && selected_valid_ratio >= 0.0 &&
         selected_valid_ratio >= std::max(0.0, minimum_valid_ratio) &&
         !std::isnan(selected_clearance_m) && selected_clearance_m >= 0.0 &&
         selected_clearance_m >= std::max(0.0, minimum_clearance_m);
}

inline bool updatePersistentLatch(
  const bool currently_latched, const bool entry_confirmed,
  const bool alignment_recovered, const bool sustained_forward_progress)
{
  if (entry_confirmed) {
    return true;
  }
  if (currently_latched && alignment_recovered && sustained_forward_progress) {
    return false;
  }
  return currently_latched;
}

}  // namespace reactive_control_v2::wrong_way_recovery
