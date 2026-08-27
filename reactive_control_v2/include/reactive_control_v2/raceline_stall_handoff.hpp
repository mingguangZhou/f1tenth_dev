#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace reactive_control_v2::raceline_stall_handoff
{

constexpr uint8_t kRacelineMode = 1;

struct Evidence
{
  bool enabled{false};
  bool arbitration_required{false};
  bool arbitration_mode_healthy{false};
  uint8_t arbitration_mode{0};
  bool command_fresh{false};
  bool command_valid{false};
  bool nominal_mode{false};
  bool scan_valid{false};
  bool odom_healthy{false};
  double requested_forward_speed_mps{0.0};
  double measured_speed_mps{0.0};
  double command_threshold_mps{0.0};
  double stopped_speed_threshold_mps{0.0};
};

inline bool present(const Evidence & evidence)
{
  return evidence.enabled && evidence.arbitration_required &&
    evidence.arbitration_mode_healthy &&
    evidence.arbitration_mode == kRacelineMode &&
    evidence.command_fresh && evidence.command_valid && evidence.nominal_mode &&
    evidence.scan_valid && evidence.odom_healthy &&
    std::isfinite(evidence.requested_forward_speed_mps) &&
    std::isfinite(evidence.measured_speed_mps) &&
    evidence.requested_forward_speed_mps >=
    std::max(0.0, evidence.command_threshold_mps) &&
    std::abs(evidence.measured_speed_mps) <=
    std::max(0.0, evidence.stopped_speed_threshold_mps);
}

inline bool confirmationReached(
  const bool timer_active, const double elapsed_sec,
  const double required_confirmation_sec)
{
  return timer_active && std::isfinite(elapsed_sec) && elapsed_sec >=
    std::max(0.0, required_confirmation_sec);
}

inline bool requestConfirmed(
  const Evidence & evidence, const bool timer_active,
  const double elapsed_sec, const double required_confirmation_sec)
{
  return present(evidence) && confirmationReached(
    timer_active, elapsed_sec, required_confirmation_sec);
}

inline bool modeContinuityBroken(
  const bool previous_mode_valid, const uint8_t previous_mode,
  const bool current_mode_healthy, const uint8_t current_mode)
{
  return !previous_mode_valid || !current_mode_healthy ||
    previous_mode != current_mode;
}

inline bool updatePersistentLatch(
  const bool currently_latched, const bool raceline_stall_confirmed,
  const bool sustained_forward_progress)
{
  if (raceline_stall_confirmed) {
    return true;
  }
  if (currently_latched && sustained_forward_progress) {
    return false;
  }
  return currently_latched;
}

}  // namespace reactive_control_v2::raceline_stall_handoff
