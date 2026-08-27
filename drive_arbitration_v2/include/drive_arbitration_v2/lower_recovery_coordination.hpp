#pragma once

#include <string>

namespace drive_arbitration_v2::lower_recovery_coordination
{

inline std::string holdReason(
  const bool enabled, const bool status_fresh,
  const std::string & arbitration_mode, const std::string & lower_state,
  const bool raceline_stall_handoff_latched)
{
  if (!enabled) {
    return "";
  }
  if (!status_fresh) {
    return "lower safety status missing or stale";
  }
  if (arbitration_mode != "2") {
    return "lower safety has not confirmed REACTIVE arbitration mode";
  }
  if (lower_state != "NOMINAL") {
    return "lower safety mode=" + lower_state;
  }
  if (raceline_stall_handoff_latched) {
    return "lower safety raceline stall handoff remains latched pending forward progress";
  }
  return "";
}

}  // namespace drive_arbitration_v2::lower_recovery_coordination
