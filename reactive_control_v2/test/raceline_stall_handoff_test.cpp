#include <limits>

#include "gtest/gtest.h"

#include "reactive_control_v2/raceline_stall_handoff.hpp"

namespace handoff = reactive_control_v2::raceline_stall_handoff;

namespace
{
handoff::Evidence validEvidence()
{
  handoff::Evidence evidence;
  evidence.enabled = true;
  evidence.arbitration_required = true;
  evidence.arbitration_mode_healthy = true;
  evidence.arbitration_mode = handoff::kRacelineMode;
  evidence.command_fresh = true;
  evidence.command_valid = true;
  evidence.nominal_mode = true;
  evidence.scan_valid = true;
  evidence.odom_healthy = true;
  evidence.requested_forward_speed_mps = 0.50;
  evidence.measured_speed_mps = 0.0;
  evidence.command_threshold_mps = 0.20;
  evidence.stopped_speed_threshold_mps = 0.05;
  return evidence;
}
}  // namespace

TEST(RacelineStallHandoff, RequiresTheCompleteHealthyRacelineContract)
{
  const auto valid = validEvidence();
  EXPECT_TRUE(handoff::present(valid));

  auto evidence = valid;
  evidence.enabled = false;
  EXPECT_FALSE(handoff::present(evidence));
  evidence = valid;
  evidence.arbitration_required = false;
  EXPECT_FALSE(handoff::present(evidence));
  evidence = valid;
  evidence.arbitration_mode_healthy = false;
  EXPECT_FALSE(handoff::present(evidence));
  evidence = valid;
  evidence.arbitration_mode = 0;
  EXPECT_FALSE(handoff::present(evidence));
  evidence = valid;
  evidence.command_fresh = false;
  EXPECT_FALSE(handoff::present(evidence));
  evidence = valid;
  evidence.command_valid = false;
  EXPECT_FALSE(handoff::present(evidence));
  evidence = valid;
  evidence.nominal_mode = false;
  EXPECT_FALSE(handoff::present(evidence));
  evidence = valid;
  evidence.scan_valid = false;
  EXPECT_FALSE(handoff::present(evidence));
  evidence = valid;
  evidence.odom_healthy = false;
  EXPECT_FALSE(handoff::present(evidence));
}

TEST(RacelineStallHandoff, RejectsPlannedStopsAndMeasuredMotion)
{
  const auto valid = validEvidence();

  auto evidence = valid;
  evidence.requested_forward_speed_mps = 0.19;
  EXPECT_FALSE(handoff::present(evidence));
  evidence = valid;
  evidence.measured_speed_mps = 0.06;
  EXPECT_FALSE(handoff::present(evidence));
  evidence = valid;
  evidence.requested_forward_speed_mps =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(handoff::present(evidence));
  evidence = valid;
  evidence.measured_speed_mps = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(handoff::present(evidence));
}

TEST(RacelineStallHandoff, RequiresTheConfiguredContinuousDuration)
{
  const auto evidence = validEvidence();
  EXPECT_FALSE(handoff::confirmationReached(false, 2.0, 1.0));
  EXPECT_FALSE(handoff::confirmationReached(true, 0.99, 1.0));
  EXPECT_TRUE(handoff::confirmationReached(true, 1.0, 1.0));
  EXPECT_FALSE(handoff::requestConfirmed(evidence, true, 0.99, 1.0));
  EXPECT_TRUE(handoff::requestConfirmed(evidence, true, 1.0, 1.0));

  auto braking = evidence;
  braking.requested_forward_speed_mps = 0.0;
  EXPECT_FALSE(handoff::requestConfirmed(braking, true, 2.0, 1.0));
}

TEST(RacelineStallHandoff, ArbitrationModeChangesBreakContinuity)
{
  EXPECT_FALSE(handoff::modeContinuityBroken(true, 1, true, 1));
  EXPECT_TRUE(handoff::modeContinuityBroken(true, 1, true, 2));
  EXPECT_TRUE(handoff::modeContinuityBroken(true, 1, false, 1));
  EXPECT_TRUE(handoff::modeContinuityBroken(false, 1, true, 1));
}

TEST(RacelineStallHandoff, PersistentLatchRequiresSustainedForwardProgressToClear)
{
  bool latched = handoff::updatePersistentLatch(false, true, false);
  EXPECT_TRUE(latched);

  latched = handoff::updatePersistentLatch(latched, false, false);
  EXPECT_TRUE(latched);
  latched = handoff::updatePersistentLatch(latched, false, true);
  EXPECT_FALSE(latched);
}
