#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "reactive_control_v2/wrong_way_recovery.hpp"

namespace recovery = reactive_control_v2::wrong_way_recovery;

namespace
{
constexpr double kPi = 3.14159265358979323846;

recovery::Evidence validEvidence(const double heading_error_deg = 130.0)
{
  recovery::Evidence evidence;
  evidence.enabled = true;
  evidence.arbitration_required = true;
  evidence.arbitration_mode_healthy = true;
  evidence.arbitration_mode = recovery::kReactiveMode;
  evidence.heading_sample_fresh = true;
  evidence.path_association_valid = true;
  evidence.scan_valid = true;
  evidence.odom_healthy = true;
  evidence.positive_forward_request = true;
  evidence.heading_error_rad = heading_error_deg * kPi / 180.0;
  evidence.entry_angle_rad = 90.0 * kPi / 180.0;
  return evidence;
}
}  // namespace

TEST(WrongWayRecovery, RequiresFreshAuthorizedSelectedModeGeometry)
{
  const auto valid = validEvidence();
  EXPECT_TRUE(recovery::entryPresent(valid));

  auto input = valid;
  input.enabled = false;
  EXPECT_FALSE(recovery::entryPresent(input));
  input = valid;
  input.arbitration_required = false;
  EXPECT_FALSE(recovery::entryPresent(input));
  input = valid;
  input.arbitration_mode_healthy = false;
  EXPECT_FALSE(recovery::entryPresent(input));
  input = valid;
  input.arbitration_mode = 0;
  EXPECT_FALSE(recovery::entryPresent(input));
  input = valid;
  input.arbitration_mode = recovery::kRacelineMode;
  EXPECT_TRUE(recovery::entryPresent(input));
  EXPECT_FALSE(recovery::reverseAuthorityValid(input));
  EXPECT_TRUE(recovery::reverseAuthorityValid(valid));
  input = valid;
  input.heading_sample_fresh = false;
  EXPECT_FALSE(recovery::entryPresent(input));
  input = valid;
  input.path_association_valid = false;
  EXPECT_FALSE(recovery::entryPresent(input));
  input = valid;
  input.scan_valid = false;
  EXPECT_FALSE(recovery::entryPresent(input));
  input = valid;
  input.odom_healthy = false;
  EXPECT_FALSE(recovery::entryPresent(input));
  input = valid;
  input.positive_forward_request = false;
  EXPECT_FALSE(recovery::entryPresent(input));
  input = valid;
  input.heading_error_rad = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(recovery::entryPresent(input));
}

TEST(WrongWayRecovery, UsesEntryAndExitHysteresis)
{
  EXPECT_FALSE(recovery::entryPresent(validEvidence(89.99)));
  EXPECT_TRUE(recovery::entryPresent(validEvidence(90.0)));
  EXPECT_TRUE(recovery::entryPresent(validEvidence(-130.0)));

  auto input = validEvidence(81.0);
  EXPECT_TRUE(
    recovery::correctionStillRequired(
      true, input, 80.0 * kPi / 180.0));
  EXPECT_FALSE(recovery::alignmentRecovered(input, 80.0 * kPi / 180.0));
  input = validEvidence(80.0);
  EXPECT_FALSE(
    recovery::correctionStillRequired(
      true, input, 80.0 * kPi / 180.0));
  EXPECT_TRUE(recovery::alignmentRecovered(input, 80.0 * kPi / 180.0));
}

TEST(WrongWayRecovery, LatchedSuspicionConfirmsUntilExitHysteresisIsReached)
{
  const auto below_entry = validEvidence(89.0);
  EXPECT_FALSE(recovery::entryPresent(below_entry));
  EXPECT_TRUE(
    recovery::suspicionConfirmationPresent(
      true, below_entry, 80.0 * kPi / 180.0));

  const auto recovered = validEvidence(80.0);
  EXPECT_FALSE(
    recovery::suspicionConfirmationPresent(
      true, recovered, 80.0 * kPi / 180.0));
  EXPECT_FALSE(
    recovery::suspicionConfirmationPresent(
      false, validEvidence(130.0), 80.0 * kPi / 180.0));
}

TEST(WrongWayRecovery, PositiveMotionWaitsForFreshDirectionGeometry)
{
  auto input = validEvidence(0.0);
  EXPECT_FALSE(recovery::forwardDirectionUnverified(input));

  input.heading_sample_fresh = false;
  EXPECT_TRUE(recovery::forwardDirectionUnverified(input));
  input = validEvidence(0.0);
  input.path_association_valid = false;
  EXPECT_TRUE(recovery::forwardDirectionUnverified(input));
  input = validEvidence(0.0);
  input.odom_healthy = false;
  EXPECT_TRUE(recovery::forwardDirectionUnverified(input));
  input = validEvidence(0.0);
  input.heading_error_rad = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(recovery::forwardDirectionUnverified(input));
  input = validEvidence(0.0);
  input.positive_forward_request = false;
  input.heading_sample_fresh = false;
  EXPECT_FALSE(recovery::forwardDirectionUnverified(input));
  input = validEvidence(0.0);
  input.arbitration_mode = 0;
  input.odom_healthy = false;
  input.heading_sample_fresh = false;
  EXPECT_FALSE(recovery::forwardDirectionUnverified(input));
}

TEST(WrongWayRecovery, RequiresContinuousConfirmation)
{
  EXPECT_FALSE(recovery::confirmationReached(false, 1.0, 0.2, 5, 3));
  EXPECT_FALSE(recovery::confirmationReached(true, 0.199, 0.2, 5, 3));
  EXPECT_FALSE(recovery::confirmationReached(true, 0.2, 0.2, 2, 3));
  EXPECT_TRUE(recovery::confirmationReached(true, 0.2, 0.2, 3, 3));
}

TEST(WrongWayRecovery, ConfirmedAlignmentUsesReentryHysteresis)
{
  auto input = validEvidence(82.0);
  EXPECT_TRUE(
    recovery::alignmentReleaseWindow(
      true, true, input, 90.0 * kPi / 180.0));
  input.heading_error_rad = 90.0 * kPi / 180.0;
  EXPECT_FALSE(
    recovery::alignmentReleaseWindow(
      true, true, input, 90.0 * kPi / 180.0));
  input = validEvidence(82.0);
  input.heading_sample_fresh = false;
  EXPECT_FALSE(
    recovery::alignmentReleaseWindow(
      true, true, input, 90.0 * kPi / 180.0));
  EXPECT_FALSE(
    recovery::alignmentReleaseWindow(
      false, true, validEvidence(82.0), 90.0 * kPi / 180.0));
}

TEST(WrongWayRecovery, ReverseSteeringReducesTheSignedHeadingError)
{
  EXPECT_EQ(recovery::chooseTurnSign(2.0, 1.0, 1.0, 3.0), 1);
  EXPECT_EQ(recovery::chooseTurnSign(-2.0, 1.0, 1.0, 3.0), -1);
  EXPECT_DOUBLE_EQ(recovery::reverseSteeringAngle(1, 0.4), 0.4);
  EXPECT_DOUBLE_EQ(recovery::reverseSteeringAngle(-1, 0.4), -0.4);

  // At the +/-pi ambiguity, choose the rear side with more observed room and
  // retain that sign for the recovery episode.
  EXPECT_EQ(recovery::chooseTurnSign(kPi, 2.0, 1.0, 3.0), 1);
  EXPECT_EQ(recovery::chooseTurnSign(-kPi, 1.0, 2.0, 3.0), -1);
  EXPECT_EQ(
    recovery::chooseTurnSign(kPi, std::numeric_limits<double>::infinity(), 1.0, 3.0),
    1);
  EXPECT_EQ(
    recovery::chooseTurnSign(-kPi, 1.0, std::numeric_limits<double>::infinity(), 3.0),
    -1);
  EXPECT_EQ(
    recovery::chooseTurnSign(
      -kPi, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(), 3.0),
    1);

  EXPECT_EQ(
    recovery::retainEpisodeTurnSign(1, -2.0, 0.5, 2.0, 3.0), 1);
  EXPECT_EQ(
    recovery::retainEpisodeTurnSign(-1, 2.0, 2.0, 0.5, 3.0), -1);
}

TEST(WrongWayRecovery, RequiresClearanceOnlyOnTheSelectedReverseSide)
{
  EXPECT_TRUE(
    recovery::selectedReverseSideClear(
      1, 0.8, 0.0, 0.25, 0.7,
      0.1, 0.5));
  EXPECT_TRUE(
    recovery::selectedReverseSideClear(
      -1, 0.0, 0.8, 0.25, 0.1,
      std::numeric_limits<double>::infinity(), 0.5));
  EXPECT_FALSE(
    recovery::selectedReverseSideClear(
      1, 0.2, 0.8, 0.25, 0.7,
      0.7, 0.5));
  EXPECT_FALSE(
    recovery::selectedReverseSideClear(
      -1, 0.8, 0.8, 0.25, 0.7,
      0.4, 0.5));
  EXPECT_FALSE(
    recovery::selectedReverseSideClear(
      0, 0.8, 0.8, 0.25, 0.7,
      0.7, 0.5));
}

TEST(WrongWayRecovery, LatchClearsOnlyAfterAlignmentAndForwardProgress)
{
  bool latched = recovery::updatePersistentLatch(false, true, false, false);
  EXPECT_TRUE(latched);
  latched = recovery::updatePersistentLatch(latched, false, true, false);
  EXPECT_TRUE(latched);
  latched = recovery::updatePersistentLatch(latched, false, false, true);
  EXPECT_TRUE(latched);
  latched = recovery::updatePersistentLatch(latched, false, true, true);
  EXPECT_FALSE(latched);
}
