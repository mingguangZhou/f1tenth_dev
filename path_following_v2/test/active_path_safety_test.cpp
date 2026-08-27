#include <limits>

#include "gtest/gtest.h"

#include "path_following_v2/active_path_safety.hpp"

namespace safety = path_following_v2::active_path_safety;

TEST(ActivePathSafety, ReserveMarginEncroachmentDoesNotFailPrimaryPath)
{
  EXPECT_EQ(
    safety::replacementFailureDisposition(false, false),
    safety::ReplacementFailureDisposition::HOLD_REDUCED_SPEED);
}

TEST(ActivePathSafety, PhysicalBlockageCanConfirmPrimaryFailure)
{
  EXPECT_EQ(
    safety::replacementFailureDisposition(true, false),
    safety::ReplacementFailureDisposition::CONFIRM_PRIMARY_FAILURE);
}

TEST(ActivePathSafety, ExhaustedPathWithoutRejoinCanConfirmPrimaryFailure)
{
  EXPECT_EQ(
    safety::replacementFailureDisposition(false, true),
    safety::ReplacementFailureDisposition::CONFIRM_PRIMARY_FAILURE);
}

TEST(ActivePathSafety, ClearCandidateReplacesActivePath)
{
  EXPECT_EQ(
    safety::activeCandidateDisposition(
      safety::CandidateValidationOutcome::CLEAR, false, false),
    safety::ActiveCandidateDisposition::ACTIVATE_REPLACEMENT);
}

TEST(ActivePathSafety, RejectedCandidateRetainsPhysicallyClearActivePath)
{
  EXPECT_EQ(
    safety::activeCandidateDisposition(
      safety::CandidateValidationOutcome::COLLISION, true, false),
    safety::ActiveCandidateDisposition::RETAIN_ACTIVE_PATH);
}

TEST(ActivePathSafety, RejectedCandidateRoutesBlockedActivePathThroughFailureHandling)
{
  EXPECT_EQ(
    safety::activeCandidateDisposition(
      safety::CandidateValidationOutcome::COLLISION, false, false),
    safety::ActiveCandidateDisposition::CONTINUE_FAILURE_HANDLING);
}

TEST(ActivePathSafety, RejectedCandidateRoutesExhaustedActivePathThroughFailureHandling)
{
  EXPECT_EQ(
    safety::activeCandidateDisposition(
      safety::CandidateValidationOutcome::COLLISION, true, true),
    safety::ActiveCandidateDisposition::CONTINUE_FAILURE_HANDLING);
}

TEST(ActivePathSafety, ValidationInputFailureRemainsFailClosed)
{
  EXPECT_EQ(
    safety::activeCandidateDisposition(
      safety::CandidateValidationOutcome::INPUT_FAILURE, true, false),
    safety::ActiveCandidateDisposition::FAIL_PRIMARY);
}

TEST(ActivePathSafety, MidPlanningValidationScansDriveConfirmationWithoutEqualityGate)
{
  constexpr std::int64_t control_scan_a = 100;
  constexpr std::int64_t validation_scan_b = 200;
  constexpr std::int64_t next_control_scan_c = 300;
  constexpr std::int64_t next_validation_scan_d = 400;
  constexpr std::int64_t third_validation_scan_e = 500;

  safety::ScanConfirmationState state;
  auto update = safety::updateDistinctScanConfirmation(
    state, validation_scan_b, true, 3);
  EXPECT_TRUE(update.consumed);
  EXPECT_EQ(update.state.consecutive_scans, 1);
  EXPECT_EQ(update.state.last_evidence_scan_id, validation_scan_b);
  EXPECT_FALSE(update.confirmed);
  state = update.state;

  update = safety::updateDistinctScanConfirmation(state, validation_scan_b, true, 3);
  EXPECT_FALSE(update.consumed);
  EXPECT_EQ(update.state.consecutive_scans, 1);
  update = safety::updateDistinctScanConfirmation(state, control_scan_a, true, 3);
  EXPECT_FALSE(update.consumed);
  EXPECT_EQ(update.state.consecutive_scans, 1);

  update = safety::updateDistinctScanConfirmation(
    state, next_validation_scan_d, true, 3);
  EXPECT_TRUE(update.consumed);
  EXPECT_EQ(update.state.consecutive_scans, 2);
  EXPECT_EQ(update.state.last_evidence_scan_id, next_validation_scan_d);
  state = update.state;
  update = safety::updateDistinctScanConfirmation(state, next_control_scan_c, true, 3);
  EXPECT_FALSE(update.consumed);
  EXPECT_EQ(update.state.consecutive_scans, 2);

  update = safety::updateDistinctScanConfirmation(
    state, third_validation_scan_e, true, 3);
  EXPECT_TRUE(update.confirmed);
}

TEST(ActivePathSafety, NewClearEvidenceBreaksFailureConfirmationSequence)
{
  safety::ScanConfirmationState state{2, 400};
  const auto update = safety::updateDistinctScanConfirmation(state, 500, false, 3);
  EXPECT_TRUE(update.consumed);
  EXPECT_EQ(update.state.consecutive_scans, 0);
  EXPECT_EQ(update.state.last_evidence_scan_id, 500);
  EXPECT_FALSE(update.confirmed);
}

TEST(ActivePathSafety, TerminalSafeYieldRequiresDistinctFreshScans)
{
  auto update = safety::updateSafeYieldConfirmation(0, true, 0.0, 0.05, 3);
  EXPECT_EQ(update.consecutive_terminal_scans, 1);
  EXPECT_FALSE(update.primary_failure_confirmed);

  update = safety::updateSafeYieldConfirmation(
    update.consecutive_terminal_scans, false, 0.0, 0.05, 3);
  EXPECT_EQ(update.consecutive_terminal_scans, 1);
  EXPECT_FALSE(update.primary_failure_confirmed);

  update = safety::updateSafeYieldConfirmation(
    update.consecutive_terminal_scans, true, 0.0, 0.05, 3);
  update = safety::updateSafeYieldConfirmation(
    update.consecutive_terminal_scans, true, 0.0, 0.05, 3);
  EXPECT_EQ(update.consecutive_terminal_scans, 3);
  EXPECT_TRUE(update.primary_failure_confirmed);
}

TEST(ActivePathSafety, PositiveSafeYieldClearsTerminalConfirmation)
{
  const auto update = safety::updateSafeYieldConfirmation(4, true, 0.20, 0.05, 5);
  EXPECT_EQ(update.consecutive_terminal_scans, 0);
  EXPECT_FALSE(update.primary_failure_confirmed);
}

TEST(ActivePathSafety, NonFiniteSafeYieldFailsClosed)
{
  const auto update = safety::updateSafeYieldConfirmation(
    1, true, std::numeric_limits<double>::quiet_NaN(), 0.05, 2);
  EXPECT_EQ(update.consecutive_terminal_scans, 2);
  EXPECT_TRUE(update.primary_failure_confirmed);
}
