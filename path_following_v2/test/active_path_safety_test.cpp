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
