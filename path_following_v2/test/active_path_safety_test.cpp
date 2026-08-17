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
