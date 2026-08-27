#include "gtest/gtest.h"

#include "drive_arbitration_v2/lower_recovery_coordination.hpp"

namespace coordination = drive_arbitration_v2::lower_recovery_coordination;

TEST(LowerRecoveryCoordination, LatchedHandoffBlocksNominalReactiveAutoReturn)
{
  EXPECT_FALSE(coordination::holdReason(true, true, "2", "NOMINAL", true).empty());
  EXPECT_TRUE(coordination::holdReason(true, true, "2", "NOMINAL", false).empty());
}

TEST(LowerRecoveryCoordination, ExistingFreshModeAndStateContractStillFailsClosed)
{
  EXPECT_FALSE(coordination::holdReason(true, false, "2", "NOMINAL", false).empty());
  EXPECT_FALSE(coordination::holdReason(true, true, "1", "NOMINAL", false).empty());
  EXPECT_FALSE(coordination::holdReason(true, true, "2", "REVERSE_RECOVERY", false).empty());
  EXPECT_TRUE(coordination::holdReason(false, false, "1", "EMERGENCY_STOP", true).empty());
}
