#include <cmath>

#include "gtest/gtest.h"

#include "path_following_v2/maneuver_speed_policy.hpp"

namespace speed = path_following_v2::maneuver_speed;

TEST(ManeuverSpeedPolicy, GentleTrajectoryUsesConfiguredCeiling)
{
  EXPECT_DOUBLE_EQ(speed::curvatureLimitedSpeed(4.5, 5.0, 3.5, 0.0), 4.5);
}

TEST(ManeuverSpeedPolicy, CurvatureRespectsLateralAccelerationLimit)
{
  const double result = speed::curvatureLimitedSpeed(4.5, 5.0, 3.5, 0.875);
  EXPECT_NEAR(result, 2.0, 1e-9);
  EXPECT_LE(result * result * 0.875, 3.5 + 1e-9);
}

TEST(ManeuverSpeedPolicy, CommandEnvelopeStillBoundsManeuverCeiling)
{
  EXPECT_DOUBLE_EQ(speed::curvatureLimitedSpeed(6.0, 5.0, 3.5, 0.0), 5.0);
}

TEST(ManeuverSpeedPolicy, InvalidCurvatureCannotIncreaseSpeed)
{
  EXPECT_DOUBLE_EQ(
    speed::curvatureLimitedSpeed(4.5, 5.0, 3.5, NAN), 4.5);
}
