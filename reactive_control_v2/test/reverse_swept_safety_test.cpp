#include <cmath>
#include <cstdint>
#include <vector>

#include "gtest/gtest.h"

#include "reactive_control_v2/reverse_swept_safety.hpp"

namespace swept = reactive_control_v2::reverse_swept_safety;

namespace
{
swept::Input clearInput()
{
  static std::vector<int8_t> cells(200U * 200U, 0);
  std::fill(cells.begin(), cells.end(), 0);
  swept::Input input;
  input.map.valid = true;
  input.map.frame = "map";
  input.map.width = 200;
  input.map.height = 200;
  input.map.resolution = 0.05;
  input.map.origin = {-5.0, -5.0, 0.0};
  input.map.cells = &cells;
  input.map.occupied_threshold = 50;
  input.ego = {0.0, 0.0, 0.0};
  input.steering_rad = 0.0;
  return input;
}

void occupyWorld(swept::Input & input, const double x, const double y)
{
  const int grid_x = static_cast<int>(std::floor(
      (x - input.map.origin.x) / input.map.resolution));
  const int grid_y = static_cast<int>(std::floor(
      (y - input.map.origin.y) / input.map.resolution));
  auto * cells = const_cast<std::vector<int8_t> *>(input.map.cells);
  (*cells)[static_cast<std::size_t>(grid_y) * input.map.width +
    static_cast<std::size_t>(grid_x)] = 100;
}
}  // namespace

TEST(ReverseSweptSafety, AcceptsClearMapAndTrafficCorridor)
{
  auto input = clearInput();
  const auto result = swept::evaluateArc(input);
  EXPECT_TRUE(result.valid) << result.reason;
  EXPECT_DOUBLE_EQ(result.checked_distance_m, input.arc_distance_m);
}

TEST(ReverseSweptSafety, RejectsMapObstacleInBlindRearCone)
{
  auto input = clearInput();
  occupyWorld(input, -0.48, 0.0);
  const auto result = swept::evaluateArc(input);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("map occupancy"), std::string::npos);
}

TEST(ReverseSweptSafety, RejectsTrafficInBlindRearCone)
{
  auto input = clearInput();
  swept::Actor actor;
  actor.pose = {-0.75, 0.0, 0.0};
  input.actors.push_back(actor);
  const auto result = swept::evaluateArc(input);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("predicted traffic"), std::string::npos);
}

TEST(ReverseSweptSafety, DistinguishesOppositeSteeringArcs)
{
  auto input = clearInput();
  input.vehicle_length_m = 0.04;
  input.vehicle_width_m = 0.02;
  input.clearance_margin_m = 0.0;
  const double steering = 25.0 * 3.14159265358979323846 / 180.0;
  const double curvature = std::tan(steering) / input.wheelbase_m;
  const auto positive_endpoint = swept::advanceByDistance(
    input.ego, -input.arc_distance_m, curvature);
  swept::Actor actor;
  actor.pose = positive_endpoint;
  input.actors.push_back(actor);
  input.steering_rad = steering;
  EXPECT_FALSE(swept::evaluateArc(input).valid);
  input.steering_rad = -steering;
  EXPECT_TRUE(swept::evaluateArc(input).valid);
}

TEST(ReverseSweptSafety, RejectsObstacleAlongSteeringTransition)
{
  auto input = clearInput();
  input.vehicle_length_m = 0.04;
  input.vehicle_width_m = 0.02;
  input.clearance_margin_m = 0.0;
  const double steering = 25.0 * 3.14159265358979323846 / 180.0;
  input.steering_rad = steering;
  const auto intermediate_endpoint = swept::advanceByDistance(
    input.ego, -input.arc_distance_m,
    std::tan(0.5 * steering) / input.wheelbase_m);
  swept::Actor actor;
  actor.pose = intermediate_endpoint;
  input.actors.push_back(actor);
  EXPECT_TRUE(swept::evaluateArc(input).valid);
  const auto transition = swept::evaluateSteeringTransition(input, 0.0, 4);
  EXPECT_FALSE(transition.valid);
  EXPECT_NE(transition.reason.find("steering transition"), std::string::npos);
}

TEST(ReverseSweptSafety, PredictsMovingTrafficAcrossTheArc)
{
  auto input = clearInput();
  swept::Actor actor;
  actor.pose = {-0.25, -1.0, 1.5707963267948966};
  actor.signed_speed_mps = 0.50;
  input.actors.push_back(actor);
  const auto result = swept::evaluateArc(input);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("predicted traffic"), std::string::npos);
}
