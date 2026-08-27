#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace reactive_control_v2::reverse_swept_safety
{

struct Pose2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Actor
{
  Pose2 pose;
  double signed_speed_mps{0.0};
  double yaw_rate_radps{0.0};
};

struct GridView
{
  bool valid{false};
  std::string frame;
  std::size_t width{0};
  std::size_t height{0};
  double resolution{0.0};
  Pose2 origin;
  const std::vector<int8_t> * cells{nullptr};
  int occupied_threshold{50};
};

struct Input
{
  GridView map;
  Pose2 ego;
  std::vector<Actor> actors;
  double steering_rad{0.0};
  double wheelbase_m{0.33};
  double reverse_speed_mps{0.25};
  double arc_distance_m{0.50};
  double sample_step_m{0.02};
  double vehicle_length_m{0.58};
  double vehicle_width_m{0.31};
  double clearance_margin_m{0.03};
};

struct Result
{
  bool valid{false};
  std::string reason;
  double checked_distance_m{0.0};
};

inline bool finitePose(const Pose2 & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

inline Pose2 advanceByDistance(
  const Pose2 & start, const double signed_distance_m, const double curvature)
{
  Pose2 output = start;
  if (std::abs(curvature) <= 1e-9) {
    output.x += signed_distance_m * std::cos(start.yaw);
    output.y += signed_distance_m * std::sin(start.yaw);
    return output;
  }
  output.yaw = start.yaw + curvature * signed_distance_m;
  output.x += (std::sin(output.yaw) - std::sin(start.yaw)) / curvature;
  output.y += (-std::cos(output.yaw) + std::cos(start.yaw)) / curvature;
  return output;
}

inline Pose2 predictActor(const Actor & actor, const double time_sec)
{
  if (std::abs(actor.yaw_rate_radps) <= 1e-9) {
    return advanceByDistance(actor.pose, actor.signed_speed_mps * time_sec, 0.0);
  }
  const double curvature = actor.yaw_rate_radps / std::max(
    1e-9, std::abs(actor.signed_speed_mps));
  if (std::abs(actor.signed_speed_mps) <= 1e-9) {
    Pose2 output = actor.pose;
    output.yaw += actor.yaw_rate_radps * time_sec;
    return output;
  }
  return advanceByDistance(
    actor.pose, actor.signed_speed_mps * time_sec,
    actor.signed_speed_mps >= 0.0 ? curvature : -curvature);
}

inline double dot(
  const double ax, const double ay, const double bx, const double by)
{
  return ax * bx + ay * by;
}

inline bool orientedBoxesOverlap(
  const Pose2 & first, const double first_half_length,
  const double first_half_width, const Pose2 & second,
  const double second_half_length, const double second_half_width)
{
  const double first_forward_x = std::cos(first.yaw);
  const double first_forward_y = std::sin(first.yaw);
  const double first_left_x = -first_forward_y;
  const double first_left_y = first_forward_x;
  const double second_forward_x = std::cos(second.yaw);
  const double second_forward_y = std::sin(second.yaw);
  const double second_left_x = -second_forward_y;
  const double second_left_y = second_forward_x;
  const double delta_x = second.x - first.x;
  const double delta_y = second.y - first.y;

  const double axes[4][2] = {
    {first_forward_x, first_forward_y},
    {first_left_x, first_left_y},
    {second_forward_x, second_forward_y},
    {second_left_x, second_left_y}};
  for (const auto & axis : axes) {
    const double first_radius = first_half_length * std::abs(
      dot(
        axis[0], axis[1], first_forward_x, first_forward_y)) +
      first_half_width * std::abs(
      dot(
        axis[0], axis[1], first_left_x, first_left_y));
    const double second_radius = second_half_length * std::abs(
      dot(
        axis[0], axis[1], second_forward_x, second_forward_y)) +
      second_half_width * std::abs(
      dot(
        axis[0], axis[1], second_left_x, second_left_y));
    if (std::abs(dot(axis[0], axis[1], delta_x, delta_y)) >
      first_radius + second_radius)
    {
      return false;
    }
  }
  return true;
}

inline bool mapFootprintClear(
  const GridView & map, const Pose2 & pose,
  const double half_length, const double half_width)
{
  if (!map.valid || map.cells == nullptr || map.frame.empty() ||
    map.width == 0 || map.height == 0 || !std::isfinite(map.resolution) ||
    map.resolution <= 0.0 || map.cells->size() != map.width * map.height ||
    !finitePose(map.origin) || !finitePose(pose))
  {
    return false;
  }

  const double forward_x = std::cos(pose.yaw);
  const double forward_y = std::sin(pose.yaw);
  const double left_x = -forward_y;
  const double left_y = forward_x;
  const double origin_cos = std::cos(map.origin.yaw);
  const double origin_sin = std::sin(map.origin.yaw);
  double min_grid_x = std::numeric_limits<double>::infinity();
  double max_grid_x = -std::numeric_limits<double>::infinity();
  double min_grid_y = std::numeric_limits<double>::infinity();
  double max_grid_y = -std::numeric_limits<double>::infinity();
  for (const double longitudinal : {-half_length, half_length}) {
    for (const double lateral : {-half_width, half_width}) {
      const double world_x = pose.x + longitudinal * forward_x + lateral * left_x;
      const double world_y = pose.y + longitudinal * forward_y + lateral * left_y;
      const double translated_x = world_x - map.origin.x;
      const double translated_y = world_y - map.origin.y;
      const double grid_x =
        (origin_cos * translated_x + origin_sin * translated_y) / map.resolution;
      const double grid_y =
        (-origin_sin * translated_x + origin_cos * translated_y) / map.resolution;
      min_grid_x = std::min(min_grid_x, grid_x);
      max_grid_x = std::max(max_grid_x, grid_x);
      min_grid_y = std::min(min_grid_y, grid_y);
      max_grid_y = std::max(max_grid_y, grid_y);
    }
  }
  if (min_grid_x < 0.0 || min_grid_y < 0.0 ||
    max_grid_x >= static_cast<double>(map.width) ||
    max_grid_y >= static_cast<double>(map.height))
  {
    return false;
  }

  const int min_x = std::max(0, static_cast<int>(std::floor(min_grid_x)) - 1);
  const int max_x = std::min(
    static_cast<int>(map.width) - 1,
    static_cast<int>(std::floor(max_grid_x)) + 1);
  const int min_y = std::max(0, static_cast<int>(std::floor(min_grid_y)) - 1);
  const int max_y = std::min(
    static_cast<int>(map.height) - 1,
    static_cast<int>(std::floor(max_grid_y)) + 1);
  const double cell_half = 0.5 * map.resolution;
  for (int grid_y = min_y; grid_y <= max_y; ++grid_y) {
    for (int grid_x = min_x; grid_x <= max_x; ++grid_x) {
      const int occupancy = static_cast<int>(
        (*map.cells)[static_cast<std::size_t>(grid_y) * map.width +
        static_cast<std::size_t>(grid_x)]);
      if (occupancy >= 0 && occupancy < map.occupied_threshold) {
        continue;
      }
      const double local_x = (static_cast<double>(grid_x) + 0.5) * map.resolution;
      const double local_y = (static_cast<double>(grid_y) + 0.5) * map.resolution;
      Pose2 cell;
      cell.x = map.origin.x + origin_cos * local_x - origin_sin * local_y;
      cell.y = map.origin.y + origin_sin * local_x + origin_cos * local_y;
      cell.yaw = map.origin.yaw;
      if (orientedBoxesOverlap(
          pose, half_length, half_width, cell, cell_half, cell_half))
      {
        return false;
      }
    }
  }
  return true;
}

inline Result evaluateArc(const Input & input)
{
  Result output;
  if (!finitePose(input.ego) || !std::isfinite(input.steering_rad) ||
    !std::isfinite(input.wheelbase_m) || input.wheelbase_m <= 0.0 ||
    !std::isfinite(input.reverse_speed_mps) || input.reverse_speed_mps <= 0.0 ||
    !std::isfinite(input.arc_distance_m) || input.arc_distance_m <= 0.0 ||
    !std::isfinite(input.sample_step_m) || input.sample_step_m <= 0.0 ||
    !std::isfinite(input.vehicle_length_m) || input.vehicle_length_m <= 0.0 ||
    !std::isfinite(input.vehicle_width_m) || input.vehicle_width_m <= 0.0 ||
    !std::isfinite(input.clearance_margin_m) || input.clearance_margin_m < 0.0)
  {
    output.reason = "invalid reverse swept-path input";
    return output;
  }
  for (const auto & actor : input.actors) {
    if (!finitePose(actor.pose) || !std::isfinite(actor.signed_speed_mps) ||
      !std::isfinite(actor.yaw_rate_radps))
    {
      output.reason = "invalid simulator actor state";
      return output;
    }
  }

  const double ego_half_length = 0.5 * input.vehicle_length_m + input.clearance_margin_m;
  const double ego_half_width = 0.5 * input.vehicle_width_m + input.clearance_margin_m;
  const double actor_half_length = 0.5 * input.vehicle_length_m + input.clearance_margin_m;
  const double actor_half_width = 0.5 * input.vehicle_width_m + input.clearance_margin_m;
  const double curvature = std::tan(input.steering_rad) / input.wheelbase_m;
  const int samples = std::max(
    1, static_cast<int>(std::ceil(input.arc_distance_m / input.sample_step_m)));
  for (int sample = 0; sample <= samples; ++sample) {
    const double distance_m = input.arc_distance_m *
      static_cast<double>(sample) / static_cast<double>(samples);
    const Pose2 ego_pose = advanceByDistance(input.ego, -distance_m, curvature);
    if (!mapFootprintClear(input.map, ego_pose, ego_half_length, ego_half_width)) {
      output.reason = "reverse swept footprint intersects map occupancy";
      output.checked_distance_m = distance_m;
      return output;
    }
    const double time_sec = distance_m / input.reverse_speed_mps;
    for (const auto & actor : input.actors) {
      const Pose2 actor_pose = predictActor(actor, time_sec);
      if (orientedBoxesOverlap(
          ego_pose, ego_half_length, ego_half_width,
          actor_pose, actor_half_length, actor_half_width))
      {
        output.reason = "reverse swept footprint intersects predicted traffic";
        output.checked_distance_m = distance_m;
        return output;
      }
    }
  }
  output.valid = true;
  output.reason = "reverse swept map and traffic corridor clear";
  output.checked_distance_m = input.arc_distance_m;
  return output;
}

inline Result evaluateSteeringTransition(
  const Input & input, const double initial_steering_rad,
  const int steering_samples = 5)
{
  Result output;
  if (!std::isfinite(initial_steering_rad) || steering_samples < 1) {
    output.reason = "invalid reverse steering-transition input";
    return output;
  }
  for (int sample = 0; sample <= steering_samples; ++sample) {
    Input sampled_input = input;
    const double blend = static_cast<double>(sample) /
      static_cast<double>(steering_samples);
    sampled_input.steering_rad = initial_steering_rad +
      blend * (input.steering_rad - initial_steering_rad);
    const Result sample_result = evaluateArc(sampled_input);
    if (!sample_result.valid) {
      output.reason = "reverse steering transition blocked: " + sample_result.reason;
      output.checked_distance_m = sample_result.checked_distance_m;
      return output;
    }
  }
  output.valid = true;
  output.reason = "reverse steering transition map and traffic corridor clear";
  output.checked_distance_m = input.arc_distance_m;
  return output;
}

}  // namespace reactive_control_v2::reverse_swept_safety
