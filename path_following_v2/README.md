# path_following_v2

ROS 2 Foxy localization-based raceline following with a persistent local
trajectory planner.

## Data flow

```text
/raceline_waypoints
        -> path_generator
        -> /path_following_v2/raceline_local_path  (raw, maximum 10 m)
                         + /scan + map->base TF
                         -> local_trajectory_planner
                         -> /path_following_v2/local_path          (final)
                         -> /path_following_v2/path_status
                         -> /path_following_v2/trajectory_speed_cap_mps
                                      |
/path_following_v2/rule_speed_index --+
/rl_speed_inference/speed_residual_mps optional
                                      -> path_following_v2
                                      -> /path_following_v2/nominal_cmd
```

The final local path is the raw raceline or a persistent map-frame trajectory
from the car's current pose toward a safe rejoin point. While that local plan
is active, its remaining modified section is joined to the newest raw raceline
so the controller continues to receive a full 10 m forward trajectory. Existing
follower, guard, and arbitrator topic interfaces remain unchanged.

## Common vehicle envelope

The measured car width is `0.28 m`. The planner, final guard, and Reactive upper
use the same per-side envelope:

```text
safety_half_width = vehicle_width / 2 + lateral_safety_margin
                  = 0.28 / 2 + 0.10
                  = 0.24 m per side
```

The two independently adjustable inputs are:

```yaml
vehicle_width_m: 0.28
lateral_safety_margin_m: 0.10
```

## Metric horizon

`path_generator` accumulates physical path length rather than assuming a CSV
point spacing:

```yaml
local_path_target_length_m: 10.0
local_path_max_points: 600
```

At roughly `0.03 m` spacing, 10 m is about 334 points. The planner also caps
usable scan returns and Frenet association at 10 m:

```yaml
scan_range_cap_m: 10.0
planning_distance_m: 10.0
```

The requested detection reach is automatically reduced only when the current
raw path would otherwise be too short to contain the detected obstacle model,
buffers, and `detour_return_min_length_m`. With the defaults and a complete
10 m raw path, the effective trigger is about 6.4 m. Scan returns farther ahead
are still available for candidate validation, but they do not cause a
predictable premature `NO_SAFE_PATH` before a complete maneuver can fit. The old
point-count behavior remains as the internal `legacy_local_path_points`
fallback when the metric target is set to zero or less. It is intentionally not
shown in the shipped YAML because the positive metric target is authoritative.

The generator stops at the physical target, one unique lap, or the point
ceiling—whichever is reached first. The simulator uses a 10 m target; the
onboard file currently keeps the tested short-track target of 4 m.

Parameter-interface rename map:

| Previous name | Current name |
| --- | --- |
| `local_path_horizon_m` | `local_path_target_length_m` |
| `max_local_path_points` | `local_path_max_points` |
| `speed_min/max` | `command_speed_min/max_mps` |
| `rule_min_speed_mps` | `rule_curve_min_speed_mps` |
| `rule_max_speed_mps` | `rule_straight_speed_mps` |
| `rule_speed_curvature_lookahead_points` | `rule_speed_curvature_preview_m` |
| `normal_speed_cap_mps` | removed; uses shared `command_speed_max_mps` |
| `detour_speed_cap_mps` | `avoidance_speed_cap_mps` |
| `speed_mode` | `speed_policy_mode` |
| `wheelbase` | `wheelbase_m` |
| `min_forward_point_x` | `min_forward_point_x_m` |

## Planner activation

The node is named `local_trajectory_planner`, because it has two related jobs:

1. Avoid a LiDAR cluster that intersects the raw raceline's widened band.
2. Recover smoothly when the localized car is laterally away from the global
   raceline.

Raceline-relative coordinates are:

- `s`: distance forward along the current raw raceline window;
- `d`: signed lateral offset, positive to the left.

The recovery hysteresis is:

```yaml
recovery_enter_lateral_error_m: 0.18
recovery_exit_lateral_error_m: 0.08
recovery_exit_heading_error_deg: 10.0
```

Thus an offset above 0.18 m requests a local plan, but an active plan is not
declared complete until the car is back within 0.08 m and aligned with the
raceline.

## Trajectory construction

Each plan starts from the actual localized car pose. It does not assume that
the car is already on the raceline.

For obstacle avoidance, the planner connects:

```text
(current pose/current d) -> (safe passing d) -> (future raceline d=0)
```

For lateral recovery without an obstacle, it connects:

```text
(current pose/current d) -> (future raceline d=0)
```

The departure and lateral-recovery curves use a quintic Hermite polynomial. It
starts at the localized car's actual lateral offset and heading slope, then
arrives at the passing offset or raceline with zero lateral slope and
acceleration. The return uses the zero-slope quintic smoothstep:

```text
q(u) = 10u^3 - 15u^4 + 6u^5,  0 <= u <= 1
```

The recovery length grows with lateral error and is clamped:

```yaml
recovery_min_rejoin_length_m: 2.5
recovery_max_rejoin_length_m: 5.0
recovery_length_gain: 3.0
```

For example, a `0.30 m` lateral error requests approximately
`2.5 + 3.0 * 0.30 = 3.4 m` to converge.

For a centered `0.50 m` obstacle, the nominal passing offsets are about
`+0.49 m` and `-0.49 m` because the obstacle half-width is `0.25 m` and the
vehicle half-envelope is `0.24 m`. That is now only the first sampled offset,
not the automatically preferred trajectory.

## Candidate safety and side choice

Candidate validation uses the same straightforward widened-band concept as the
final trajectory guard. A candidate is rejected when at least
`blocked_min_points` connected LiDAR returns enter its `0.24 m` half-band. One
isolated beam does not reject the complete plan.

For each side, the planner starts at the minimum offset that clears the obstacle
and tests a bounded set of farther offsets:

```yaml
candidate_lateral_step_m: 0.08
max_candidates_per_side: 6
max_lateral_shift_m: 0.80
```

It selects the valid offset with the largest bottleneck clearance over the
actual passing plateau. This approximates the centre of the available passage,
instead of preferring the path closest to the raceline. Candidates must also:

- fit the 10 m window including complete rejoin;
- remain within `max_lateral_shift_m`;
- keep only the locally modified section below the
  wheelbase/steering-derived curvature limit (ordinary raceline curvature
  after the rejoin is not part of candidate rejection);
- retain a valid scan, TF, and raw path.

The return length is adaptive and uses the longest configured value that fits:

```yaml
detour_return_min_length_m: 2.5
detour_return_max_length_m: 5.0
```

Thus the return is no longer fixed at 1.8 m. Known raceline geometry may extend
beyond current scan visibility, while the held trajectory is rechecked as new
scan space becomes visible.

When both sides are valid, clearance is scored over the maneuver-specific
passing section, not the shared beginning. The active side is retained unless
the alternative is wider by at least:

```yaml
side_switch_clearance_advantage_m: 0.10
```

### Early refinement and side commitment

The first safe trajectory is published immediately. For at most `0.75 s`, the
planner reevaluates the bounded left/right candidates every two fresh scans.
The alternative side must remain at least `0.10 m` safer for two refinement
evaluations before it can replace the initial plan.

```yaml
precommit_max_sec: 0.75
precommit_refine_interval_scans: 2
side_switch_confirmation_evaluations: 2
side_commit_obstacle_distance_m: 3.0
side_commit_lateral_progress_m: 0.08
```

The current side is committed as soon as any one condition is met: the 0.75 s
window ends, the obstacle is within 3.0 m along the raceline, or the car has
moved 0.08 m laterally toward the selected side. After commitment, the stored
side does not change unless the active trajectory is confirmed blocked and a
material replan is required.

## Exactly how a plan is held

An accepted candidate is copied into `active_plan_`, assigned a monotonically
increasing `plan_id`, and anchored in the map frame. The geometry is not rebuilt
from every scan. At each control cycle the planner only:

1. Finds the car's monotonically advancing index on the stored path.
2. Removes the already-passed prefix of the locally modified section.
3. Joins its zero-offset rejoin point to the newest raw raceline ahead.
4. Caps the combined command trajectory at the shared 10 m metric horizon.
5. Republishes it with a fresh timestamp and rechecks the remaining widened
   band against the latest scan.

The stored local curve and `plan_id` still do not change during this extension.
Only the ordinary raceline tail is refreshed. Near completion, when the car has
passed the rejoin point but is still inside the four-scan convergence
confirmation, the published path is the current full raw raceline rather than
an almost-empty stored tail. This prevents a short-path handoff to Reactive.

Normal release requires all of these:

```text
plan age >= minimum_plan_hold_sec
progress index >= stored rejoin index
abs(raceline lateral error) <= recovery_exit_lateral_error_m
abs(raceline heading error) <= recovery_exit_heading_error_deg
the above remains true for rejoin_confirmation_scans fresh scans
```

Defaults are:

```yaml
minimum_plan_hold_sec: 0.50
rejoin_confirmation_scans: 4
```

At a 20 Hz scan rate, the final convergence confirmation is about `0.20 s`.
There is deliberately no fixed normal detour duration: a 5 m plan is held until
the car executes it and reaches its rejoin conditions.

`maximum_plan_hold_sec: 12.0` is only a stale-plan backstop. Reaching it requests
a new plan from the current pose; it does not silently release the car to the
raw raceline.

## When an active plan is updated

Outside the short pre-commit refinement above, the stable plan is replaced and
receives a new `plan_id` only when at least one material condition occurs:

- its remaining widened band becomes blocked;
- the car is more than `plan_deviation_replan_m` from it;
- it reaches the stored-path end without satisfying rejoin conditions;
- it reaches `maximum_plan_hold_sec`;
- a valid replacement is then found from the current pose and scan.

The same passing side is preferred unless the other side has the configured
clearance advantage. If a valid replacement is not yet available, the planner
publishes `REPLAN_PENDING`, keeps a low cap, and confirms the failure over fresh
scan messages:

```yaml
plan_deviation_replan_m: 0.35
active_path_blocked_confirmation_scans: 3
no_safe_path_confirmation_scans: 3
replan_pending_speed_cap_mps: 0.4   # onboard; 0.5 simulator
```

Only `NO_SAFE_PATH_CONFIRMED` or an immediate `CRITICAL_OBSTACLE` requests
Reactive ownership. A single ordinary planning failure no longer latches
Reactive.

## States

| State | `trajectory_mode` | Meaning |
|---|---|---|
| `READY` | `RACELINE` | Raw path clear and car inside recovery threshold |
| `READY` | `AVOIDING` | Executing stored path toward/past the passing offset |
| `READY` | `REJOINING` | Executing the stored return toward the raceline |
| `READY` | `RECOVERING_TO_RACELINE` | Converging from lateral displacement |
| `READY` | `REPLAN_PENDING` | Slowed while a material update/failure is confirmed |
| `NO_SAFE_PATH_CONFIRMED` | `NONE` | Repeated failure; request Reactive |
| `CRITICAL_OBSTACLE` | `NONE` | Immediate safety transfer |
| input/TF/scan failures | `NONE` | Fail closed to Reactive or STOP |

Diagnostics include `plan_id`, `plan_age_sec`, `plan_progress_index`,
`plan_rejoin_index`, confirmation counters, side, obstacle geometry, clearance,
curvature, `side_committed`, effective detection distance, and speed cap.

## Bounded computation

Pure raceline following does not run candidate search. Obstacle planning checks
no more than `2 x max_candidates_per_side` trajectories. That bounded search
runs only for initial planning, material replanning, or the short pre-commit
window; the normal held-plan loop only advances progress, appends the current
raceline tail, and validates the stored path. The extension is a linear pass
over the current path points and does not run candidate generation or curvature
search again. The existing 600-point failsafe remains sufficient for the 10 m
path at the current approximately 0.03 m spacing.

## Speed and visualization

The physical command envelope is configured once at the top of each YAML:

```yaml
/**:
  ros__parameters:
    command_speed_min_mps: 1.0
    command_speed_max_mps: 10.0
```

The generator and follower use the same range to encode/decode the normalized
rule-speed index. The local planner uses `command_speed_max_mps` as the
ordinary-raceline cap, so there is no separate normal-speed-cap value to keep
synchronized.

Normal raceline demand is tuned only in `path_generator`:

```yaml
rule_curve_min_speed_mps: 1.0
rule_straight_speed_mps: 2.5
rule_speed_curvature_gain: 2.0
rule_speed_curvature_preview_m: 0.09
```

The curvature preview is physical distance, not waypoint count. `0.09 m`
preserves the former three-waypoint preview at the current approximately
`0.03 m` spacing. Changing it is functional speed tuning and should be tested
separately from this parameter cleanup.

The planner exposes only temporary maneuver caps:

```yaml
avoidance_speed_cap_mps: 1.5
recovery_speed_cap_mps: 1.5
replan_pending_speed_cap_mps: 0.5
```

`speed_policy_mode` selects rule-only (`0`) or rule plus a fresh RL speed
residual (`1`). The established future RL structure is unchanged: bounded
learning modifies the rule baseline, while the physical envelope, command-rate
limit, maneuver cap, arbitrator, and lower safety controller remain outside the
policy.

The follower applies the planner cap after rule/RL speed calculation:

```text
final_speed = min(rate_limited_rule_plus_RL_speed, fresh planner cap)
```

The final authorized trajectory marker uses:

- dark blue: ordinary raceline;
- light blue (`0.20, 0.80, 1.00`): a local replan currently selected by the
  arbitrator and passed through the lower controller;
- orange: Reactive upper trajectory.

Candidate debug markers use light blue for the selected local candidate, grey
for the alternative, and red for obstacle-cluster returns.

## Nodes and topics

### `path_generator_node`

Publishes the raw metric raceline window, rule speed index, and upstream status.

### `local_trajectory_planner_node`

Inputs:

- `/path_following_v2/raceline_local_path`
- `/scan`
- TF from the raw-path frame to `robot_frame`

Outputs:

- `/path_following_v2/local_path`
- `/path_following_v2/path_status`
- `/path_following_v2/trajectory_speed_cap_mps`
- `/path_following_v2/detour_markers` (topic retained for RViz compatibility)

### `path_following_v2_node`

Consumes the final path and cap and publishes
`/path_following_v2/nominal_cmd`. It remains the deterministic pure-pursuit
follower; only the lower safety controller publishes final `/drive`.

## Build and run

```bash
cd /sim_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select path_following_v2 drive_arbitration_v2 reactive_control_v2 oudtra_driver_bringup
source install/setup.bash

ros2 launch oudtra_driver_bringup full_stack_sim_launch.py
```

Useful checks:

```bash
ros2 topic echo /path_following_v2/path_status
ros2 topic echo /path_following_v2/trajectory_speed_cap_mps
ros2 topic echo /path_following_v2/local_path --once
ros2 topic echo /drive_arbitration_v2/selected_mode
```

First confirm `plan_id` stays constant through `AVOIDING` and `REJOINING`, the
stored modified section advances without changing shape, the fresh raceline
tail keeps the published light-blue path at the configured physical horizon,
and Reactive is requested only after confirmed no-safe-path or critical
conditions.
