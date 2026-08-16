# path_following_v2

ROS 2 Foxy localization-based raceline following with a persistent local
trajectory planner.

## Data flow

```text
/raceline_waypoints
        -> path_generator
        -> /path_following_v2/raceline_local_path  (raw, metric horizon)
centerline_points_smooth.csv (loaded once at planner startup)
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
planned in centerline-relative coordinates. The centerline supplies a stable
track frame, while the raceline remains the path the planner prefers whenever
clearance permits. While a local plan is active, its remaining modified section
is joined to the newest raw raceline so the controller continues to receive a
full forward trajectory. Existing follower, guard, and arbitrator topic
interfaces remain unchanged.

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

Candidate planning adds a small hard reserve outside that physical envelope.
The simulator uses `0.06 m`, so its planned path stays at least `0.30 m` from
connected obstacle returns. This reserve absorbs scan noise and newly exposed
obstacle surfaces before they reach the physical safety boundary.

## Metric horizon

`path_generator` accumulates physical path length rather than assuming a CSV
point spacing:

```yaml
local_path_target_length_m: 14.0
local_path_max_points: 600
```

At roughly `0.03 m` spacing, 14 m is about 467 points. The simulator keeps the
10 m LiDAR cap while allowing the known path and centerline horizon to extend
farther:

```yaml
scan_range_cap_m: 10.0
planning_distance_m: 14.0
```

The requested detection reach is automatically reduced only when the current
raw path would otherwise be too short to contain the detected obstacle model,
buffers, post-obstacle hold, minimum smooth return, and final aligned tail. With the
simulator defaults and a complete 14 m raw path, the effective trigger is about
6.9 m. Scan returns farther ahead
are still available for candidate validation, but they do not cause a
predictable premature `NO_SAFE_PATH` before a complete maneuver can fit. The old
point-count behavior remains as the internal `legacy_local_path_points`
fallback when the metric target is set to zero or less. It is intentionally not
shown in the shipped YAML because the positive metric target is authoritative.

The generator stops at the physical target, one unique lap, or the point
ceiling—whichever is reached first. The simulator uses a 14 m target. The
onboard target is 6 m: long enough to fit a held pass and smooth return, while
still materially smaller than the simulator window.

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

The obstacle trigger still checks whether a LiDAR cluster intersects the raw
raceline. Once triggered, candidate search uses centerline-relative coordinates:

- `s`: distance forward along the centerline;
- `d`: signed lateral offset from the centerline, positive to the left.

At every planning station, the raw raceline is converted into its corresponding
centerline offset. That offset is the preferred target, not a hard requirement,
so the planner can move across the track to pass an obstacle and then return.
`centerline_direction: auto` compares the local centerline tangent with the
published raceline and reverses the centerline window when necessary. This
handles generated centerline/raceline CSVs whose stored point orders differ.

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
the car is already on the raceline. The primary planner then:

1. Extracts one forward centerline window; the CSV is already in memory.
2. Places longitudinal stations at a fixed physical spacing.
3. Samples a fixed grid of lateral offsets at each station.
4. Removes samples that violate the selected passing side, vehicle envelope,
   maximum lateral shift, slope, or steering-derived curvature limit.
5. Uses a bounded dynamic-programming beam search to retain only the cheapest
   partial paths.
6. Optimizes each promising offset sequence inside its connected safe corridor.
   The small fixed-iteration solver penalizes lateral slope, curvature,
   curvature change, and deviation from the previously accepted path.
7. Interpolates the optimized station offsets with a shape-preserving cubic.
8. Runs the existing dense curvature and connected-LiDAR-band validator before
   accepting any result.

For each side, a smooth guide leaves the raceline before the hard passing
corridor and returns afterward. This lets the forward-only beam search retain
useful early departure states instead of discovering the lateral shift too
late. The cost then prefers that guide, low lateral slope, low curvature, low
curvature change, useful obstacle clearance, and cycle-to-cycle continuity.
Every smoothing update is projected back between its local left and right
bounds. The guide ends exactly on the future raceline. Safety limits remain
hard constraints; cost weights cannot buy a path through an obstacle.

The recovery length grows with lateral error and is clamped:

```yaml
recovery_min_rejoin_length_m: 2.5
recovery_max_rejoin_length_m: 5.0
recovery_length_gain: 3.0
```

For example, a `0.30 m` lateral error requests approximately
`2.5 + 3.0 * 0.30 = 3.4 m` to converge.

For a centered `0.50 m` obstacle, the simulator hard passing boundary is about `0.55 m`
from the obstacle centreline because the obstacle half-width is `0.25 m` and
the planning half-envelope is `0.24 + 0.06 = 0.30 m`. The search may choose a
wider sample when that improves clearance without creating excessive curvature.

## Candidate safety and side choice

Candidate validation uses the same connected-return concept as the final
trajectory guard, plus the planning reserve. In simulation, a candidate is
rejected when at least `blocked_min_points` connected LiDAR returns enter its
`0.30 m` planning half-band. One isolated beam does not reject the complete
plan. The physical safety envelope remains `0.24 m`.

The planner searches left and right separately. Its primary computation limits
are explicit:

```yaml
lattice_station_step_m: 0.30       # simulator; 0.25 onboard
lattice_lateral_step_m: 0.05
lattice_beam_width: 90              # simulator; 70 onboard
lattice_max_final_candidates: 4
lattice_max_compute_time_ms: 8.0    # per side; 6.0 onboard
max_lateral_shift_m: 0.80
```

The main stability/smoothness weights are:

```yaml
lattice_continuity_weight: 12.0
lattice_curvature_rate_weight: 5.0  # simulator; 2.0 onboard
corridor_smoothing_iterations: 12  # simulator; 8 onboard
corridor_continuity_weight: 12.0
corridor_curvature_weight: 12.0     # simulator; 10.0 onboard
corridor_curvature_rate_weight: 5.0 # simulator; 2.0 onboard
```

Continuity weights pull a material same-side replan toward the previous
accepted path. Curvature and curvature-rate weights remove bends and sudden
steering changes; the hard corridor and final validator still decide safety.

Each final lattice solution is still independently checked against dense path
geometry and the connected LiDAR evidence. Candidates must:

- fit the configured metric window including complete rejoin;
- remain within `max_lateral_shift_m`;
- keep only the locally modified section below the
  wheelbase/steering-derived curvature limit (ordinary raceline curvature
  after the rejoin is not part of candidate rejection);
- retain a valid scan, TF, and raw path.

The simulator keeps the passing offset for another `0.80 m` after the obstacle,
then uses the longest configured return that fits. After that return it follows
the raceline exactly for `1.20 m`, which aligns both position and direction
before the stored path is joined to the refreshed raceline:

```yaml
detour_post_obstacle_hold_m: 0.80
detour_return_min_length_m: 4.0
detour_return_max_length_m: 6.0
lattice_rejoin_alignment_length_m: 1.20
```

The onboard hold is `0.60 m`, its minimum return is `2.5 m`, and its aligned
tail is `0.75 m`. The alignment tail deliberately uses multiple lattice
stations; pinning only the endpoint can reach the raceline at an angle and
produce an abrupt steering correction at handoff. Known raceline geometry may
extend beyond current scan visibility, while the held trajectory is rechecked
as new scan space becomes visible.

When no maneuver is active, the direction decision is open: both sides are
evaluated. A side whose minimum clearance is more than
`side_clearance_tie_m: 0.03` wider wins; the bounded objective chooses only when
the clearances are comparable. Publishing that path closes the decision
immediately. The selected left/right side stays latched for the whole pass. A
material replan searches only that same side and is attracted to the remaining
accepted path. It never silently substitutes the geometrically different
legacy generator. This open/closed rule prevents scan noise from flipping the
avoidance direction.

For a brand-new maneuver only, `lattice_fallback_to_legacy_planner` still lets
the bounded quintic generator make one deterministic attempt if centerline
search cannot produce a final-valid path. Once a centerline maneuver is active,
that fallback is disabled so replanning cannot create a discontinuous swerve.

## Exactly how a plan is held

An accepted candidate is copied into `active_plan_`, assigned a monotonically
increasing `plan_id`, and anchored in the map frame. The geometry is not rebuilt
from every scan. At each control cycle the planner only:

1. Finds the car's monotonically advancing index on the stored path.
2. Removes the already-passed prefix of the locally modified section.
3. Joins its zero-offset rejoin point to the newest raw raceline ahead.
4. Caps the combined command trajectory at the configured metric horizon.
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

The stable plan is replaced and receives a new `plan_id` only when at least one
material condition occurs:

- its remaining widened band becomes blocked;
- the car is more than `plan_deviation_replan_m` from it;
- it reaches the stored-path end without satisfying rejoin conditions;
- it reaches `maximum_plan_hold_sec`;
- a valid replacement is then found from the current pose and scan.

During an avoidance maneuver, replacement search is restricted to the latched
side. After the stored pass anchor, a recovery update may optimize the return
from the current pose. If a valid replacement is not yet available, the planner
keeps the stored path, publishes `REPLAN_PENDING`, applies a low cap, and
confirms an actual blockage/end failure over fresh scan messages:

```yaml
plan_deviation_replan_m: 0.35
active_path_blocked_confirmation_scans: 4
no_safe_path_confirmation_scans: 5
replan_pending_speed_cap_mps: 0.8   # onboard; 1.0 simulator
```

Only `NO_SAFE_PATH_CONFIRMED` or an immediate `CRITICAL_OBSTACLE` requests
Reactive ownership. A single ordinary planning failure no longer latches
Reactive.

## States

| State | `trajectory_mode` | Meaning |
|---|---|---|
| `READY` | `RACELINE` | Raw path clear and car inside recovery threshold |
| `READY` | `AVOIDANCE_DEPARTING` | Moving into the latched passing corridor |
| `READY` | `AVOIDANCE_PASSING` | Holding the selected side through the pass anchor |
| `READY` | `AVOIDANCE_RETURNING` | Executing the optimized return toward the raceline |
| `READY` | `RECOVERING_TO_RACELINE` | Converging from lateral displacement |
| `READY` | `REPLAN_PENDING` | Slowed while a material update/failure is confirmed |
| `NO_SAFE_PATH_CONFIRMED` | `NONE` | Repeated failure; request Reactive |
| `CRITICAL_OBSTACLE` | `NONE` | Immediate safety transfer |
| input/TF/scan failures | `NONE` | Fail closed to Reactive or STOP |

Diagnostics include `plan_id`, `plan_age_sec`, `plan_progress_index`,
`plan_rejoin_index`, confirmation counters, side, obstacle geometry, clearance,
curvature, `side_committed`, `maneuver_phase`, effective detection distance,
and speed cap.

## Bounded computation

Pure raceline following does not run lattice search. The centerline CSV is read
once at startup, and a local centerline window is prepared only when a new plan
or material replan is actually requested. There is no external optimization
process or nonlinear solver.

Search work is bounded by station spacing, lateral spacing, beam width, final
candidate count, and a per-side time budget. Corridor optimization uses 12
fixed iterations in simulation and 8 onboard. The simulator search allows
`8 ms` per side; the lower-power onboard defaults use a `70`-state beam and
`6 ms` per side. The diagnostic fields `lattice_evaluated_transitions` and
`lattice_compute_time_ms` expose actual search work.

The normal held-plan loop only advances progress, appends the current raceline
tail, and validates the stored path. It does not rerun candidate generation.

## Speed and visualization

The physical command envelope is configured once at the top of each YAML:

```yaml
/**:
  ros__parameters:
    command_speed_min_mps: 0.5
    command_speed_max_mps: 10.0
```

The generator and follower use the same range to encode/decode the normalized
rule-speed index. The local planner uses `command_speed_max_mps` as the
ordinary-raceline cap, so there is no separate normal-speed-cap value to keep
synchronized.

Normal raceline demand is tuned only in `path_generator`:

```yaml
rule_curve_min_speed_mps: 1.0
rule_straight_speed_mps: 5.0
rule_speed_curvature_gain: 2.0
rule_speed_curvature_preview_m: 0.50
```

The curvature preview is physical distance, not waypoint count.

The planner exposes only temporary maneuver caps:

```yaml
avoidance_speed_cap_mps: 1.2
recovery_speed_cap_mps: 1.5
replan_pending_speed_cap_mps: 1.0
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
- `centerline_csv_path`, loaded locally once rather than subscribed as a topic

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

The full-stack launch passes matching raceline and centerline files to the
publisher and planner. To select another generated pair explicitly:

```bash
ros2 launch oudtra_driver_bringup full_stack_sim_launch.py \
  raceline_csv_path:=/absolute/path/raceline_points_smooth.csv \
  centerline_csv_path:=/absolute/path/centerline_points_smooth.csv
```

Useful checks:

```bash
ros2 topic echo /path_following_v2/path_status
ros2 topic echo /path_following_v2/trajectory_speed_cap_mps
ros2 topic echo /path_following_v2/local_path --once
ros2 topic echo /drive_arbitration_v2/selected_mode
```

First confirm `plan_id` stays constant through `AVOIDANCE_DEPARTING`,
`AVOIDANCE_PASSING`, and `AVOIDANCE_RETURNING`, the
stored modified section advances without changing shape, the fresh raceline
tail keeps the published light-blue path at the configured physical horizon,
and Reactive is requested only after confirmed no-safe-path or critical
conditions.
