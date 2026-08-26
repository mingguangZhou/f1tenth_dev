# path_following_v2

ROS 2 Foxy localization-based raceline following with a persistent local
trajectory planner.

## Data flow

```text
/raceline_waypoints
        -> path_generator
        -> /path_following_v2/raceline_local_path  (raw, metric horizon)
/raceline_path (full static path from the same racing-line CSV)
                         + /scan + map + map->base TF
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

The final local path is the raw racing line or a persistent map-frame trajectory
planned directly in racing-line-relative coordinates. The transient-local full
`/raceline_path` is the sole Frenet reference, so the normal and rejoin target is
always `d=0`; no centerline CSV is loaded by the planner. While a local plan is
active, its remaining modified section is joined to the newest raw racing-line
window so the controller continues to receive a full forward trajectory.
Existing follower, guard, and arbitrator topic interfaces remain unchanged.

`config/path_following_v2.yaml` is the canonical behavior profile for both
simulation and onboard operation. `path_following_v2_sim.yaml` changes only
simulated time, frames, and TF timing; it contains no planning or speed tuning.

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

Candidate planning adds a `0.05 m` hard reserve outside that physical envelope,
so planned paths stay at least `0.29 m` from connected obstacle returns. This
reserve absorbs scan noise and newly exposed obstacle surfaces before they
reach the physical safety boundary.

## Metric horizon

`path_generator` accumulates physical path length rather than assuming a CSV
point spacing:

```yaml
local_path_target_length_m: 10.0
local_path_max_points: 350
```

At roughly `0.03 m` spacing, 10 m is about 334 points. Both platforms use the
same 10 m known-path, LiDAR, and planning ceilings:

```yaml
scan_range_cap_m: 10.0
planning_distance_m: 10.0
```

The requested detection reach is automatically reduced only when the current
raw path would otherwise be too short to contain the detected obstacle model,
buffers, post-obstacle hold, minimum smooth return, and final aligned tail.
Scan returns farther ahead remain available for candidate validation, but do
not trigger a maneuver that cannot fit in the current raw window. The old
point-count behavior remains as the internal `legacy_local_path_points`
fallback when the metric target is set to zero or less.

The generator stops at the physical target, one unique lap, or the point
ceiling—whichever is reached first. Both platforms use the same 10 m target.

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
racing line. Once triggered, candidate search uses racing-line-relative coordinates:

- `s`: distance forward along the global racing line;
- `d`: signed lateral offset from the racing line, positive to the left.

At every unmodified planning station the preferred target is exactly `d=0`.
The planner may move across the track to pass an obstacle, then returns to zero.
The runtime racing-line publisher is the single owner of CSV parsing, traversal
direction, and loop closure; both the raw-window generator and planner consume
its outputs. Before lattice search, the planner samples the global reference
against the raw window and fails closed if their geometry differs by more than
`raceline_reference_match_tolerance_m` (default `0.10 m`).

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
the car is already on the raceline. In the general Frenet planner, the primary
planner then:

1. Extracts one forward window from the full transient-local racing-line path.
2. Places longitudinal stations at a fixed physical spacing.
3. Samples a fixed grid of lateral offsets at each station.
4. Removes samples that violate the selected passing side, map-derived
   asymmetric corridor, vehicle envelope, maximum lateral shift, slope, or
   steering-derived curvature limit.
5. Uses a bounded dynamic-programming beam search to retain only the cheapest
   partial paths.
6. Optimizes each promising offset sequence inside its connected safe corridor.
   The small fixed-iteration solver penalizes lateral slope, curvature,
   curvature change, and deviation from the previously accepted path.
7. Interpolates the optimized station offsets with a shape-preserving cubic.
8. Runs the existing dense curvature and connected-LiDAR-band validator before
   accepting any result.

The canonical fixed-obstacle profile enables `static_obstacle_fast_mode`. It
first orders the two sides with a cheap occupancy-map clearance score and tries
at most two analytic offsets on the preferred side. A side falls back to one
bounded, full-return lattice attempt only when those analytic candidates fail.
With `static_first_valid_side: true`, a valid result ends the search without
evaluating the other side. The same dense map, curvature, and LiDAR validators
remain mandatory regardless of which backend generated the candidate.

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

For a centered `0.50 m` obstacle, the hard passing boundary is about `0.54 m`
from the obstacle centreline because the obstacle half-width is `0.25 m` and
the planning half-envelope is `0.24 + 0.05 = 0.29 m`. The search may choose a
wider sample when that improves clearance without creating excessive curvature.

## Candidate safety and side choice

Candidate validation uses the same connected-return concept as the final
trajectory guard, plus the planning reserve. A candidate is rejected when at
least `blocked_min_points` connected LiDAR returns enter its `0.29 m` planning
half-band. One isolated beam does not reject the complete plan. The physical
safety envelope remains `0.24 m`.

The planner searches left and right separately. Its primary computation limits
are explicit:

```yaml
static_obstacle_fast_mode: true
static_analytic_candidates_per_side: 2
static_analytic_extra_clearance_m: 0.04
static_first_valid_side: true
planning_scan_pool_size: 2
lattice_fallback_to_legacy_planner: false
lattice_station_step_m: 0.30
lattice_lateral_step_m: 0.075
lattice_beam_width: 40
lattice_max_final_candidates: 2
lattice_max_compute_time_ms: 6.0
max_lateral_shift_m: 0.90
```

The main stability/smoothness weights are:

```yaml
lattice_continuity_weight: 12.0
lattice_curvature_rate_weight: 2.0
corridor_smoothing_iterations: 4
corridor_continuity_weight: 12.0
corridor_curvature_weight: 10.0
corridor_curvature_rate_weight: 2.0
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

The planner keeps the passing offset for another `0.60 m` after the obstacle,
then uses the longest configured return that fits. After that return it follows
the raceline exactly for `0.75 m`, which aligns both position and direction
before the stored path is joined to the refreshed raceline:

```yaml
detour_post_obstacle_hold_m: 0.60
detour_return_min_length_m: 2.5
detour_return_max_length_m: 5.0
lattice_rejoin_alignment_length_m: 0.75
```

The alignment tail deliberately uses multiple lattice stations; pinning only
the endpoint can reach the raceline at an angle and produce an abrupt steering
correction at handoff. Known raceline geometry may extend beyond current scan
visibility, while the held trajectory is rechecked as new scan space becomes
visible.

With the canonical static profile, the direction decision starts by sampling
map clearance on both sides. The higher-scoring side is searched first; the
other side is searched only if the first side has no full-validation-safe
candidate. Publishing a path closes the decision immediately. The selected
left/right side stays latched for the whole pass, and a material replan searches
only that same side. This avoids duplicate work in the common case without
allowing a map score alone to accept a trajectory.

The generic profile, selected with `static_obstacle_fast_mode: false`, retains
the exhaustive two-side comparison and moving-obstacle logic. The canonical
profile also sets `lattice_fallback_to_legacy_planner: false`; static fast mode
therefore never enters the geometrically different legacy generator. The
legacy fallback can be restored explicitly for generic experiments, but it is
not part of the bounded onboard contract.

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
keeps the stored path, publishes `REPLAN_PENDING`, applies a reduced cap, and
confirms an actual physical blockage or exhausted path over fresh scan
messages. Entering the extra planning-clearance reserve requests a replan, but
does not by itself declare the physically clear path unsafe:

```yaml
plan_deviation_replan_m: 0.35
active_path_blocked_confirmation_scans: 4
no_safe_path_confirmation_scans: 5
replan_pending_speed_cap_mps: 2.25
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
`plan_rejoin_index`, separate planning-margin and physical-blockage confirmation
counters, side, obstacle geometry, clearance, candidate objective domain,
curvature, remaining-path maximum curvature, `side_committed`,
`maneuver_phase`, effective detection distance, and speed cap. Static-profile
diagnostics additionally report `static_planning_backend`, analytic candidate
count, side-score/analytic/total search times, both map-side scores, lattice
transition count, clearance-grid time, and lattice compute time. The obstacle
trial logger records these fields in both its sampled CSV and event sidecar.

## Bounded computation

Pure racing-line following does not run candidate or lattice search. The
planner receives the full global racing line once through transient-local QoS
and prepares a local Frenet window only when a new plan or material replan is
requested. There is no second CSV parser, external optimization process, or
nonlinear solver. Runtime visualization markers are disabled in the canonical
profile.

The canonical search first tries at most two analytic offsets per attempted
side. A failed side gets exactly one full-return lattice attempt. Its search is
bounded by `0.30 m` station spacing, `0.075 m` lateral spacing, a `40`-state
beam, two final candidates, four fixed corridor-smoothing iterations, and a
`6 ms` lattice compute budget. The budget applies to one lattice attempt; it is
not a bound on the complete planning callback or scan-to-command reaction time.

Planning-only geometry uses conservative two-beam minimum pooling: each pair
contributes its nearest finite in-range return and its original support count.
Obstacle detection, active-path safety checks, and final candidate acceptance
continue to use the latest full-resolution scan. Thus pooling reduces downstream
planning-context and clearance-grid work without weakening the final acceptance
gate. Setting `planning_scan_pool_size: 1` restores exact full-resolution
planning.

The diagnostic fields `static_side_score_time_ms`,
`static_analytic_time_ms`, `static_planning_total_time_ms`,
`lattice_evaluated_transitions`, `lattice_clearance_grid_time_ms`, and
`lattice_compute_time_ms` expose the bounded search work. The static total is
planner candidate-search time, while a `PLANNING_HOLD`-to-outcome interval also
contains callback scheduling, input waiting, and final validation.

The normal held-plan loop only advances progress, appends the current raceline
tail, and validates the stored path. It does not rerun candidate generation.

## Moving obstacles and safe waiting

The canonical static profile deliberately skips velocity tracking and
rolling/open-ended pass generation. Moving returns are treated as instantaneous
obstacles by the normal collision validators, but their future motion is not
predicted. Set `static_obstacle_fast_mode: false` when moving-obstacle behavior
is required; that generic profile tracks the selected obstacle in the map frame
and keeps the chosen side open until the target is confirmed behind the car.
Candidate paths in both profiles must remain inside the occupancy-map corridor.

If the committed side is temporarily blocked, the planner publishes a safe
path prefix and slows to a stop while retrying that side. This remains a primary
planner state (`FOLLOWING_OBSTACLE`) and avoids a reactive steering handoff.
Refreshed raceline tails are appended only through a gap-, heading-, and
curvature-continuous splice.

The current safe-yield profile uses a `1.0 m` standoff, retains `0.5 m` of
terminal path for controlled braking, and permits at most `2.25 m/s`:

```yaml
yield_standoff_m: 1.00
yield_min_path_length_m: 0.50
yield_max_speed_mps: 2.25
yield_deceleration_mps2: 2.0
```

## Speed and visualization

The physical command envelope is configured once at the top of each YAML:

```yaml
/**:
  ros__parameters:
    command_speed_min_mps: 0.5
    command_speed_max_mps: 9.0
```

The generator and follower use the same range to encode/decode the normalized
rule-speed index. The local planner uses `command_speed_max_mps` as the
ordinary-raceline cap, so there is no separate normal-speed-cap value to keep
synchronized.

Normal raceline demand is tuned only in `path_generator`:

```yaml
rule_curve_min_speed_mps: 0.9
rule_straight_speed_mps: 4.5
rule_speed_curvature_gain: 2.0
rule_speed_curvature_preview_m: 0.50
```

The curvature preview is physical distance, not waypoint count.

The shared maneuver settings are:

```yaml
avoidance_speed_cap_mps: 3.15
recovery_speed_cap_mps: 3.6
replan_pending_speed_cap_mps: 2.25
maneuver_lateral_acceleration_limit_mps2: 3.24
```

The avoidance and recovery values are ceilings, not fixed maneuver speeds. On
each cycle the planner measures the maximum curvature still ahead in the held
local plan and computes:

```text
curvature_cap = sqrt(lateral_acceleration_limit / remaining_maximum_curvature)
maneuver_cap = min(configured ceiling, command maximum, curvature_cap)
```

This lets a gentle detour approach the normal raceline demand while slowing a
tight detour enough to keep estimated lateral acceleration bounded. As the car
passes the curved part, only the remaining geometry is considered, so the cap
rises progressively during a smooth return. The follower's existing command
rate limiter controls the actual acceleration. These ceilings and the
`3.24 m/s^2` lateral-acceleration limit are shared by both platforms.

`speed_policy_mode` selects rule-only (`0`) or rule plus a fresh RL speed
residual (`1`). The established future RL structure is unchanged: bounded
learning modifies the rule baseline, while the physical envelope, command-rate
limit, maneuver cap, arbitrator, and lower safety controller remain outside the
policy.

The follower applies the planner cap after rule/RL speed calculation:

```text
final_speed = min(rate_limited_rule_plus_RL_speed, fresh dynamic planner cap)
```

The final authorized trajectory marker uses:

- dark blue: ordinary raceline;
- light blue (`0.20, 0.80, 1.00`): a local replan currently selected by the
  arbitrator and passed through the lower controller;
- red: Reactive upper trajectory.

Candidate debug markers use light blue for the selected local candidate, grey
for the alternative, and red for obstacle-cluster returns.

## Nodes and topics

### `path_generator_node`

Publishes the raw metric raceline window, rule speed index, and upstream status.

### `local_trajectory_planner_node`

Inputs:

- `/path_following_v2/raceline_local_path`
- `/raceline_path` (full global Frenet reference from the same CSV publisher)
- `/scan`
- `/map`
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

The simulator full-stack launch drives the validated optimized IFAC Roboracer
racing line by default and uses that same published path as the planner's Frenet
frame. To select another racing line explicitly:

```bash
ros2 launch oudtra_driver_bringup full_stack_sim_launch.py \
  raceline_csv_path:=/absolute/path/raceline_points_smooth.csv
```

The retained manually tuned Spielberg simulator raceline remains at
`centerline_tools/output_backup/V0_reward_ppo_speed_spielberg_1000k_20260612/raceline_points_smooth.csv`.

Useful checks:

```bash
ros2 topic echo /path_following_v2/path_status
ros2 topic echo /path_following_v2/trajectory_speed_cap_mps
ros2 topic echo /path_following_v2/local_path --once
ros2 topic echo /drive_arbitration_v2/selected_mode
```

For repeatable fixed-pose obstacle trials, keep the simulator and autonomy
stack running and execute this inside the container:

```bash
/sim_ws/src/path_following_v2/tools/run_obstacle_trials.sh \
  --reuse-stack --obstacle 4 --trials 5 --duration 18
```

The ignored `path_following_v2/trial_logs/` directory receives a sampled CSV,
state-change JSONL, and JSON summary for each trial. The summary reports side
choice, arbitration/failure counts, planning-margin versus physical-blockage
counters, and speed statistics for every planner mode.

After collecting matching baseline and candidate directories, generate the
paired latency/maneuverability report with:

```bash
python3 /sim_ws/src/path_following_v2/tools/analyze_static_profile_ab.py \
  --baseline-dir /tmp/static_fast_ab/baseline \
  --candidate-dir /tmp/static_fast_ab/candidate \
  --raceline-csv /sim_ws/src/centerline_tools/output_backup/ifac_roboracer/raceline_points_optimized.csv \
  --output /tmp/static_fast_ab/report.json
```

The command writes the JSON report plus a Markdown report at the same path with
the `.md` suffix. Its planner compute timings remain separate from the
`PLANNING_HOLD`-to-outcome end-to-end latency proxy.

To verify complete laps rather than a single obstacle approach, run:

```bash
/sim_ws/src/path_following_v2/tools/run_multi_lap_test.sh \
  --laps 3 --timeout 600
```

This resets the car to the configured Spielberg start, unwraps progress around
the closed active raceline, records cross-track error and all existing planner,
guard, arbitrator, Reactive, scan, command, and safety diagnostics, and exits
with failure if the requested lap count is not completed before the timeout.

First confirm `plan_id` stays constant through `AVOIDANCE_DEPARTING`,
`AVOIDANCE_PASSING`, and `AVOIDANCE_RETURNING`, the
stored modified section advances without changing shape, the fresh raceline
tail keeps the published light-blue path at the configured physical horizon,
and Reactive is requested only after confirmed no-safe-path or critical
conditions.
