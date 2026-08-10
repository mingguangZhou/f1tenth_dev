# Raceline to Reactive V2 integration design

## 1. Responsibility split

The integration keeps control generation, supervision, and final safety in
separate packages:

```text
centerline_tools -> raw raceline -> local_trajectory_planner -> primary candidate --+
                                      ^                                     |
                                      | /scan                               |
/scan -> reactive_control_v2 upper -> reactive candidate -------------------+-> drive_arbitration_v2
                                                                            |       |
/scan + final primary path -> raceline_guard -------------------------------+       v
/pf/health (onboard only) --------------------------------------------------+  selected_cmd + selected_mode
                                                                       |
                                                                       v
                                                    reactive_control_v2 lower
                                                                       |
                                                                       v
                                                                    /drive
```

- `centerline_tools` owns static raceline loading and publication.
- `path_following_v2` owns map-based raw-path generation, local left/right
  persistent local replanning, rule/RL speed with a planner cap, and the primary
  candidate command.
- `reactive_control_v2/upper_corridor_follower` runs continuously as a warm
  localization-free candidate.
- `drive_arbitration_v2/raceline_guard` answers whether current scan endpoints
  interfere with the final selected primary path band. The name remains for
  compatibility; the path may be an unchanged raceline or a local replan.
- `drive_arbitration_v2/drive_arbitrator` selects the authorized control mode.
- `reactive_control_v2/lower_safety_controller` is the only final `/drive`
  publisher and retains emergency braking, FTG, low-speed assistance, and
  reverse recovery.

## 2. Modes and command ownership

`/drive_arbitration_v2/selected_mode` is a continuous `std_msgs/UInt8`
heartbeat:

| Code | Mode | Selected command | Lower FTG/reverse authorized |
|---:|---|---|---|
| 0 | `WAITING` | Zero | No |
| 1 | `RACELINE` | `/path_following_v2/nominal_cmd` | No |
| 2 | `REACTIVE` | `/reactive_control_v2/nominal_cmd`, or zero while the lower layer takes FTG | Yes |
| 3 | `STOP` | Zero | No |

In integrated launches, the lower controller uses
`require_arbitration_mode: true`. A missing or stale selected-mode heartbeat is
a STOP condition. An upper `PATH_INVALID` or `BLOCKED` status can request FTG
only while the selected mode is `REACTIVE`. This prevents a warm-standby upper
controller from affecting healthy raceline driving.

Standalone Reactive V2 keeps `require_arbitration_mode: false`, preserving its
previous upper-to-lower behavior without requiring this package.

## 3. Exact fallback expression

The high-level transition request is:

```text
fallback_requested =
    RACELINE_UNAVAILABLE
    OR RACELINE_BLOCKED
    OR PF_INVALID_OR_UNAVAILABLE   # onboard only
```

`REACTIVE` remains latched while any fallback condition is present. The
arbitrator records the trigger classes seen during that Reactive episode:

```text
RACELINE_BLOCKED
PF_INVALID_OR_UNAVAILABLE
RACELINE_UNAVAILABLE
```

When every primary requirement becomes healthy again, a steady-clock recovery
timer starts. The complete primary-ready expression must remain true for the
whole `raceline_recovery_stable_sec` interval. In other words, recovery requires
all of the following simultaneously:

```text
primary trajectory-planner status is fresh and READY
AND path follower status is fresh and DRIVING
AND raceline candidate command is fresh and finite
AND final primary-trajectory guard status is fresh and CLEAR
AND PF health is fresh and state 1 or 2       # onboard only
AND lower safety status is fresh
AND lower safety state is NOMINAL
AND lower safety confirms arbitration mode 2 # still REACTIVE
```

If any requirement fails during the interval, the timer is cancelled and must
start again from zero. Automatic return is also permitted only when the switch
for every trigger class recorded during the Reactive episode is enabled. This
prevents, for example, an episode that began with a blockage but later also saw
a PF failure from being recovered using only the blockage policy.

The current active test configuration is:

```yaml
latch_reactive_mode: true
allow_auto_recovery_from_blocked: true
allow_auto_recovery_from_pf_invalid: true
allow_auto_recovery_from_raceline_unavailable: true
raceline_recovery_stable_sec: 0.5
enable_lower_safety_recovery_coordination: true
lower_status_timeout_sec: 0.30
```

Therefore, after an obstacle has cleared—or after PF/path availability has
recovered—the arbitrator keeps selecting Reactive until the lower controller is
also `NOMINAL`, then requires another continuous 0.5 seconds before changing to
`RACELINE`. Any lower FTG, emergency stop, reverse, settle, stale status, or
mode mismatch resets this timer. The simulator ignores PF health, so the PF
recovery switch has no effect there.

When fresh lower status reports `EMERGENCY_STOP` and confirms arbitration mode
1 (`RACELINE`), the arbitrator treats this as a missed `RACELINE_BLOCKED`
condition and relatches `REACTIVE`. Emergency braking remains active at the
lower layer, but selecting Reactive re-authorizes its existing FTG, dead-end,
and reverse-recovery logic. The lower controller's own reverse-entry and
reverse-exit tests are unchanged.

The parameter name `allow_auto_recovery_from_pf_invalid` covers every PF state
that makes localization unusable: explicit state 3, missing/stale health, and
malformed/unknown health. This is deliberately an active integration-test
setup. It does not yet add separate cross-track-error, heading-alignment, or
handover-speed gates beyond the existing `READY`, `DRIVING`, finite-command,
and clear-guard requirements.

Manual reset remains available at any time:

```bash
ros2 topic pub --once /drive_arbitration_v2/reset \
  std_msgs/msg/Bool "{data: true}"
```

After reset the arbitrator clears the recorded trigger classes, enters
`WAITING`, and reevaluates all primary inputs.
The startup grace (`primary_startup_timeout_sec`, currently 3.0 s) prevents node
startup order from immediately latching Reactive mode.

## 4. `RACELINE_UNAVAILABLE` in detail

The global raceline is static configuration data, not a sensor heartbeat.
`centerline_tools/raceline_publisher` loads its CSV, publishes with reliable
transient-local QoS, and republishes periodically. After a valid raceline has
been cached by `path_generator`, the arbitrator does not declare it unavailable
merely because no new global-waypoint message arrived.

Instead, availability is based on the live output chain:

```text
raceline_available =
    fresh(local-trajectory-planner status) AND planner state == READY
    AND fresh(path_follower status) AND path_follower state == DRIVING
    AND fresh(raceline candidate command)
    AND finite(command speed and steering)
```

The guard and PF checks are independent requirements described below.

### Primary trajectory-planner states

`path_generator_node` publishes an internal 8 m raw path on
`/path_following_v2/raceline_local_path`. `local_trajectory_planner_node` publishes
the final `/path_following_v2/local_path` plus the public
`/path_following_v2/path_status` heartbeat. The diagnostic name remains
`path_following_v2/path_generator` so the existing arbitrator parser does not
need a compatibility-breaking change.

| State | Meaning |
|---|---|
| `READY`, `trajectory_mode=RACELINE` | Raw raceline is clear; unchanged final path and normal cap published. |
| `READY`, `trajectory_mode=AVOIDING` | Executing a persistent obstacle-passing trajectory. |
| `READY`, `trajectory_mode=REJOINING` | Executing its stored return to the raceline. |
| `READY`, `trajectory_mode=RECOVERING_TO_RACELINE` | Converging from lateral displacement. |
| `READY`, `trajectory_mode=REPLAN_PENDING` | Slowed while a material update/failure is confirmed. |
| `NO_SAFE_PATH_CONFIRMED` | Repeated fresh scans found no safe local trajectory. |
| `CRITICAL_OBSTACLE` | Obstacle is too close to begin the configured departure. |
| `SCAN_INVALID` | Scan is absent, stale, malformed, or below the valid-beam threshold. |
| `TF_UNAVAILABLE` | Scan-to-path transform cannot be resolved. |
| `INPUT_INVALID` | Raw path is absent, stale, malformed, or too short. |

The upstream raw generator separately reports raceline/TF/horizon failures on
`/path_following_v2/raceline_path_status`. Those failures stop its raw-path
heartbeat, so the public planner reports `INPUT_INVALID`. On every public
planner failure it publishes a zero trajectory-speed cap while omitting a new
final path. This stops the primary candidate immediately while arbitration
evaluates Reactive availability.

### Path-follower states

`/path_following_v2/status` publishes a heartbeat with diagnostic name
`path_following_v2/path_follower`:

| State | Meaning |
|---|---|
| `PATH_MISSING` / `PATH_STALE` | No usable live local path. |
| `SPEED_INPUT_STALE` | Rule-speed input is absent, invalid, or stale. |
| `SPEED_CAP_STALE` | Required trajectory-planner cap is absent, invalid, or stale. |
| `TF_UNAVAILABLE` | The localization-dependent transform failed. |
| `LOOKAHEAD_INVALID` | Pure pursuit cannot select a forward target. |
| `COMMAND_INVALID` | A computed command is non-finite. |
| `DRIVING` | All required inputs are valid and a finite candidate was published. |

The arbitrator requires both status heartbeats plus the candidate command. A
zero command by itself is never used as proof of health or failure.

Current default arbitration timeouts are:

```yaml
path_status_timeout_sec: 0.30
follower_status_timeout_sec: 0.30
raceline_command_timeout_sec: 0.25
```

Existing pure-pursuit and rule/RL tuning remains unchanged. The raw local
horizon is now physical-distance based (`8.0 m`) and the planner speed cap is
applied after the rule/RL calculation and normal rate limiter. Arbitration can
still reject the chain earlier than the follower's own path timeout.

## 5. Persistent local replanning and `RACELINE_BLOCKED`

An obstacle intersecting the raw raceline first requests local planning, not
Reactive mode. `local_trajectory_planner` clusters adjacent scan returns, calculates
measured obstacle extents in raceline `(s,d)` coordinates, creates quintic left
and right departure/pass/rejoin paths, and validates each complete candidate
against all current finite scan returns plus the steering-derived curvature
limit. A valid candidate stays in arbitration mode `RACELINE` because command
ownership and localization dependence have not changed.

The guard then applies the deliberately lightweight expanded-path method to
the final `/path_following_v2/local_path`:

```text
interference if:
distance(scan endpoint, selected primary path) <=
    vehicle_width / 2 + lateral_safety_margin
```

Common planner/guard/Reactive-upper parameters are:

```yaml
vehicle_width_m: 0.28
lateral_safety_margin_m: 0.10
guard_half_width_m: 0.24  # computed
guard_distance_m: 4.0
guard_path_step_m: 0.10
blocked_min_points: 3
blocked_confirmation_scans: 3
clear_confirmation_scans: 3
critical_block_distance_m: 0.80
```

No polygon union or oriented vehicle rectangles are constructed. The final
path is downsampled before point-to-segment checks. Ordinary blockage needs three
distinct scans; qualifying interference within the critical distance bypasses
that delay. The guard reports `UNKNOWN`, `CLEAR`, or `BLOCKED` on
`/drive_arbitration_v2/raceline_guard_status`.

`UNKNOWN` never means clear. A missing path/TF can lead to Reactive mode; a
missing or invalid LaserScan also makes the Reactive chain unavailable, so the
combined result is STOP.

The raw generator accumulates points until `local_path_horizon_m: 8.0`;
`max_local_path_points: 600` is only a dense/malformed-path failsafe. The guard
checks the first `guard_distance_m: 4.0` of the final path and reports
`checked_path_reach_m` for test visibility.

The final obstacle hierarchy is:

```text
raw raceline -> persistent local replan -> Reactive corridor -> lower FTG/reverse -> emergency stop
```

An accepted local plan is map-frame anchored and identified by `plan_id`. Its
published prefix is trimmed as the car advances, but its remaining geometry is
not regenerated from each scan. It is normally released only after the stored
rejoin index is passed, lateral error is within `0.08 m`, heading error is within
`10 deg`, and those conditions persist for four fresh scans. A 12 s maximum
requests a new current-pose plan; it does not directly release the stored plan.

Material update triggers are confirmed active-path blockage, more than `0.35 m`
deviation from the stored path, plan-end without convergence, or the 12 s stale
backstop. Ordinary failure is reported as `READY/REPLAN_PENDING` with a low
speed cap while three fresh scans are collected. The arbitrator defers a
non-critical guard `BLOCKED` result to this confirmation logic while a local
replan is already executing. A critical guard result, lower emergency stop,
`NO_SAFE_PATH_CONFIRMED`, and invalid planner inputs remain immediate safety
triggers. When Reactive is already latched, guard `CLEAR` is still required for
the existing 0.5 s recovery handover.

`NO_SAFE_PATH_CONFIRMED` and `CRITICAL_OBSTACLE` map to the existing
`RACELINE_BLOCKED` trigger class. Invalid planner inputs map to
`RACELINE_UNAVAILABLE`; final-guard `BLOCKED` also maps to
`RACELINE_BLOCKED`. All make the primary chain non-ready and request Reactive. The existing
Reactive latch and 0.5 s lower-coordinated recovery are unchanged. Recovery may
return to a validated local replan; the raw raceline itself does not need to be clear
when the final selected primary path is safe.

## 6. Localization policy

Onboard configuration uses:

```yaml
require_pf_health: true
```

PF states 1 (`GOOD`) and 2 (`DEGRADED_BUT_USABLE`) permit raceline mode. State
3 (`INVALID`) requests Reactive mode immediately. Missing, stale, malformed, or
unknown PF health also makes localization-dependent raceline driving
unavailable.

Simulator configuration uses:

```yaml
require_pf_health: false
```

The simulator route relies on its ground-truth `map -> ego_racecar/base_link`
transform and does not start or subscribe semantically to particle-filter
health for arbitration.

## 7. Reactive-chain availability and STOP behavior

After a fallback trigger, `REACTIVE` is selected only when the upper status is
fresh and one of these is true:

- upper state is `DRIVING` with a fresh finite reactive command;
- upper state is `PATH_VALIDATION_PENDING` with a fresh finite stop command;
- upper state is `PATH_INVALID` or `BLOCKED`, allowing the authorized lower FTG
  path to take over.

States such as `WAITING_FOR_SCAN` or `INPUT_INVALID`, or a stale upper status,
make the Reactive chain unavailable. The arbitrator selects `STOP` and keeps
the Reactive latch. If the primary chain is not ready when Reactive inputs
recover, it resumes `REACTIVE`. If the primary chain is continuously ready and
its recorded-trigger recovery policy allows return, the normal stable recovery
timer may instead complete and select `RACELINE`.

## 8. Master launches

The dedicated `oudtra_driver_bringup` package owns system composition:

```bash
# Onboard: particle_filter is normally started and checked separately;
# arbitration still requires /pf/health.
ros2 launch oudtra_driver_bringup full_stack_onboard_launch.py

# Simulator: no particle_filter; ignores /pf/health.
ros2 launch oudtra_driver_bringup full_stack_sim_launch.py
```

Both launches start the raceline publisher, raw path generator, local trajectory
planner, follower, Reactive upper, guard, arbitrator, and Reactive lower. The
final command chain is:

```text
/drive_arbitration_v2/selected_cmd
    -> lower_safety_controller
    -> /drive
```

The onboard launch keeps `start_particle_filter:=false` by default. The PF can
still be included for the old one-command behavior by explicitly passing
`start_particle_filter:=true`.

The optional RL speed-inference package was not in the supplied archive, so the
master launch does not invent its package/executable name. It may continue to
run separately on `/rl_speed_inference/speed_residual_mps`; the existing
follower behavior remains unchanged when the residual is absent or stale.

## 9. Terminal logging through the master launch

The master launches keep the high-level decision nodes visible and make the
periodic candidate-controller output quiet by default:

| Launch argument | Default | Intended output |
|---|---|---|
| `drive_arbitrator_log_level` | `info` | Selected-mode and reason transitions. |
| `lower_safety_log_level` | `info` | Lower NOMINAL/FTG/reverse/STOP transitions. |
| `local_trajectory_planner_log_level` | `info` | Planner startup/configuration summary and plan activation/release. |
| `path_generator_log_level` | `warn` | Warnings and errors only. |
| `path_follower_log_level` | `warn` | Warnings and errors only. |
| `reactive_upper_log_level` | `warn` | Upper failure transitions, warnings, and errors. |
| `raceline_guard_log_level` | `warn` | Guard warnings and errors. |

The path-generator local-path summary and path-follower candidate-command
summary are `DEBUG`, not `INFO`. The candidate log is explicitly named
`candidate_cmd` so it cannot be mistaken for the command currently selected by
the arbitrator. Reactive lower periodic reverse debug is disabled in both YAML
files; its state changes remain visible.

For focused debugging, override only the relevant node, for example:

```bash
ros2 launch oudtra_driver_bringup full_stack_sim_launch.py \
  path_follower_log_level:=debug
```

`output="screen"` only routes messages to the terminal; the per-node log level
sets which severities appear. The raceline-publisher and optional
particle-filter nodes are started through their own included launch files, so
their internal logger levels are not overridden by these seven arguments.

## 10. First integration checks

Build all packages in the same Foxy workspace, source the overlay, and inspect:

```bash
ros2 topic echo /path_following_v2/path_status
ros2 topic echo /path_following_v2/trajectory_speed_cap_mps
ros2 topic echo /path_following_v2/status
ros2 topic echo /drive_arbitration_v2/raceline_guard_status
ros2 topic echo /drive_arbitration_v2/status
ros2 topic echo /drive_arbitration_v2/selected_mode
ros2 topic echo /reactive_control_v2/lower_safety_status
ros2 topic info /drive --verbose
```

Confirm that only the lower safety controller publishes `/drive`. Begin at a
reduced raceline speed until the available LiDAR range, guard reach, switching
delay, and measured braking distance have been validated onboard.
