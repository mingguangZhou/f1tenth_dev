# Raceline to Reactive V2 integration design

## 1. Responsibility split

The integration keeps control generation, supervision, and final safety in
separate packages:

```text
centerline_tools -> path_following_v2 -> raceline candidate ----+
                                                               |
/scan -> reactive_control_v2 upper -> reactive candidate -------+-> drive_arbitration_v2
                                                               |       |
/scan + local raceline -> raceline_guard -----------------------+       v
/pf/health (onboard only) --------------------------------------+  selected_cmd + selected_mode
                                                                       |
                                                                       v
                                                    reactive_control_v2 lower
                                                                       |
                                                                       v
                                                                    /drive
```

- `centerline_tools` owns static raceline loading and publication.
- `path_following_v2` owns map-based local-path generation, rule/RL speed, and
  the raceline candidate command.
- `reactive_control_v2/upper_corridor_follower` runs continuously as a warm
  localization-free candidate.
- `drive_arbitration_v2/raceline_guard` only answers whether current scan
  endpoints interfere with the upcoming raceline band.
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
path generator status is fresh and READY
AND path follower status is fresh and DRIVING
AND raceline candidate command is fresh and finite
AND raceline guard status is fresh and CLEAR
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
    fresh(path_generator status) AND path_generator state == READY
    AND fresh(path_follower status) AND path_follower state == DRIVING
    AND fresh(raceline candidate command)
    AND finite(command speed and steering)
```

The guard and PF checks are independent requirements described below.

### Path-generator states

`/path_following_v2/path_status` publishes a heartbeat with diagnostic name
`path_following_v2/path_generator`:

| State | Meaning |
|---|---|
| `WAITING_RACELINE` | No global raceline has been received. |
| `RACELINE_INVALID` | A received raceline has invalid row sizing or is empty. |
| `TF_UNAVAILABLE` | `map -> base_link` (or simulator robot frame) cannot be resolved. |
| `LOCAL_PATH_INVALID` | Fewer than two local poses could be produced. |
| `READY` | A local path and rule-speed index were published this cycle. |

The important safety correction is the `TF_UNAVAILABLE` behavior. Previously,
TF failure selected raceline index zero and still published a plausible local
path. The updated node publishes the failure state and publishes neither a new
local path nor a new speed index for that cycle.

### Path-follower states

`/path_following_v2/status` publishes a heartbeat with diagnostic name
`path_following_v2/path_follower`:

| State | Meaning |
|---|---|
| `PATH_MISSING` / `PATH_STALE` | No usable live local path. |
| `SPEED_INPUT_STALE` | Rule-speed input is absent, invalid, or stale. |
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

Existing controller tuning, including the follower's own `path_timeout_sec`,
speed limits, lookaheads, steering limits, and the 80-point local horizon, was
left unchanged. Arbitration can therefore reject the chain earlier than the
follower's internal timeout without altering its standalone tuning.

## 5. `RACELINE_BLOCKED`

The guard implements the deliberately lightweight expanded-raceline method.
It transforms the upcoming local path into the LaserScan frame and checks the
minimum point-to-segment distance for valid scan endpoints:

```text
interference if:
distance(scan endpoint, upcoming local path) <=
    vehicle_width / 2 + lateral_safety_margin
```

With the initial parameters:

```yaml
vehicle_width_m: 0.32
lateral_safety_margin_m: 0.04
guard_half_width_m: 0.20  # computed
guard_distance_m: 4.0
guard_path_step_m: 0.10
blocked_min_points: 3
blocked_confirmation_scans: 2
clear_confirmation_scans: 3
critical_block_distance_m: 0.80
```

No polygon union or oriented vehicle rectangles are constructed. The local
path is downsampled before the point-to-segment checks. Ordinary blockage needs
two distinct scans; a qualifying cluster within the critical distance bypasses
that delay. The guard reports `UNKNOWN`, `CLEAR`, or `BLOCKED` on
`/drive_arbitration_v2/raceline_guard_status`.

`UNKNOWN` never means clear. A missing path/TF can lead to Reactive mode; a
missing or invalid LaserScan also makes the Reactive chain unavailable, so the
combined result is STOP.

The requested guard distance can exceed the currently available 80-point local
path. The guard reports `checked_path_reach_m` so this is visible during tests;
it checks all available forward path instead of inventing geometry. The local
horizon can be increased later after baseline integration tests.

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
# Onboard: includes particle_filter; requires /pf/health.
ros2 launch oudtra_driver_bringup full_stack_onboard.launch.py

# Simulator: no particle_filter; ignores /pf/health.
ros2 launch oudtra_driver_bringup full_stack_sim.launch.py
```

Both launches start the raceline publisher, path generator/follower, Reactive
upper, guard, arbitrator, and Reactive lower. The final command chain is:

```text
/drive_arbitration_v2/selected_cmd
    -> lower_safety_controller
    -> /drive
```

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
ros2 launch oudtra_driver_bringup full_stack_sim.launch.py \
  path_follower_log_level:=debug
```

`output="screen"` only routes messages to the terminal; the per-node log level
sets which severities appear. The raceline-publisher and particle-filter nodes
are started through their own included launch files, so their internal logger
levels are not overridden by these six arguments.

## 10. First integration checks

Build all packages in the same Foxy workspace, source the overlay, and inspect:

```bash
ros2 topic echo /path_following_v2/path_status
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
