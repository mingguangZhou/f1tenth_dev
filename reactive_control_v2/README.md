# reactive_control_v2

Package version: `0.3.0`

`reactive_control_v2` is a compact ROS 2 Foxy fallback stack for driving
without a global map, localization result, or raceline reference.

The `upper_corridor_follower` node transforms the current LiDAR scan into the
configured `base_frame`, converts it into connected free-space corridor
branches, selects a stable branch, generates a smooth local center path, and
uses pure pursuit to publish a nominal Ackermann command.

The `lower_safety_controller` is the final command gateway. It normally passes
the selected upper command through, applies only very loose absolute sanity
limits, provides a slow Follow-the-Gap (FTG) fallback when the selected command
fails or, when enabled, the upper reports `PATH_INVALID`/`BLOCKED`, and otherwise stops. It
also contains a deliberately simple narrow-forward emergency distance/TTC
brake and a bounded reverse-recovery state machine.

The lower controller deliberately does **not** compare the commanded Ackermann
trajectory against the scan. Reverse recovery uses VESC-derived odometry as its
primary motion feedback; scan-derived motion and PF switching remain future
extensions.

## I/O

### Inputs

| Topic | Type | Required | Purpose |
|---|---|---:|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | Yes | Local free-space geometry |
| `/ego_racecar/odom` (sim) or `/odom` (onboard) | `nav_msgs/msg/Odometry` | Required for reverse | VESC/simulator speed feedback and reverse distance/stop confirmation |
| `/reactive_control_v2/enable` | `std_msgs/msg/Bool` | No by default | Reserved external mode-enable input |
| `/reactive_control_v2/selected_cmd` | `ackermann_msgs/msg/AckermannDriveStamped` | Yes for nominal mode | Command selected by the upper stack or drive arbitrator |
| `/reactive_control_v2/status` | `diagnostic_msgs/msg/DiagnosticArray` | No | Lets the lower layer recognize upper `PATH_INVALID`/`BLOCKED` explicitly |

### Outputs

| Topic | Type | Purpose |
|---|---|---|
| `/reactive_control_v2/nominal_cmd` | `ackermann_msgs/msg/AckermannDriveStamped` | Upper corridor follower's nominal command |
| `/reactive_control_v2/local_path` | `nav_msgs/msg/Path` | Smoothed selected corridor center path |
| `/reactive_control_v2/markers` | `visualization_msgs/msg/MarkerArray` | Corridor fill/edges, lookahead point, and steering arrow |
| `/reactive_control_v2/status` | `diagnostic_msgs/msg/DiagnosticArray` | State, stop reason, corridor reach/width, and current command |
| `/reactive_control_v2/safe_cmd` | `ackermann_msgs/msg/AckermannDriveStamped` | Lower controller's final safe command; remapped to `/drive` by the simulator launch |
| `/reactive_control_v2/lower_safety_status` | `diagnostic_msgs/msg/DiagnosticArray` | Lower mode/reason plus recovery-relevant evidence |

The standalone simulator launch wires the nodes as follows:

```text
upper_corridor_follower /nominal_cmd
              -> lower_safety_controller /selected_cmd
              -> /drive
```

When `drive_arbitration` is used, configure its output as
`/reactive_control_v2/selected_cmd` and keep the lower output as the only source
connected to `/drive`. Also set `enable_fallback_on_upper_failure_status: false` until the
arbitrator publishes status for its actually selected source; otherwise an
unselected upper-corridor `BLOCKED` status could override a healthy raceline
command.

Upper status names include `DRIVING`, `PATH_VALIDATION_PENDING`, `PATH_INVALID`, `BLOCKED`,
`WAITING_FOR_SCAN`, `TF_UNAVAILABLE`, `INPUT_INVALID`, `ODOM_STALE`, and
`DISABLED`. Every non-driving upper state publishes zero speed.

Lower modes are `NOMINAL`, `FALLBACK_FTG`, `EMERGENCY_STOP`,
`REVERSE_RECOVERY`, and `RECOVERY_SETTLE`.

## Lower safety controller

The lower decision order is intentionally short and deterministic:

1. Require a fresh, sufficiently valid scan. Otherwise publish STOP.
2. Apply the narrow-forward emergency distance/TTC brake. This check is
   independent of commanded steering and does not predict a swept path.
3. If the selected command is fresh and finite, pass it through after the two
   loose absolute clamps.
4. If the command is stale/non-finite, generate a conservative FTG command. A
   fresh upper `PATH_INVALID` or `BLOCKED` also requests FTG only when
   `enable_fallback_on_upper_failure_status: true`.
5. FTG bubbles the nearest obstacle, scores complete gaps using width and
   depth, and targets the centre of the deepest region inside the best gap.
6. If FTG cannot find a sufficiently wide, clear gap, publish STOP.
7. Apply bounded, hysteretic low-speed assistance to eligible nonzero forward
   or reverse demands when VESC odometry remains in the configured stall zone.
8. If a forward dead end or command-versus-VESC-speed mismatch persists, and
   scan/odometry/reverse-side evidence are healthy, enter bounded reverse.
9. Stop reversing after stable forward FTG recovery or the configured reverse
   limit, then publish zero command until the vehicle is fully stationary.

A valid zero-speed command is not automatically treated as a controller
failure. This preserves intentional upper stops. Only an explicit eligible
upper status or a stale/non-finite command requests FTG.

`enable_fallback_on_upper_failure_status` is the clearer replacement for the
old `use_upper_status_fallback` name:

- `true`: a fresh upper `PATH_INVALID` or `BLOCKED` makes the lower controller
  use FTG if a usable gap exists, otherwise STOP;
- `false`: the lower ignores those upper states and passes a fresh finite upper
  STOP command through. Its own scan, TTC, timeout, and malformed-command safety
  checks remain active.

The supplied standalone stack YAML sets this to `true`.

The supplied final command limits are:

```yaml
absolute_speed_limit_mps: 20.0
absolute_steering_limit_deg: 25.0
```

The speed value is a loose sanity bound. Steering is the approximate physical
limit. Finite excesses are clamped; `NaN`, infinity and timeouts request FTG.
Negative finite nominal speed is allowed by this gateway so the interface is
compatible with reverse commands. Ordinary FTG remains forward-only; only the
bounded recovery state publishes negative speed.

The physical and fallback steering clamps are both set to approximately
`+/-25 degrees`. The upper follower retains its existing, smaller tuned limit.

The corrected conservative FTG keeps the existing v0.2.3 scan classification,
minimum-clearance rule, nearest-obstacle bubble, emergency brake, and low-speed
range. Its selection is now:

1. Divide the remaining free beams into continuous gaps.
2. Reject gaps narrower than `fallback_min_gap_width_deg`.
3. Score each remaining gap mainly by width, then by mean and maximum depth;
   use only a small penalty for turning away from straight ahead.
4. Select beams at least `fallback_deepest_region_ratio` times the maximum
   depth in the winning gap.
5. Aim at the weighted centre of that deepest region and clamp steering to
   `fallback_steering_limit_deg`.

This removes the old tie-break that chose the deepest beam nearest zero angle,
which could make the car continue almost straight along the inside edge of a
large gap.

The lower status includes the mode/reason, VESC speed health, low-speed-assist
request/output/shortfall, front emergency state, raw and stable FTG availability,
reverse attempt/distance/duration,
reverse-side evidence, and the fallback target.

FTG terminal tuning information is independent of the upper follower's
`full_terminal_debug`. It is disabled by default:

```yaml
fallback_terminal_debug: false
fallback_terminal_debug_period_sec: 0.50
```

When enabled, it periodically prints scan validity, nearest obstacle and bubble
angle, selected gap width/depth/score, target angle/range, and final FTG speed
and steering. Keep it off for normal running to avoid unnecessary terminal and
ROS log output.

## Low-speed assistance

The lower controller can temporarily raise a nonzero forward or reverse demand
when VESC odometry shows that the vehicle remains in the low-speed stall zone:

```yaml
enable_low_speed_assist: true
low_speed_assist_demand_max_mps: 1.00
low_speed_assist_output_mps: 1.00
low_speed_assist_entry_shortfall_mps: 0.50
low_speed_assist_stall_speed_mps: 0.30
low_speed_assist_confirmation_sec: 0.30
low_speed_assist_exit_shortfall_mps: 0.20
```

`low_speed_assist_demand_max_mps` is only the eligibility ceiling.
`low_speed_assist_output_mps` independently defines the forced absolute command,
so the supplied output is `+1.00 m/s` for a forward request and `-1.00 m/s` for
a reverse request. Shortfall is always nonnegative and magnitude based:

```text
max(0, |requested speed| - |measured speed|)
```

Assistance never turns a zero demand into motion and is disabled during
`EMERGENCY_STOP` and `RECOVERY_SETTLE`. It requires fresh scan and odometry,
preserves steering during ordinary forward control, and repeats the emergency
distance/TTC check using the assisted forward speed. It exits when shortfall is
at or below `low_speed_assist_exit_shortfall_mps`.

## Reverse recovery

Reverse recovery is owned entirely by `lower_safety_controller`, which remains
the only publisher of the final command in `stack` and `lower` modes.

All lower-controller freshness checks, confirmation durations, recovery
durations, distance integration intervals, and diagnostic throttles use a
monotonic steady clock. ROS time is used only for outgoing message headers.
Consequently, a missing, paused, or reset simulator `/clock` cannot freeze the
safety state machine, and the controller does not alter `/clock` or the timing
behavior of any other ROS node.

Two independent conditions may request reverse:

1. **Persistent dead end:** the output is stopped, the VESC speed confirms that
   the vehicle is stationary, and either the front emergency brake is active or
   FTG has no valid route for `dead_end_confirmation_sec`.
2. **Physical stuck inference:** a meaningful forward command is available but
   VESC-reported speed remains below `stuck_speed_threshold_mps` for
   `stuck_confirmation_sec`.

Both require a fresh valid scan, fresh finite odometry, remaining recovery
attempts, and acceptable available rear-side scan evidence. Stale scan or
odometry always produces STOP and never initiates reverse.

Reverse uses `-reverse_speed_mps` with zero steering. Low-speed assistance may
temporarily replace its magnitude with `low_speed_assist_output_mps` while
preserving the negative direction.
The reverse remains bounded by both
`reverse_max_distance_m` and `reverse_max_duration_sec`.

During reverse, FTG continues to be evaluated every control cycle. Reversing
stops only after the configured minimum movement/time and five consecutive
cycles of a valid FTG route with the forward emergency condition cleared, or
when a safety/maximum limit is reached. `RECOVERY_SETTLE` then holds zero speed
until VESC feedback stays within the stationary threshold for
`recovery_settle_time_sec`. Forward motion resumes through FTG; the upper
corridor follower regains control later through its existing recovery
hysteresis.

The attempt counter resets only after measured forward speed reaches
`reverse_attempt_reset_speed_mps` and persists for
`reverse_attempt_reset_forward_time_sec`. This prevents rapid repeated reverse
oscillation in an unresolved dead end.

The Hokuyo's available rear-side beams do not observe directly behind the car.
They are therefore an additional abort check, not proof of an obstacle-free
rear path. Recovery remains deliberately slow, short, distance/time limited,
and intended to retrace recently occupied space.

Rate-limited reverse diagnostics are disabled by default onboard:

```yaml
reverse_terminal_debug: false
reverse_terminal_debug_period_sec: 0.50
```

The supplied simulator YAML enables this diagnostic temporarily. While stopped,
it prints the subscribed odometry age and speed, stationary/dead-end timer, FTG
recovery count, and reverse-side gate. This makes a missing or stale simulator
odometry topic visible instead of leaving an unexplained emergency STOP.

The simulator YAML also sets `reverse_require_side_clearance: false`: the
simulated scan is not treated as a reliable rear-safety sensor, and some scan
models provide no usable side-rear beams. The onboard YAML retains
`reverse_require_side_clearance: true`.

Set `enable_reverse_recovery: false` to retain the previous forward-only lower
controller behavior while keeping all other FTG and emergency settings.

By default, the upper terminal prints one concise warning only when the planner
enters a stop state or its stop reason changes. Set `full_terminal_debug: true`
to restore the complete transition and periodic diagnostics. In that mode,
`terminal_status_period_sec` (default `2.0 s`) controls the periodic interval.

For `PATH_INVALID`, version `0.1.3` reports the first failed segment for both
the smoothed path and the raw midpoint fallback. Each failure includes the
segment endpoints, failed sample `(x, y)`, segment heading, and one stable check
code:

- `OUTSIDE_PLANNING_ANGLE`
- `BEAM_INDEX_OUT_OF_RANGE`
- `BEAM_UNOBSERVED`
- `BEYOND_OBSERVED_RANGE`
- `OBSTACLE_ENVELOPE_COLLISION`

For an envelope collision, the line prints `clearance=<actual><<required> m`
and the nearest obstacle point in `base_frame`.
For a range rejection, it prints the candidate point range, observed beam range,
and configured endpoint margin. The same fields are published on the status
topic. A red RViz sphere marks the smoothed-path failure; a magenta sphere marks
the raw-path failure.

## Swept-path validation and hysteresis

The upper follower validates densely sampled points along the smoothed path. If
that fails, it validates the raw corridor-midpoint path. The feature is
controlled explicitly in the upper YAML:

```yaml
enable_swept_path_validation: true
swept_path_failure_confirmation_cycles: 1
swept_path_recovery_confirmation_cycles: 2
```

With the supplied values, the first cycle in which both paths fail immediately
latches `PATH_INVALID`, publishes zero speed, and requests lower FTG when
`enable_fallback_on_upper_failure_status` is true. Once latched, two consecutive
valid path cycles are required before returning to `DRIVING`.
Unrelated states such as `BLOCKED`, invalid input, TF failure, and the lower
controller's independent emergency brake do not use these counters.

For controlled low-speed debugging only:

```yaml
enable_swept_path_validation: false
```

This skips validation of both smoothed and raw paths, clears its hysteresis
state, and prevents the upper from generating swept-path `PATH_INVALID` or
`PATH_VALIDATION_PENDING`. Consequently, there is no swept-path status for the
lower controller to turn into FTG. Other upper planning checks and all lower
safety checks are unchanged.

## Algorithm

1. Resolve the existing static TF from the LaserScan frame to `base_frame`,
   transform scan endpoints, then crop and lightly median-filter the scan.
2. At regular forward slices, test laterally sampled vehicle-center positions.
3. A position is usable only when it is observed by LiDAR and remains at least
   `vehicle_width / 2 + lateral_safety_margin` from every measured obstacle.
4. Join overlapping free intervals between consecutive slices into corridor
   branches.
5. Keep the previously chosen side while it remains sufficiently good;
   otherwise select by forward reach, then minimum width, then heading change.
6. Smooth the interval midpoints spatially and temporally, clamping every point
   back inside the current corridor after each smoothing operation.
7. When enabled, validate the smoothed path and then the raw fallback path;
   apply entry/recovery hysteresis only to this swept-path result.
8. Select the first path point beyond `lookahead_distance_m` and apply the same
   pure-pursuit relationship as the previous path-following controller:

   ```text
   curvature = 2 * target_y / target_distance^2
   steering  = atan(wheelbase * curvature)
   ```

9. Interpolate speed from `velocity_max_mps` at zero steering to
   `velocity_min_mps` at maximum steering. A short visible corridor applies an
   additional linear slowdown and ultimately commands zero.

No explicit left/right boundary detector is required. Walls, cones, obstacles,
and vehicles are all treated as occupied scan geometry.

## Build

Copy this package into the ROS workspace source directory:

```bash
cp -r reactive_control_v2 /sim_ws/src/
cd /sim_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select reactive_control_v2
source install/setup.bash
```

### Updating an existing build

When replacing an older copy, perform one package-scoped clean build. This is
important because ZIP extraction preserves timestamps and CMake can otherwise
keep an older executable whose object file appears newer than the replaced
source:

```bash
cd /sim_ws
rm -rf build/reactive_control_v2 install/reactive_control_v2
colcon build --packages-select reactive_control_v2
source install/setup.bash
```

The correct upper executable starts with:

```text
reactive_control_v2 v0.3.0 upper_corridor_follower ready: ... swept_path_validation=true (1 fail/2 recover), full_terminal_debug=false
```

The lower executable also prints:

```text
reactive_control_v2 v0.3.0 lower_safety_controller ready: ... reverse_recovery=true, low_speed_assist=true, ftg_debug=false
```

With upper `full_terminal_debug: true`, it also confirms receipt of the first
LaserScan and prints the first planning result. If the startup line does not contain `v0.3.0`, the shell is still
resolving an older installed copy. Check it with:

```bash
ros2 pkg prefix reactive_control_v2
```

If dependencies are missing:

```bash
cd /sim_ws
rosdep install -i --from-paths src --rosdistro foxy -y
```

## Simulator test

Start the F1TENTH simulator first. In a second terminal inside the same
container:

```bash
cd /sim_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch reactive_control_v2 reactive_control_v2_sim_launch.py
```

The default `drive_command_source:=stack` starts both nodes. The upper command
enters the lower safety gateway, and only the lower `safe_cmd` is remapped to
`/drive`:

```bash
ros2 launch reactive_control_v2 reactive_control_v2_sim_launch.py \
  drive_command_source:=stack
```

To bypass the lower controller and let only the upper follower publish `/drive`
for debugging:

```bash
ros2 launch reactive_control_v2 reactive_control_v2_sim_launch.py \
  drive_command_source:=upper
```

In this mode, the lower controller is not started. Use it only for controlled
tests because lower scan validation, FTG, emergency braking, and final sanity
limits are all bypassed.

To run only the lower controller's built-in FTG fallback:

```bash
ros2 launch reactive_control_v2 reactive_control_v2_sim_launch.py \
  drive_command_source:=lower
```

In lower-only mode, the upper follower is not started. The lower selected-command
input is intentionally isolated, so `selected command unavailable or stale`
selects `FALLBACK_FTG`. This is the expected lower-only behavior, not an error.
Lower emergency scan checks remain active.

The three modes are therefore:

| `drive_command_source` | Nodes started | Command sent to `/drive` |
|---|---|---|
| `stack` (default) | Upper + lower | Lower gateway output; normally passes the upper command |
| `upper` | Upper only | Upper corridor-following command |
| `lower` | Lower only | Lower built-in FTG fallback command |

To inspect the complete upper-plus-lower stack without controlling the car:

```bash
ros2 launch reactive_control_v2 reactive_control_v2_sim_launch.py \
  drive_topic:=/reactive_control_v2/test_drive
```

To supervise an external drive-arbitrator output instead of the included upper
command, use `stack` and pass its topic:

```bash
ros2 launch reactive_control_v2 reactive_control_v2_sim_launch.py \
  drive_command_source:=stack \
  nominal_cmd_topic:=/drive_arbitration/selected_cmd
```

Make sure that arbitrator output is not simultaneously connected directly to
`/drive`, and set `enable_fallback_on_upper_failure_status: false` in the lower-controller
YAML for this arrangement.

The simulator launch intentionally does not start another RViz process. In the
RViz window already opened by the simulator, add:

- a `MarkerArray` display on `/reactive_control_v2/markers`;
- optionally, a `Path` display on `/reactive_control_v2/local_path`.

The selected corridor fill and boundaries are dark orange so they remain
distinct from the center path.

Version `0.1.2` explicitly sets the `TRIANGLE_LIST` fill marker scale to
`1.0` on all axes. This removes RViz's `corridor_fill/0: Scale of 0` warning
without changing the corridor geometry.

Version `0.1.3` adds point-level swept-path failure diagnostics and RViz
failure-point markers. It does not weaken the safety envelope or otherwise
change corridor/path selection.

Version `0.1.4` makes full terminal diagnostics optional, avoids constructing
path/status/marker messages when they have no subscribers, limits RViz marker
publication to 10 Hz by default, reuses the median-filter scratch buffer, and
uses an optimized `RelWithDebInfo` build when no build type is supplied.
Planning, obstacle clearance, path selection, steering, and speed logic are
unchanged.

Version `0.1.5` renames the source, executable, and ROS node to
`upper_corridor_follower`. It uses the existing TF to transform scan geometry
into `base_frame`, so path origin `(0,0)`, control, validation, and RViz output
are all referenced to `base_link`. If TF is unavailable, it commands STOP.

The supplied simulator YAML uses:

```yaml
base_frame: "ego_racecar/base_link"
transform_timeout_sec: 0.05
```

For the onboard frame tree, use:

```yaml
base_frame: "base_link"
```

Do not publish another static transform; the node consumes the transform already
provided by the simulator or onboard launch.

If the scan frame is unexpected, confirm it with:

```bash
ros2 topic echo /scan --once
```

## ROS launch log directory

The startup line `All log files can be found below ...` is emitted by the ROS 2
launch framework, not by this package. The node uses `output="screen"` and its
useful status output appears in the terminal, but ROS launch still creates a
small `launch.log`.

To keep those launch files out of `/root/.ros/log` inside the container, direct
them to a temporary directory before running:

```bash
mkdir -p /tmp/ros2_logs
ROS_LOG_DIR=/tmp/ros2_logs \
ros2 launch reactive_control_v2 reactive_control_v2_sim_launch.py
```

For the safe visualization-only test:

```bash
mkdir -p /tmp/ros2_logs
ROS_LOG_DIR=/tmp/ros2_logs \
ros2 launch reactive_control_v2 reactive_control_v2_sim_launch.py \
  drive_topic:=/reactive_control_v2/test_drive
```

ROS 2 launch still prints the standard log-directory line, but it now points to
`/tmp/ros2_logs`. These small files are temporary container data.

## Useful checks

```bash
ros2 topic hz /scan
ros2 topic echo /reactive_control_v2/status
ros2 topic echo /reactive_control_v2/lower_safety_status
ros2 topic echo /reactive_control_v2/local_path --once
ros2 topic echo /drive
```

To require an explicit enable signal, set `require_enable_message: true` and
publish:

```bash
ros2 topic pub --once /reactive_control_v2/enable std_msgs/msg/Bool "{data: true}"
```

## Main tuning order

1. Confirm `vehicle_width_m` from the actual car.
2. Set `lateral_safety_margin_m`.
3. Tune `forward_max_m`, `lateral_limit_m`, and the planning angle limits for
   the LiDAR field of view.
4. Tune `spatial_smoothing_*` and `temporal_smoothing_alpha`.
5. Tune pure-pursuit and speed parameters last.

The supplied values are conservative simulator starting values, not final
onboard racing values.

## Version

`0.3.0` adds hysteretic bidirectional VESC low-speed assistance, using separate
parameters for the eligible demand ceiling and forced output magnitude. Its
shortfall is `max(0, |requested speed| - |measured speed|)`. It also raises the
configured reverse attempt limit to ten, resets that budget after measured
forward progress above the configured threshold for 0.30 s, and commands every
reverse with zero steering.

`0.2.9` moves every lower-controller elapsed-time and input-freshness check to
a monotonic steady clock. ROS time remains in outgoing message headers only.
This prevents a missing, paused, or reset simulator `/clock` from freezing
dead-end/stuck confirmation, reverse/settle duration, reverse-distance
integration, attempt reset, stop duration, or diagnostic throttling. No other
node's clock or timer is changed.

`0.2.8` fixes dead-end confirmation when instantaneous FTG or emergency scan
classification flickers near a threshold. Once a healthy, stationary emergency
starts the timer, it stays latched until the vehicle moves, the sensor inputs
become unhealthy, reverse recovery is disabled, or a forward FTG route is
stably usable. Reverse entry still requires current dead-end evidence, so an
aged timer alone cannot command reverse.

`0.2.7` keeps the `0.2.6` reverse state machine and fixes simulator bring-up:
the simulator-only rear-side gate no longer prevents all reverse commands, the
startup line reports the actual odometry topic, and reverse-entry diagnostics
also run while waiting in `EMERGENCY_STOP`. The stricter onboard rear-side gate
is unchanged.

`0.2.6` adds bounded reverse recovery to the proven `0.2.5` upper/lower stack.
It uses fresh VESC-derived odometry for stuck/stationary detection, supports
persistent dead-end and forward-command/low-speed triggers, retraces the last
forward steering at low speed, requires stable FTG recovery, and inserts a
complete-stop state before forward FTG. Reverse scan/odom safety, time/distance
limits, attempt hysteresis, YAML grouping, diagnostics, and optional terminal
debugging are included. Scan-derived chassis motion remains a later extension.

`0.2.5` returns to the `0.2.3` architecture and replaces only its conservative
FTG gap/target selection. Gaps are scored using width and depth, and the target
is the centre of the deepest part of the selected gap so the fallback turns
clearly into an opening. It also uses the physical `+/-25 degree` steering
limit and adds optional FTG terminal diagnostics, disabled by default.

`0.2.3` corrects the launch selector semantics: `upper` now starts only the
upper follower, `lower` starts only the lower FTG controller, and `stack`
(the default) starts the complete upper-to-lower safety path. No controller
tuning values or C++ control logic changed.

`0.2.2` adds an explicit swept-path validation switch, validation-only entry
and recovery hysteresis, the clearer
`enable_fallback_on_upper_failure_status` lower parameter, and the
`drive_command_source` launch selector for upper-only or lower-gateway testing.
Existing tuning values are unchanged.

`0.2.0` adds the minimal lower safety controller, conservative built-in FTG,
simple forward emergency braking, recovery-ready diagnostics, and simulator
launch wiring in which only the lower controller publishes the final command.
The lower layer does not perform scan-versus-command trajectory validation and
does not reverse.

`0.1.1` adds state-change/periodic terminal diagnostics, dark-orange corridor
boundaries, temporary ROS log-directory instructions, and a simulator launch
that does not start RViz.

`0.1.2` fixes the corridor fill scale, makes logging exhaustive, and adds an
unmistakable startup version signature.

`0.1.3` explains the first failed validation sample for smoothed and raw paths,
publishes the same debug fields diagnostically, and marks the failed points in
RViz.

`0.1.5` corrects the LiDAR/`base_link` reference, adds TF-unavailable stopping
and nearest-obstacle coordinates, and renames the node to
`upper_corridor_follower`.
