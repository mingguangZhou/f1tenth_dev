# reactive_control_v2

Package version: `0.1.5`

`reactive_control_v2` is a compact, forward-only ROS 2 Foxy controller for
driving without a global map, localization result, or raceline reference.

The `upper_corridor_follower` node transforms the current LiDAR scan into the
configured `base_frame`, converts it into connected free-space corridor
branches, selects a stable branch, generates a smooth local center path, and
uses pure pursuit to publish a nominal Ackermann command.

This first version deliberately contains no reverse recovery, special 0.5 m
gap mode, PF switching, or independent emergency-braking layer.

## I/O

### Inputs

| Topic | Type | Required | Purpose |
|---|---|---:|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | Yes | Local free-space geometry |
| `/ego_racecar/odom` | `nav_msgs/msg/Odometry` | No by default | Speed reporting and optional freshness gate |
| `/reactive_control_v2/enable` | `std_msgs/msg/Bool` | No by default | Reserved external mode-enable input |

### Outputs

| Topic | Type | Purpose |
|---|---|---|
| `/reactive_control_v2/nominal_cmd` | `ackermann_msgs/msg/AckermannDriveStamped` | Nominal forward command; remapped to `/drive` by the simulator launch |
| `/reactive_control_v2/local_path` | `nav_msgs/msg/Path` | Smoothed selected corridor center path |
| `/reactive_control_v2/markers` | `visualization_msgs/msg/MarkerArray` | Corridor fill/edges, lookahead point, and steering arrow |
| `/reactive_control_v2/status` | `diagnostic_msgs/msg/DiagnosticArray` | State, stop reason, corridor reach/width, and current command |

Status names are `DRIVING`, `BLOCKED`, `WAITING_FOR_SCAN`, `TF_UNAVAILABLE`,
`INPUT_INVALID`, `ODOM_STALE`, and `DISABLED`. Every non-driving state
publishes zero speed.

By default, the terminal prints one concise warning only when the planner
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
7. Select the first path point beyond `lookahead_distance_m` and apply the same
   pure-pursuit relationship as the previous path-following controller:

   ```text
   curvature = 2 * target_y / target_distance^2
   steering  = atan(wheelbase * curvature)
   ```

8. Interpolate speed from `velocity_max_mps` at zero steering to
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

The correct executable starts with:

```text
reactive_control_v2 v0.1.5 upper_corridor_follower ready: ... full_terminal_debug=false
```

With `full_terminal_debug: true`, it also confirms receipt of the first
LaserScan and prints the first planning result. If the startup line does not contain `v0.1.5`, the shell is still
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

The launch remaps the nominal command to `/drive`. To inspect the nominal
command without controlling the car:

```bash
ros2 launch reactive_control_v2 reactive_control_v2_sim_launch.py \
  drive_topic:=/reactive_control_v2/test_drive
```

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
