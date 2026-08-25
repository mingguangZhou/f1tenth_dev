# oudtra_driver_bringup

Master launch package for the integrated raceline-to-Reactive V2 stack.

With the atomic development commands, start/enter Docker from the repository
root and launch the stack inside the container:

```bash
./dk.sh start
f1 build auto
f1 auto
```

`dk.sh start` performs no dependency installation or network access;
`f1 build auto` only compiles. Run `./dk.sh deps` explicitly after adding a new
system dependency to a package manifest.

Run `f1 sim` in a separate container terminal for the IFAC Roboracer simulator
with three fixed obstacles and one moving traffic car. Use
`f1 sim --no-agents` to remove the moving car while retaining the fixed
obstacles, or `f1 sim --no-obstacle` to run the clean map with neither.

In simulation, `f1 auto` follows the optimized IFAC Roboracer racing line at
`/sim_ws/src/centerline_tools/output_backup/ifac_roboracer/`
`raceline_points_optimized.csv`. The retained Spielberg line remains available
as an explicit override for a regression run:

```bash
f1 auto \
  raceline_csv_path:=/sim_ws/src/centerline_tools/output_backup/V0_reward_ppo_speed_spielberg_1000k_20260612/raceline_points_smooth.csv
```

This changes both the driven path and the local planner's Frenet reference. The
published racing line is the single reference, with its nominal/rejoin target at
`d=0`; the planner does not load a separate centerline CSV.

Onboard:

```bash
ros2 launch oudtra_driver_bringup full_stack_onboard_launch.py
```

The onboard wrapper reads the fixed racing line at
`/f1tenth_ws/src/f1tenth_dev/centerline_tools/output_backup/ifac_roboracer/`
`raceline_points_optimized.csv`. The centerline and offline optimization tools
are not runtime planning inputs. `centerline_tools` only needs to remain
installed for its `raceline_publisher`; replacing the fixed CSV and restarting
the stack does not require rebuilding that package.

Routine code deployment copies and rebuilds these four packages:

```text
path_following_v2
reactive_control_v2
drive_arbitration_v2
oudtra_driver_bringup
```

After copying them under `/f1tenth_ws/src/f1tenth_dev`, rebuild from
`/f1tenth_ws` and source `/f1tenth_ws/install/setup.bash` before launching.

```bash
cd /f1tenth_ws
source /opt/ros/foxy/setup.bash
test -r /f1tenth_ws/src/f1tenth_dev/centerline_tools/output_backup/ifac_roboracer/raceline_points_optimized.csv
colcon build --packages-select \
  path_following_v2 reactive_control_v2 \
  drive_arbitration_v2 oudtra_driver_bringup
source /f1tenth_ws/install/setup.bash
```

Simulator (no particle filter and no `/pf/health` requirement):

```bash
ros2 launch oudtra_driver_bringup full_stack_sim_launch.py
```

Both compatibility launches include the same `full_stack_launch.py`. The
non-`_sim` component YAML files are the canonical production behavior profile.
The `_sim.yaml` files contain only simulated clock, frame, odometry, particle-
filter availability, and simulator-only recovery gates; do not put speed,
planning, geometry, or smoothing tuning in those adapters. Parameter order is:

```text
production behavior -> platform adapter -> integrated safety -> platform safety
```

Supplying `raceline_csv_path:=...` remains supported for experiments. Platform
wrappers deliberately own the `/sim_ws` versus `/f1tenth_ws` path difference;
the shared launch contains no workspace-specific path.

Both launches preserve the direction stored in the racing-line CSV by default
(`raceline_direction:=csv`). The racing-line publisher applies the selected
direction once, and both the local path generator and planner consume that same
published geometry. There is no runtime `centerline_direction` to synchronize.
To traverse the selected racing line in reverse:

```bash
ros2 launch oudtra_driver_bringup full_stack_sim_launch.py \
  raceline_direction:=reverse
```

The launches start the raceline publisher, raw local-raceline generator,
persistent local trajectory planner, path follower, Reactive V2 upper
controller, final-trajectory guard, drive arbitrator, and Reactive V2 lower
safety controller. `local_trajectory_planner` publishes the existing
`/path_following_v2/local_path`, so the follower and guard always consume the
same final raceline, obstacle-avoidance, or raceline-recovery trajectory.

Onboard, `start_particle_filter` remains `false` by default. Start and inspect
the particle filter separately first, then run the master launch as shown above:

```bash
ros2 launch particle_filter localize_launch.py
ros2 launch oudtra_driver_bringup full_stack_onboard_launch.py
```

For the optional old single-command behavior, use
`start_particle_filter:=true` on the onboard master launch.

By default the shared terminal shows the local-planner startup summary plus
drive-arbitrator and lower-safety state transitions. The path
generator/follower, Reactive upper, and guard show only warnings and errors.
Enable focused detail with a launch override:

```bash
ros2 launch oudtra_driver_bringup full_stack_sim_launch.py \
  path_follower_log_level:=debug
```

Available per-node arguments are `path_generator_log_level`,
`local_trajectory_planner_log_level`, `path_follower_log_level`,
`reactive_upper_log_level`,
`raceline_guard_log_level`, `drive_arbitrator_log_level`, and
`lower_safety_log_level`.

Useful integration checks:

```bash
ros2 topic echo /path_following_v2/path_status
ros2 topic echo /path_following_v2/trajectory_speed_cap_mps
ros2 topic echo /drive_arbitration_v2/selected_mode
ros2 topic info /drive --verbose
```

The actually authorized local trajectory marker is dark blue during ordinary
raceline tracking, light blue while `AVOIDANCE_DEPARTING`,
`AVOIDANCE_PASSING`, `AVOIDANCE_RETURNING`, `RECOVERING_TO_RACELINE`, or
`REPLAN_PENDING` is being executed, and red in Reactive mode. Inspect
`plan_id` in `/path_following_v2/path_status`: it should
remain unchanged while one map-anchored plan is trimmed and executed, and only
change after a material replan request.

`/drive` must still have only one publisher:
`reactive_control_v2/lower_safety_controller`.

The optional RL speed-inference package is not launched by this stack. The
canonical production profile remains in rule-only mode; RL configuration files
and executables are outside this migration.
