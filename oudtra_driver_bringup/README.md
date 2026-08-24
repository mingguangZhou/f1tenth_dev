# oudtra_driver_bringup

Master launch package for the integrated raceline-to-Reactive V2 stack.

With the atomic development commands, start/enter Docker from the repository
root and launch the stack inside the container:

```bash
./dk.sh start
f1 build auto
f1 auto
```

`dk.sh start` installs the managed ROS dependencies once per container and
rechecks them when a package manifest changes; `f1 build auto` only compiles.

Run `f1 sim` in a separate container terminal for the IFAC Roboracer simulator
with three fixed obstacles and one moving traffic car. Use
`f1 sim --no-agents` to remove the moving car.

In simulation, `f1 auto` follows the validated optimized IFAC Roboracer
raceline by default. The retained Spielberg line remains available for an
explicit regression run:

```bash
f1 auto \
  raceline_csv_path:=/sim_ws/src/centerline_tools/output_backup/V0_reward_ppo_speed_spielberg_1000k_20260612/raceline_points_smooth.csv
```

This changes only the driven raceline. The smooth centerline remains the local
planner's Frenet reference.

Onboard:

```bash
ros2 launch oudtra_driver_bringup full_stack_onboard_launch.py
```

Simulator (no particle filter and no `/pf/health` requirement):

```bash
ros2 launch oudtra_driver_bringup full_stack_sim_launch.py
```

Both launches preserve the direction stored in the CSV by default
(`raceline_direction:=csv`). The centerline uses
`centerline_direction:=auto`, which matches its local direction to the
published raceline even when the two generated CSV files have opposite point
order. To traverse the selected raceline in reverse:

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

The optional RL speed-inference package was not present in this integration
archive, so it is not guessed or launched here. It can continue to run
separately and publish `/rl_speed_inference/speed_residual_mps`; the existing
path follower retains its current rule-only or rule-plus-residual configuration.
