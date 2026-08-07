# oudtra_driver_bringup

Master launch package for the integrated raceline-to-Reactive V2 stack.

Onboard:

```bash
ros2 launch oudtra_driver_bringup full_stack_onboard_launch.py
```

Simulator (no particle filter and no `/pf/health` requirement):

```bash
ros2 launch oudtra_driver_bringup full_stack_sim_launch.py
```

Both launches preserve the direction stored in the CSV by default
(`raceline_direction:=csv`). To traverse the selected raceline in reverse:

```bash
ros2 launch oudtra_driver_bringup full_stack_sim_launch.py \
  raceline_direction:=reverse
```

The launches start the raceline publisher, path generator/follower, Reactive V2
upper controller, raceline guard, drive arbitrator, and Reactive V2 lower safety
controller. The onboard launch additionally starts `particle_filter` by default.

By default the shared terminal shows the drive-arbitrator and lower-safety
state transitions, while the path generator/follower, Reactive upper, and guard
show only warnings and errors. Enable focused detail with a launch override:

```bash
ros2 launch oudtra_driver_bringup full_stack_sim_launch.py \
  path_follower_log_level:=debug
```

Available per-node arguments are `path_generator_log_level`,
`path_follower_log_level`, `reactive_upper_log_level`,
`raceline_guard_log_level`, `drive_arbitrator_log_level`, and
`lower_safety_log_level`.

The optional RL speed-inference package was not present in this integration
archive, so it is not guessed or launched here. It can continue to run
separately and publish `/rl_speed_inference/speed_residual_mps`; the existing
path follower retains its current rule-only or rule-plus-residual configuration.
