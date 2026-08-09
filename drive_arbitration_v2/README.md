# drive_arbitration_v2

ROS 2 Foxy supervisory package selecting between the localization-dependent
raceline follower and localization-free Reactive Control V2.

The package contains:

- `raceline_guard_node`: checks LaserScan endpoints against a parameterized band
  around the upcoming local raceline.
- `drive_arbitrator_node`: owns `WAITING`, `RACELINE`, `REACTIVE`, and `STOP`
  modes and publishes the selected candidate command.

Only `reactive_control_v2/lower_safety_controller` should publish the final
`/drive` topic. See `DESIGN.md` for interfaces and exact trigger definitions.

## Ultimate chosen local trajectory (visualization only)

The arbitrator publishes a single RViz `visualization_msgs/msg/Marker` on:

```text
/drive_arbitration_v2/ultimate_chosen_local_trajectory
```

- green: the raceline local path while arbitration is `RACELINE` and the lower
  controller confirms `NOMINAL` for that mode;
- orange: the upper corridor path while arbitration is `REACTIVE` and the lower
  controller confirms `NOMINAL` for that mode;
- hidden: lower FTG, reverse, emergency stop, recovery settle, waiting, stop,
  stale status, or stale/invalid path.

This publisher does not modify candidate commands, arbitration, or `/drive`.

Standalone launches:

```bash
ros2 launch drive_arbitration_v2 drive_arbitration_v2.launch.py
ros2 launch drive_arbitration_v2 drive_arbitration_v2_sim.launch.py
```

The supplied sim and onboard YAML files enable automatic return from blockage,
PF invalid/unavailable, and raceline unavailable after the complete raceline
chain remains healthy and clear for 0.5 seconds. Trigger-specific recovery can
be enabled independently with:

```yaml
allow_auto_recovery_from_blocked: true
allow_auto_recovery_from_pf_invalid: true
allow_auto_recovery_from_raceline_unavailable: true
raceline_recovery_stable_sec: 0.5
```

Manual reset of the Reactive latch remains available:

```bash
ros2 topic pub --once /drive_arbitration_v2/reset std_msgs/msg/Bool "{data: true}"
```
