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

- dark blue: the raceline local path while arbitration is `RACELINE` and the lower
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
PF invalid/unavailable, and raceline unavailable. Return now requires one
continuous 0.5-second interval in which both conditions are true:

1. The complete raceline chain is healthy and the raceline guard is clear.
2. Fresh lower-controller status confirms `NOMINAL` while arbitration is still
   `REACTIVE` (mode code `2`).

Thus, the timer is reset while the lower layer reports `FALLBACK_FTG`,
`EMERGENCY_STOP`, `REVERSE_RECOVERY`, or `RECOVERY_SETTLE`. Reverse recovery is
not cancelled merely because the raceline guard becomes clear.

There is also a handover-race safety net. If arbitration has selected
`RACELINE`, but fresh lower status confirms `EMERGENCY_STOP` for raceline mode,
the arbitrator records a `RACELINE_BLOCKED` trigger and relatches `REACTIVE`.
The lower controller continues to hold the emergency stop, while its existing
dead-end timer, FTG, and reverse recovery become authorized again.

Configuration:

```yaml
allow_auto_recovery_from_blocked: true
allow_auto_recovery_from_pf_invalid: true
allow_auto_recovery_from_raceline_unavailable: true
raceline_recovery_stable_sec: 0.5
enable_lower_safety_recovery_coordination: true
lower_status_timeout_sec: 0.30
```

`enable_lower_safety_recovery_coordination: false` restores the previous
raceline-return behavior. This update does not change reverse-entry thresholds,
reverse-exit conditions, FTG calculations, or the final `/drive` safety checks.

Manual reset of the Reactive latch remains available:

```bash
ros2 topic pub --once /drive_arbitration_v2/reset std_msgs/msg/Bool "{data: true}"
```
