# path_following_v2

Clean RoboRacer/F1TENTH raceline-following baseline for ROS 2 Foxy.

This package intentionally keeps the controller side simple:

```text
/raceline_waypoints
        ↓
path_generator
        ├── /path_following_v2/local_path
        └── /path_following_v2/rule_speed_index

/path_following_v2/local_path
/path_following_v2/rule_speed_index
/rl_speed_inference/speed_residual_mps   optional
        ↓
path_following_v2
        ↓
/drive
```

## Nodes

### `path_generator_node`

Subscribes to the global raceline from `centerline_tools`:

```text
/raceline_waypoints
```

Expected waypoint row format:

```text
[index, x, y, yaw, curvature, curvature_abs]
```

Publishes:

```text
/path_following_v2/local_path
/path_following_v2/rule_speed_index
```

The local path is a short forward segment of the global raceline. This keeps the controller ready for future obstacle-avoidance insertion.

The rule speed is aligned with the RL training environment:

```text
rule_curvature = max(curvature_abs[nearest_idx ... nearest_idx + rule_speed_curvature_lookahead_points])
rule_speed_mps = clip(rule_max_speed_mps / (1 + rule_speed_curvature_gain * rule_curvature),
                      rule_min_speed_mps,
                      rule_max_speed_mps)
rule_speed_index = (rule_speed_mps - speed_min) / (speed_max - speed_min)
```

Default simulator values:

```yaml
speed_min: 1.0
speed_max: 10.0
rule_min_speed_mps: 1.0
rule_max_speed_mps: 6.0
rule_speed_curvature_gain: 2.0
rule_speed_curvature_lookahead_points: 3
```

### `path_following_v2_node`

Subscribes to:

```text
/path_following_v2/local_path
/path_following_v2/rule_speed_index
/rl_speed_inference/speed_residual_mps
```

Publishes:

```text
/drive
```

Speed modes:

```text
speed_mode = 0: rule-based speed only
speed_mode = 1: rule-based speed + fresh RL residual; if residual is absent/stale, use rule-based speed only
```

Final speed handling:

```text
rule_speed_mps = speed_min + rule_speed_index * (speed_max - speed_min)
final_speed_mps = clamp(rule_speed_mps + residual_mps, speed_min, speed_max)
final_speed_mps = rate_limit(final_speed_mps, max_speed_delta_per_step_mps)
```

The pure-pursuit steering lookahead is independent from the rule-speed curvature preview:

```yaml
fixed_steering_lookahead_m: 0.60
use_speed_dependent_steering_lookahead: true
steering_min_lookahead_m: 0.6
steering_max_lookahead_m: 1.6
steering_lookahead_speed_gain: 0.25
```

## Usage

Build:

```bash
cd /sim_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select path_following_v2
source install/setup.bash
```

Run simulator configuration:

```bash
ros2 launch path_following_v2 path_following_v2_sim_launch.py
```

For rule-based speed only, keep:

```yaml
speed_mode: 0
```

For RL-boosted residual speed, set:

```yaml
speed_mode: 1
```

Then launch `rl_speed_inference` separately. Before inference publishes a fresh residual, the car uses rule-based speed only. If the inference node is killed or the residual topic becomes stale, the follower falls back to rule speed only. Residual/path/rule freshness is checked with steady wall-clock time, not ROS sim time, so the watchdog is not blocked by `/clock` behavior.

## Key topics

```bash
ros2 topic echo /path_following_v2/local_path --once
ros2 topic echo /path_following_v2/rule_speed_index --once
ros2 topic echo /rl_speed_inference/speed_residual_mps --once
ros2 topic echo /drive --once
```
