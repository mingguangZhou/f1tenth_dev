# rl_speed_inference

ROS 2 Foxy runtime wrapper for the trained PPO residual speed model.

This package intentionally publishes only the **RL residual speed**. It does not publish the final speed command and does not implement fallback arbitration. The path follower combines:

```text
final_speed = rule_speed + fresh_rl_residual
```

or uses rule speed only if the residual is absent/stale.

## Runtime flow

```text
/ego_racecar/odom
raceline CSV
        ↓
ppo_speed_node
        ↓
/rl_speed_inference/speed_residual_mps
```

The path follower consumes this residual together with `/path_following_v2/rule_speed_index`.

## Observation and action alignment with training

The node builds the same 8-feature observation used by the current speed-training environment:

```text
[
  current_speed_mps,
  rule_speed_mps,
  cross_track_error,
  heading_error,
  curv_short_abs,
  curv_mid_abs,
  curv_long_abs,
  previous_delta_speed_mps,
]
```

The PPO action is interpreted as a residual:

```text
action ∈ [-1, 1]
raw_delta_speed_mps = max_delta_speed_mps * action
published_residual_mps = gate_scale * raw_delta_speed_mps
```

The residual gate matches the training environment operating-domain gate. It fades the residual toward zero when tracking error is outside the trained/reasonable range.

## Speed range setup

Simulator defaults:

```yaml
speed_min: 1.0
speed_max: 10.0
rule_min_speed_mps: 1.0
rule_max_speed_mps: 6.0
max_delta_speed_mps: 4.0
```

Meaning:

```text
speed_min/speed_max:
  final physical command envelope used by the path follower

rule_min_speed_mps/rule_max_speed_mps:
  rule-based baseline speed envelope, used to build the observation

max_delta_speed_mps:
  maximum absolute residual before gate scaling
```

The final command rate limit is handled in `path_following_v2`, not here.

## Usage

Build:

```bash
cd /sim_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select rl_speed_inference
source install/setup.bash
```

Run:

```bash
ros2 launch rl_speed_inference rl_speed_inference_sim_launch.py
```

Check:

```bash
ros2 topic echo /rl_speed_inference/speed_residual_mps --once
```

Make sure the configured model and raceline paths exist:

```yaml
model_path: "/sim_ws/src/rl_training/models/V0_reward_ppo_speed_spielberg_1000k_20260612.zip"
centerline_csv: "/sim_ws/src/centerline_tools/centerline_output/raceline_points_smooth.csv"
```
