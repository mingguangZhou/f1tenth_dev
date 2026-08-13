# rl_speed_inference

ROS 2 Foxy node that runs a trained PPO speed policy and publishes a speed residual:

```text
/rl_speed_inference/speed_residual_mps   std_msgs/Float64
```

`path_following_v2` remains responsible for composing the final speed, checking residual freshness, and falling back to the rule-based speed when the residual is unavailable.

## Two supported model interfaces

### Recommended: `speed_ratio`

Use this for the newest ratio-assist models trained from `rl_training/configs/ratio_assist/V1_reward_ratio_spielberg_master.yaml`.

Observation expected by the model:

```text
[current_speed_ratio, rule_speed_ratio, cross_track_error_m, heading_error_rad,
 curv_short_abs, curv_mid_abs, curv_long_abs, previous_assist_ratio]
```

Action conversion:

```text
if action >= 0:
  assist_ratio = action * positive_assist_ratio * assist_gain
else:
  assist_ratio = action * negative_assist_ratio * assist_gain

residual_mps = assist_ratio * rule_speed_mps
```

`assist_gain` is the practical safety/faster knob:

```text
0.10  very cautious first onboard test
0.25  cautious
0.50  moderate, validate in sim before any onboard use
1.00  full learned trend, sim only unless extensively validated
```

### Legacy: `physical_mps`

Use only for old physical residual models.

```text
[current_speed_mps, rule_speed_mps, cross_track_error_m, heading_error_rad,
 curv_short_abs, curv_mid_abs, curv_long_abs, previous_delta_speed_mps]

delta_speed_mps = action * max_delta_speed_mps
```

A legacy example is kept at:

```text
config/legacy_physical_mps.yaml
```

## Config files

```text
config/rl_speed_inference.yaml      onboard/reserved ratio-assist template
config/rl_speed_inference_sim.yaml  highspeed simulator ratio-assist template
config/legacy_physical_mps.yaml     old physical-residual model template
```

The most important onboard parameters are:

```yaml
residual_output_mode: "speed_ratio"
model_path: "/absolute/path/to/model.zip"
centerline_csv: "/absolute/path/to/raceline_points_smooth.csv"
odom_topic: "/pf/pose/odom"
assist_gain: 0.10
command_speed_min_mps: 0.5
command_speed_max_mps: 2.0
rule_curve_min_speed_mps: 0.5
rule_straight_speed_mps: 1.5
rule_speed_curvature_preview_m: 0.50
```

## Copy model to onboard

From the dev laptop:

```bash
scp /path/to/V1_reward_ratio_ppo_speed_spielberg_500k_20260812.zip \
  <jetson_user>@<jetson_ip>:~/f1tenth_dev/rl_training/models/
```

Copy the exact raceline CSV used by the path follower:

```bash
scp /path/to/raceline_points_smooth.csv \
  <jetson_user>@<jetson_ip>:~/f1tenth_dev/centerline_tools/centerline_output/
```

Then edit `config/rl_speed_inference.yaml` to point to those absolute onboard paths.

## Build and run onboard

```bash
cd ~/f1tenth_dev
source /opt/ros/foxy/setup.bash
colcon build --packages-select rl_speed_inference
source install/setup.bash

ros2 launch rl_speed_inference rl_speed_inference_launch.py \
  params_file:=/absolute/path/to/rl_speed_inference.yaml
```

Before driving, check the input and output topics:

```bash
ros2 topic hz /pf/pose/odom
ros2 topic echo /rl_speed_inference/speed_residual_mps
```

Start with `assist_gain: 0.10`. Increase only after simulator and static topic checks look correct.
