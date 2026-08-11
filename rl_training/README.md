# RoboRacer RL Training Module

This package contains the current **speed-residual PPO training pipeline** for the RoboRacer / F1TENTH stack.

The supported scope is deliberately narrow:

```text
nominal raceline following
+ curvature-based rule speed
+ PPO speed residual
+ pure-pursuit steering
```

It does **not** train obstacle avoidance, fallback recovery, local trajectory planning, or drive arbitration. Those remain runtime stack responsibilities.

---

## Current design status

The current environment is a **speed-only residual RL environment**:

```text
raceline-relative state
        ↓
curvature-based rule speed
        ↓
PPO outputs one normalized residual action
        ↓
speed residual is scaled by profile max_delta_speed_mps
        ↓
final target speed is rate-limited and clipped
        ↓
pure-pursuit steering follows the raceline
        ↓
f110_gym step
```

The PPO action is one-dimensional:

```text
action = [correction_action]
correction_action ∈ [-1, 1]
raw_delta_speed_mps = max_delta_speed_mps * correction_action
```

The observation is still 8-dimensional:

```text
[
  current_speed_mps,
  rule_speed_mps,
  cross_track_error,
  heading_error,
  curv_short_abs,
  curv_mid_abs,
  curv_long_abs,
  previous_delta_speed_mps
]
```

The latest cleanup aligns the parameter names and curvature-preview style with `path_following_v2` where possible. Curvature previews can now be metre-based instead of point-count based.

---

## Important profile concept

Do not treat one model/profile as universal. The package now separates two intended speed-profile families.

### 1. `reserved_realistic`

Purpose:

```text
competition-oriented safe profile
based on path_following_v2 / onboard-style conservative parameters
intended to be mildly improved by RL residual speed
```

Use this profile when the goal is a deployable or near-deployable model.

Relevant files:

```text
configs/reserved_realistic_template.yaml
configs/reserved_realistic_train.yaml
configs/reserved_realistic_eval_fixed_start_gate.yaml
configs/reserved_realistic_analysis_same_idx0.yaml
```

The template is based on the newest path-following v2-style parameters. Edit it after onboard tests confirm the final safe baseline speeds.

### 2. `highspeed_sim_stress`

Purpose:

```text
simulator-only high-speed boundary/stress profile
useful for checking whether the RL method can learn risky speed boundaries
not intended as the onboard reserved competition profile
```

Relevant files:

```text
configs/highspeed_sim_stress.yaml
configs/highspeed_sim_stress_train.yaml
configs/highspeed_sim_stress_eval_fixed_start_gate.yaml
configs/highspeed_sim_stress_analysis_same_idx0.yaml
```

This profile keeps the current high-speed RL setup:

```text
command speed envelope: 1.0 to 10.0 m/s
rule speed range:       1.0 to 6.0 m/s
PPO residual range:     ±4.0 m/s
```

The high-speed profile is useful for algorithm development, but a high-speed-trained model should be treated as an experimental transfer candidate when tested under a reserved profile.

---

## Can a high-speed model be tested on the reserved profile?

Yes, as an experiment, but it is not the final recommended deployment path.

The PPO action itself is normalized:

```text
correction_action ∈ [-1, 1]
```

So, in principle, an inference node can use the model action as a percentage-like output and rescale it using the active profile:

```text
reserved_delta_speed_mps = correction_action * reserved_profile_max_delta_speed_mps
```

This is safer than directly reusing a high-speed physical residual. However, the observation contains physical values such as `current_speed_mps` and `rule_speed_mps`, so a model trained around a `6.0 m/s` rule-straight speed is outside its original distribution if used with a much slower reserved profile.

Recommended practice:

```text
highspeed model on reserved profile:
  OK for transfer / curiosity test

reserved model trained on reserved profile:
  recommended for serious evaluation and deployment
```

---

## Package layout

```text
rl_training/
├── README.md
├── setup.py
├── configs/
│   ├── reserved_realistic_template.yaml
│   ├── reserved_realistic_train.yaml
│   ├── reserved_realistic_eval_fixed_start_gate.yaml
│   ├── reserved_realistic_analysis_same_idx0.yaml
│   ├── highspeed_sim_stress.yaml
│   ├── highspeed_sim_stress_train.yaml
│   ├── highspeed_sim_stress_eval_fixed_start_gate.yaml
│   ├── highspeed_sim_stress_analysis_same_idx0.yaml
│   └── legacy/
├── models/
│   └── V0_reward_ppo_speed_spielberg_1000k_20260612.zip
├── rl_training/
│   ├── f110_speed_env.py
│   ├── centerline_utils.py
│   ├── pure_pursuit.py
│   ├── speed_policies.py
│   └── config_utils.py
└── scripts/
    ├── evaluate_rule_based_speed.py
    ├── evaluate_ppo_speed.py
    ├── analyze_speed_policy_comparison.py
    └── train_ppo_speed.py
```

---

## Setup

Run commands from the package root:

```bash
cd /sim_ws/src/rl_training
pip3 install -e .
```

The YAML configs assume this relative project structure:

```text
../f1tenth_gym_ros/maps/Spielberg_map.png
../centerline_tools/centerline_output/raceline_points_smooth.csv
../centerline_tools/centerline_output/corner_key_points_edited.csv
```

If paths differ, edit the `map:` section in the selected YAML config.

---

## Main commands: reserved realistic profile

### 1. Evaluate rule-based baseline

Use this before training. It checks whether the non-RL profile is valid.

```bash
python3 scripts/evaluate_rule_based_speed.py \
  --config configs/reserved_realistic_eval_fixed_start_gate.yaml
```

Expected output:

```text
lap completed
no crash
no timeout
reward breakdown
```

### 2. Train a reserved-realistic model from scratch

```bash
python3 scripts/train_ppo_speed.py \
  --config configs/reserved_realistic_train.yaml \
  --model_name ppo_reserved_realistic_v1_1000k
```

The output is written to:

```text
models/ppo_reserved_realistic_v1_1000k.zip
```

### 3. Continue training

```bash
python3 scripts/train_ppo_speed.py \
  --config configs/reserved_realistic_train.yaml \
  --load_model_path models/ppo_reserved_realistic_v1_1000k.zip \
  --total_timesteps 500000 \
  --model_name ppo_reserved_realistic_v1_1500k_continued
```

By default, the Stable-Baselines3 timestep counter is not reset when continuing training. Use `--reset_num_timesteps` only when you intentionally want a fresh counter.

### 4. Evaluate PPO model

```bash
python3 scripts/evaluate_ppo_speed.py \
  --config configs/reserved_realistic_eval_fixed_start_gate.yaml \
  --model_path models/ppo_reserved_realistic_v1_1500k_continued.zip
```

### 5. Same-start comparison analysis

```bash
python3 scripts/analyze_speed_policy_comparison.py \
  --config configs/reserved_realistic_analysis_same_idx0.yaml \
  --model_path models/ppo_reserved_realistic_v1_1500k_continued.zip \
  --out_dir analysis_reserved_realistic_idx0
```

### 6. Multi-start same-index analysis

```bash
for IDX in 0 2000 4000 6000 8000 10000; do
  python3 scripts/analyze_speed_policy_comparison.py \
    --config configs/reserved_realistic_analysis_same_idx0.yaml \
    --start_centerline_idx "$IDX" \
    --model_path models/ppo_reserved_realistic_v1_1500k_continued.zip \
    --out_dir "analysis_reserved_realistic_idx${IDX}"
done
```

Summarize outputs:

```bash
for d in analysis_reserved_realistic_idx*; do
  echo ""
  echo "===== $d ====="
  cat "$d/evaluation_summary.csv"
  echo ""
  cat "$d/reward_breakdown_summary.csv"
done
```

---

## Main commands: high-speed simulation stress profile

### 1. Evaluate high-speed rule baseline

```bash
python3 scripts/evaluate_rule_based_speed.py \
  --config configs/highspeed_sim_stress_eval_fixed_start_gate.yaml
```

### 2. Train high-speed stress model

```bash
python3 scripts/train_ppo_speed.py \
  --config configs/highspeed_sim_stress_train.yaml \
  --model_name ppo_highspeed_sim_stress_v1_1000k
```

### 3. Continue high-speed training

```bash
python3 scripts/train_ppo_speed.py \
  --config configs/highspeed_sim_stress_train.yaml \
  --load_model_path models/ppo_highspeed_sim_stress_v1_1000k.zip \
  --total_timesteps 500000 \
  --model_name ppo_highspeed_sim_stress_v1_1500k_continued
```

### 4. Evaluate high-speed PPO

```bash
python3 scripts/evaluate_ppo_speed.py \
  --config configs/highspeed_sim_stress_eval_fixed_start_gate.yaml \
  --model_path models/ppo_highspeed_sim_stress_v1_1500k_continued.zip
```

### 5. Same-start analysis

```bash
python3 scripts/analyze_speed_policy_comparison.py \
  --config configs/highspeed_sim_stress_analysis_same_idx0.yaml \
  --model_path models/ppo_highspeed_sim_stress_v1_1500k_continued.zip \
  --out_dir analysis_highspeed_sim_stress_idx0
```

---

## Using a new map and raceline

### When should you train a new model?

Train a new model when any of these changes are significant:

```text
new map geometry
new raceline CSV
new direction / reversed raceline
new rule-speed profile
new command speed envelope
new steering lookahead behavior
new reward design
```

For small path/path-name changes only, no retraining is needed. For a new competition map or substantially different raceline, retraining is strongly recommended because the curvature distribution, rule-speed profile, and optimal residual locations change.

### Minimum inputs for a new map

You need:

```text
map image used by f110_gym:
  <map_name>.png or <map_name>.pgm

map yaml if required by surrounding tools:
  <map_name>.yaml

raceline CSV:
  raceline_points_smooth.csv

optional corner key CSV for plots:
  corner_key_points_edited.csv
```

The raceline CSV must contain at least the fields used by `centerline_utils.py`, normally including:

```text
x
y
yaw
curvature or curvature-like field
```

### New-map workflow

1. Create/generate the raceline using `centerline_tools`.

Typical output:

```text
../centerline_tools/centerline_output/raceline_points_smooth.csv
../centerline_tools/centerline_output/corner_key_points_edited.csv
```

2. Copy an existing config.

For a deployable-style experiment:

```bash
cp configs/reserved_realistic_train.yaml configs/my_map_reserved_train.yaml
cp configs/reserved_realistic_eval_fixed_start_gate.yaml configs/my_map_reserved_eval.yaml
cp configs/reserved_realistic_analysis_same_idx0.yaml configs/my_map_reserved_analysis.yaml
```

For a simulator stress experiment:

```bash
cp configs/highspeed_sim_stress_train.yaml configs/my_map_highspeed_train.yaml
cp configs/highspeed_sim_stress_eval_fixed_start_gate.yaml configs/my_map_highspeed_eval.yaml
cp configs/highspeed_sim_stress_analysis_same_idx0.yaml configs/my_map_highspeed_analysis.yaml
```

3. Edit the `map:` section.

Example:

```yaml
map:
  map_path: ../f1tenth_gym_ros/maps/MyNewMap
  map_ext: .png
  centerline_csv: ../centerline_tools/centerline_output/my_new_map/raceline_points_smooth.csv
  corner_csv: ../centerline_tools/centerline_output/my_new_map/corner_key_points_edited.csv
```

4. Choose a start strategy.

For fair same-start analysis, use a raceline index:

```yaml
start:
  start_centerline_idx: 0
  random_start_along_centerline: false
```

For robust training, use random raceline starts:

```yaml
start:
  start_centerline_idx: -1
  random_start_along_centerline: true
  start_lateral_noise_std: 0.02
  start_lateral_noise_max: 0.05
  start_yaw_noise_std: 0.02
  start_yaw_noise_max: 0.05
```

5. Validate the rule-based baseline first.

```bash
python3 scripts/evaluate_rule_based_speed.py \
  --config configs/my_map_reserved_eval.yaml
```

Do not train PPO until the rule-based baseline can complete the lap safely. If the rule baseline fails, fix the map/raceline/start/speed parameters first.

6. Train.

```bash
python3 scripts/train_ppo_speed.py \
  --config configs/my_map_reserved_train.yaml \
  --model_name ppo_my_map_reserved_v1_1000k
```

7. Evaluate.

```bash
python3 scripts/evaluate_ppo_speed.py \
  --config configs/my_map_reserved_eval.yaml \
  --model_path models/ppo_my_map_reserved_v1_1000k.zip
```

8. Run same-start analysis.

```bash
python3 scripts/analyze_speed_policy_comparison.py \
  --config configs/my_map_reserved_analysis.yaml \
  --start_centerline_idx 0 \
  --model_path models/ppo_my_map_reserved_v1_1000k.zip \
  --out_dir analysis_my_map_reserved_idx0
```

9. Run multi-start analysis.

Choose several valid raceline indices spread across the new map:

```bash
for IDX in 0 2000 4000 6000 8000 10000; do
  python3 scripts/analyze_speed_policy_comparison.py \
    --config configs/my_map_reserved_analysis.yaml \
    --start_centerline_idx "$IDX" \
    --model_path models/ppo_my_map_reserved_v1_1000k.zip \
    --out_dir "analysis_my_map_reserved_idx${IDX}"
done
```

Adjust the index list if the new raceline has fewer points.

---

## Important parameters to tune in YAML

### Map and raceline

```yaml
map:
  map_path: ../f1tenth_gym_ros/maps/Spielberg_map
  map_ext: .png
  centerline_csv: ../centerline_tools/centerline_output/raceline_points_smooth.csv
  corner_csv: ../centerline_tools/centerline_output/corner_key_points_edited.csv
```

### Speed profile

Path-following-style names are preferred:

```yaml
speed:
  command_speed_min_mps: 1.0
  command_speed_max_mps: 10.0
  rule_curve_min_speed_mps: 1.0
  rule_straight_speed_mps: 6.0
  rule_speed_curvature_gain: 2.0
  rule_speed_curvature_preview_m: 0.50
  max_delta_speed_mps: 4.0
  max_speed_delta_per_step_mps: 0.2
```

Meaning:

```text
command_speed_min/max_mps:
  final physical command clamp

rule_curve_min_speed_mps:
  slowest curvature-rule speed

rule_straight_speed_mps:
  fastest curvature-rule speed on straight sections

max_delta_speed_mps:
  physical residual scale for PPO action
```

### Curvature previews

```yaml
curvature:
  model_curvature_short_preview_m: 0.30
  model_curvature_mid_preview_m: 1.20
  model_curvature_long_preview_m: 2.40
```

The observation remains `curv_short`, `curv_mid`, `curv_long`; only their preview distances are metre-based.

### Reward

```yaml
reward:
  target_lap_steps: 6350
  target_speed_smoothness_weight: 0.04
  reward_curvature_section_start_m: 0.10
  reward_curvature_section_end_m: 2.00
  curvature_speed_section_weight: 0.006
  residual_smoothness_weight: 0.08
  residual_free_band_mps: 0.8
  residual_excess_weight: 0.05
```

### Steering compatibility with path_following_v2

```yaml
steering:
  wheelbase_m: 0.33
  steering_max_deg: 20.6
  min_forward_point_x_m: 0.05
  use_speed_dependent_steering_lookahead: true
  steering_min_lookahead_m: 0.6
  steering_max_lookahead_m: 1.6
  steering_lookahead_speed_gain: 0.25
```

`min_forward_point_x_m` filters pure-pursuit target candidates so the selected target is actually in front of the car in the vehicle frame.

### Gate

The RL gate is optional.

For normal raceline-start training:

```yaml
gate:
  enable_rl_gate: false
```

For fixed off-raceline diagnostics, such as starting from `(0,0)`:

```yaml
gate:
  enable_rl_gate: true
```

In the real ROS2 stack, the higher-level racing mode / drive arbitration can serve as the main gate by only running RL inference in raceline-following mode.

---

## Reward design summary

The current reward is:

```text
reward =
    progress_reward
  + lap_reward
  + early_finish_bonus
  - tracking_penalty
  - curvature_speed_section_penalty
  - target_speed_smoothness_penalty
  - residual_smoothness_penalty
  - residual_excess_penalty
  - time_penalty
  - crash_penalty
  - timeout_penalty
```

The evaluation scripts print a reward breakdown so tuning can be based on actual term contributions instead of only the final return.

---

## Outputs from analysis

`analyze_speed_policy_comparison.py` writes files such as:

```text
evaluation_summary.csv
reward_breakdown_summary.csv
rule_timeseries.csv
model_timeseries.csv
speed_vs_cumulative_progress.png
residual_vs_cumulative_progress.png
tracking_errors_vs_cumulative_progress.png
xy_speed_trace.png
corner_entrance_apex_exit_speeds.png
```

Use these to judge:

```text
lap completion
step count improvement
tracking penalty increase
curvature-speed penalty increase
residual smoothness/excess penalties
whether speed gain is smooth or extreme
```

---

## Practical acceptance checklist

Before treating a model as useful:

```text
1. Rule-based baseline completes the lap safely.
2. PPO completes the lap safely.
3. PPO is faster than rule-based on same-start analysis.
4. PPO has no crash / timeout / bad tracking.
5. PPO residual is not saturated everywhere.
6. Tracking penalty increase is acceptable.
7. Multi-start analysis is stable across the map.
8. For deployable reserved profile, the trained profile matches the intended runtime speed envelope.
```
