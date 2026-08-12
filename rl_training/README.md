# RoboRacer RL Training Module

This package contains the speed-only PPO training and evaluation pipeline for nominal raceline following.

The scope is intentionally narrow:

```text
raceline path
+ curvature-based rule speed
+ PPO speed assist/residual
+ pure-pursuit steering
+ f110_gym simulation
```

It does **not** train obstacle avoidance, local trajectory planning, fallback recovery, or drive arbitration. Those remain responsibilities of the ROS2 racing stack.

---

## Current recommended workflow: ratio assist

The recommended competition-oriented flow is now:

```text
1. Train the model in the highspeed_sim_stress_ratio profile.
   This profile is simulator-only and intentionally allows risky speeds, so PPO can learn from mistakes and boundary behavior.

2. Apply the trained model to the reserved_realistic_ratio profile.
   This profile uses the safer onboard-style speed envelope. The same learned action is converted into a smaller physical speed residual.

3. Tune assist_gain on the reserved profile.
   assist_gain is the manual safer/faster knob. It lets the learned trend stay the same while the physical boost is scaled.
```

This matches the intended competition use: the model learns a **trend/ratio**, not a hard-coded absolute m/s speed number.

---

## Two residual modes

The environment supports two modes.

### 1. `speed_ratio` mode — recommended new path

YAML location:

```text
configs/ratio_assist/
```

The PPO action is dimensionless:

```text
action ∈ [-1, 1]
```

The environment converts it to a profile-relative speed assist:

```text
if action >= 0:
    assist_ratio = action * positive_assist_ratio * assist_gain
else:
    assist_ratio = action * negative_assist_ratio * assist_gain

delta_speed_mps = assist_ratio * rule_speed_mps
target_speed_mps = rule_speed_mps + delta_speed_mps
```

So the same policy can be trained with a high-speed profile and then applied to a reserved profile with a different speed scale.

In `speed_ratio` mode, the observation remains 8-dimensional, but speed-related entries are normalized:

```text
[
  current_speed_ratio,
  rule_speed_ratio,
  cross_track_error_m,
  heading_error_rad,
  curv_short_abs,
  curv_mid_abs,
  curv_long_abs,
  previous_assist_ratio,
]
```

### 2. `physical_mps` mode — legacy path

YAML location:

```text
configs/legacy_physical_mps/
```

The old interpretation is preserved:

```text
delta_speed_mps = action * max_delta_speed_mps
```

Use this mode to reproduce previous experiments or evaluate old saved physical-residual models. New highspeed-train → reserved-apply experiments should use `speed_ratio` mode.

---

## Package layout

```text
rl_training/
├── configs/
│   ├── ratio_assist/
│   │   ├── highspeed_sim_stress_ratio_train.yaml
│   │   ├── highspeed_sim_stress_ratio_eval_fixed_start_gate.yaml
│   │   ├── highspeed_sim_stress_ratio_analysis_same_idx0.yaml
│   │   ├── reserved_realistic_ratio_template.yaml
│   │   ├── reserved_realistic_ratio_eval_fixed_start_gate.yaml
│   │   ├── reserved_realistic_ratio_analysis_same_idx0.yaml
│   │   └── reserved_realistic_ratio_train_optional.yaml
│   └── legacy_physical_mps/
│       └── old physical-residual configs
├── scripts/
│   ├── train_ppo_speed.py
│   ├── evaluate_rule_based_speed.py
│   ├── evaluate_ppo_speed.py
│   ├── analyze_speed_policy_comparison.py
│   ├── ratio_assist/
│   │   └── short shell wrappers
│   └── legacy_physical_mps/
│       └── legacy usage note
└── rl_training/
    ├── f110_speed_env.py
    ├── centerline_utils.py
    ├── pure_pursuit.py
    ├── speed_policies.py
    └── config_utils.py
```

---

## Setup

Run from the package root:

```bash
cd /sim_ws/src/rl_training
pip3 install -e .
```

The provided configs assume these relative paths:

```text
../f1tenth_gym_ros/maps/Spielberg_map.png
../centerline_tools/centerline_output/raceline_points_smooth.csv
../centerline_tools/centerline_output/corner_key_points_edited.csv
```

Edit the `map:` block in the selected YAML file when using a different map/raceline.

---

## Ratio-assist main commands

### 1. Evaluate highspeed rule-based baseline

Before training, check that the rule-based profile completes:

```bash
python3 scripts/evaluate_rule_based_speed.py \
  --config configs/ratio_assist/highspeed_sim_stress_ratio_eval_fixed_start_gate.yaml
```

### 2. Train the highspeed ratio model

```bash
python3 scripts/train_ppo_speed.py \
  --config configs/ratio_assist/highspeed_sim_stress_ratio_train.yaml \
  --model_name ppo_highspeed_ratio_trend_v1_1000k
```

Equivalent wrapper:

```bash
bash scripts/ratio_assist/train_highspeed_ratio.sh \
  --model_name ppo_highspeed_ratio_trend_v1_1000k
```

### 3. Continue training

```bash
python3 scripts/train_ppo_speed.py \
  --config configs/ratio_assist/highspeed_sim_stress_ratio_train.yaml \
  --load_model_path models/ppo_highspeed_ratio_trend_v1_1000k.zip \
  --total_timesteps 500000 \
  --model_name ppo_highspeed_ratio_trend_v1_1500k_continued
```

By default the Stable-Baselines3 timestep counter is not reset when continuing training. Use `--reset_num_timesteps` only when you intentionally want a fresh counter.

### 4. Evaluate the model in the highspeed profile

```bash
python3 scripts/evaluate_ppo_speed.py \
  --config configs/ratio_assist/highspeed_sim_stress_ratio_eval_fixed_start_gate.yaml \
  --model_path models/ppo_highspeed_ratio_trend_v1_1000k.zip
```

### 5. Apply the same model to the reserved profile

Start conservatively:

```bash
python3 scripts/evaluate_ppo_speed.py \
  --config configs/ratio_assist/reserved_realistic_ratio_eval_fixed_start_gate.yaml \
  --model_path models/ppo_highspeed_ratio_trend_v1_1000k.zip \
  --assist_gain 0.25
```

Try stronger assistance in simulation:

```bash
python3 scripts/evaluate_ppo_speed.py \
  --config configs/ratio_assist/reserved_realistic_ratio_eval_fixed_start_gate.yaml \
  --model_path models/ppo_highspeed_ratio_trend_v1_1000k.zip \
  --assist_gain 0.50

python3 scripts/evaluate_ppo_speed.py \
  --config configs/ratio_assist/reserved_realistic_ratio_eval_fixed_start_gate.yaml \
  --model_path models/ppo_highspeed_ratio_trend_v1_1000k.zip \
  --assist_gain 0.75
```

### 6. Same-start comparison on reserved profile

This compares rule-based vs PPO from the exact same raceline CSV row:

```bash
python3 scripts/analyze_speed_policy_comparison.py \
  --config configs/ratio_assist/reserved_realistic_ratio_analysis_same_idx0.yaml \
  --start_centerline_idx 0 \
  --model_path models/ppo_highspeed_ratio_trend_v1_1000k.zip \
  --assist_gain 0.50 \
  --out_dir analysis_reserved_ratio_idx0_gain050
```

### 7. Multi-start reserved-profile check

```bash
for IDX in 0 2000 4000 6000 8000 10000; do
  python3 scripts/analyze_speed_policy_comparison.py \
    --config configs/ratio_assist/reserved_realistic_ratio_analysis_same_idx0.yaml \
    --start_centerline_idx "$IDX" \
    --model_path models/ppo_highspeed_ratio_trend_v1_1000k.zip \
    --assist_gain 0.50 \
    --out_dir "analysis_reserved_ratio_idx${IDX}_gain050"
done
```

Summarize:

```bash
for d in analysis_reserved_ratio_idx*_gain050; do
  echo "===== $d ====="
  cat "$d/evaluation_summary.csv"
  cat "$d/reward_breakdown_summary.csv"
done
```

---

## What to tune in ratio mode

Main training profile:

```text
configs/ratio_assist/highspeed_sim_stress_ratio_train.yaml
```

Main reserved application profile:

```text
configs/ratio_assist/reserved_realistic_ratio_template.yaml
configs/ratio_assist/reserved_realistic_ratio_eval_fixed_start_gate.yaml
configs/ratio_assist/reserved_realistic_ratio_analysis_same_idx0.yaml
```

Important parameters:

```yaml
speed:
  command_speed_min_mps: 1.0
  command_speed_max_mps: 10.0
  rule_curve_min_speed_mps: 1.0
  rule_straight_speed_mps: 6.0
  rule_speed_curvature_preview_m: 0.50

rl_residual:
  residual_output_mode: speed_ratio
  positive_assist_ratio: 0.667
  negative_assist_ratio: 0.50
  assist_gain: 1.0

reward:
  assist_smoothness_weight: 0.08
  assist_free_band: 0.15
  assist_excess_weight: 0.05
```

For reserved application, tune mainly:

```yaml
rl_residual:
  assist_gain: 0.25   # safer
  assist_gain: 0.50   # medium
  assist_gain: 0.75   # faster
  assist_gain: 1.00   # strongest transfer, simulation-only first
```

---

## RL gate and runtime logic

The optional environment gate still exists:

```yaml
gate:
  enable_rl_gate: true
```

It fades the learned assist to zero when cross-track or heading error is outside a safe domain. This is useful for fixed-start simulator diagnostics.

For the actual ROS2 racing stack, the higher-level mode selection may already activate RL inference only in the raceline-following scenario. In that case, the runtime does not need an extra gate inside `path_following_v2`; the inference node can simply publish no residual or zero residual when raceline mode is not active.

---

## New map / new raceline workflow

Train or at least re-evaluate when any of these change significantly:

```text
map geometry
raceline CSV
raceline direction
rule-speed profile
command speed envelope
steering lookahead behavior
reward design
observation mode: physical_mps vs speed_ratio
```

For a new map, copy the ratio configs:

```bash
cp configs/ratio_assist/highspeed_sim_stress_ratio_train.yaml configs/ratio_assist/my_map_highspeed_ratio_train.yaml
cp configs/ratio_assist/highspeed_sim_stress_ratio_eval_fixed_start_gate.yaml configs/ratio_assist/my_map_highspeed_ratio_eval.yaml
cp configs/ratio_assist/reserved_realistic_ratio_eval_fixed_start_gate.yaml configs/ratio_assist/my_map_reserved_ratio_eval.yaml
cp configs/ratio_assist/reserved_realistic_ratio_analysis_same_idx0.yaml configs/ratio_assist/my_map_reserved_ratio_analysis.yaml
```

Edit the `map:` block:

```yaml
map:
  map_path: ../f1tenth_gym_ros/maps/MyNewMap
  map_ext: .png
  centerline_csv: ../centerline_tools/centerline_output/my_new_map/raceline_points_smooth.csv
  corner_csv: ../centerline_tools/centerline_output/my_new_map/corner_key_points_edited.csv
```

Then run:

```bash
python3 scripts/evaluate_rule_based_speed.py \
  --config configs/ratio_assist/my_map_highspeed_ratio_eval.yaml

python3 scripts/train_ppo_speed.py \
  --config configs/ratio_assist/my_map_highspeed_ratio_train.yaml \
  --model_name ppo_my_map_highspeed_ratio_v1_1000k

python3 scripts/evaluate_ppo_speed.py \
  --config configs/ratio_assist/my_map_reserved_ratio_eval.yaml \
  --model_path models/ppo_my_map_highspeed_ratio_v1_1000k.zip \
  --assist_gain 0.25
```

Do **not** train PPO before the rule-based baseline completes safely on the selected map/raceline.

---

## Key outputs from analysis

`analyze_speed_policy_comparison.py` writes:

```text
evaluation_summary.csv
reward_breakdown_summary.csv
rule_timeseries.csv
model_timeseries.csv
aligned_by_cumulative_progress.csv
speed_vs_cumulative_progress.png
residual_vs_cumulative_progress.png
tracking_errors_vs_cumulative_progress.png
xy_speed_trace.png
corner_speed_summary.csv
corner_entrance_apex_exit_speeds.png
```

Acceptance checklist:

```text
lap completed
no crash
no bad tracking termination
model steps lower than rule-based
assist_ratio not saturated everywhere
tracking penalty not dramatically worse
reserved assist_gain can be reduced for safety without retraining
```

---

## Legacy physical-mps usage

Old configs are under:

```text
configs/legacy_physical_mps/
```

Example:

```bash
python3 scripts/evaluate_ppo_speed.py \
  --config configs/legacy_physical_mps/highspeed_sim_stress_eval_fixed_start_gate.yaml \
  --model_path models/V0_reward_ppo_speed_spielberg_1000k_20260612.zip
```

Use this only for previous physical-residual experiments. New competition-oriented transfer work should use `configs/ratio_assist/`.
