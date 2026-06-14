# RoboRacer RL Training Module

This package contains the current **V0 speed-residual PPO training pipeline** for the RoboRacer / F1TENTH racing stack.

The purpose of this cleanup is to make the package easier to share with teammates. The current saved model is intentionally treated as a named baseline:

```text
models/V0_reward_ppo_speed_spielberg_1000k_20260612.zip
```

## Current status

### V0: implemented in this package

V0 is a **speed-residual-only RL pipeline**:

```text
rule-based raceline follower
        ↓
curvature-based rule speed
        ↓
PPO outputs speed residual
        ↓
gated + rate-limited target speed
        ↓
pure pursuit steering remains rule-based
```

The PPO action is one-dimensional:

```text
action = [speed_residual_action]
```

The model does **not** learn lateral offset yet.

### Planned V1: not implemented in this package yet

The next major design direction is:

```text
action = [Δspeed_index, Δlateral_offset]
```

That future V1 pipeline will let RL make both longitudinal and lateral local racing corrections. This README only documents the current V0 code and commands.

---

## Package layout

```text
rl_training/
├── README.md
├── README_RACING_AGENT.md              # compatibility copy of this README
├── setup.py
├── configs/
│   ├── V0_reward_ppo_speed_spielberg_train.yaml
│   ├── V0_reward_ppo_speed_spielberg_eval_fixed_start_gate.yaml
│   ├── V0_reward_ppo_speed_spielberg_analysis_same_idx0_gate.yaml
│   └── legacy/                         # old v1/v2 config names kept for reference
├── experiments/
│   └── V0_reward_ppo_speed_spielberg_1000k_20260612.env
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
    ├── train_ppo_speed.py
    └── presets/
```

---

## Setup

Run commands from the package root:

```bash
cd rl_training
pip3 install -e .
```

The config files assume this relative project structure:

```text
../f1tenth_gym_ros/maps/Spielberg_map.png
../centerline_tools/centerline_output/raceline_points_smooth.csv
../centerline_tools/centerline_output/corner_key_points_edited.csv
```

If your local paths differ, edit the `map:` section in the YAML config files.

---

## Main V0 model

The only kept trained model is:

```text
models/V0_reward_ppo_speed_spielberg_1000k_20260612.zip
```

Meaning:

```text
V0        = current speed-residual-only RL environment
reward    = section-curvature reward with residual smoothness/excess regularization
ppo       = Stable-Baselines3 PPO
spielberg = trained/evaluated on Spielberg map setup
1000k     = 1,000,000 training timesteps
20260612  = model date label
```

---

## Validate the rule-based baseline

Use this to check the non-RL baseline under the same V0 evaluation configuration:

```bash
python3 scripts/evaluate_rule_based_speed.py \
  --config configs/V0_reward_ppo_speed_spielberg_eval_fixed_start_gate.yaml
```

Expected output includes:

```text
=== Rule-based Evaluation Summary ===
Executed steps
Total reward
Average target speed
Lap completed
Crashed
Timeout
Bad tracking
Reward Breakdown
```

This is the baseline that the PPO residual model should beat or at least compare against.

---

## Validate the current V0 PPO model

```bash
python3 scripts/evaluate_ppo_speed.py \
  --config configs/V0_reward_ppo_speed_spielberg_eval_fixed_start_gate.yaml \
  --model_path models/V0_reward_ppo_speed_spielberg_1000k_20260612.zip
```

The evaluation uses:

```text
fixed start pose
no start noise
no observation noise
RL gate enabled
same reward breakdown as training
```

The RL gate is important because it mimics the intended runtime safety behavior: the learned residual is only faded in when tracking is healthy.

---

## Generate comparison analysis plots and CSVs

Use this to compare rule-based speed and PPO speed behavior over a rollout:

```bash
python3 scripts/analyze_speed_policy_comparison.py \
  --config configs/V0_reward_ppo_speed_spielberg_eval_fixed_start_gate.yaml \
  --model_path models/V0_reward_ppo_speed_spielberg_1000k_20260612.zip \
  --out_dir speed_policy_analysis_V0_fixed_start_gate \
  --print_every 100
```

Typical outputs are written into the selected output directory and may include:

```text
speed traces
rule speed vs PPO target speed
residual correction traces
curvature preview plots
tracking error plots
reward-term summaries
corner-related analysis if corner CSV is available
```

For a same-raceline-index analysis, use:

```bash
python3 scripts/analyze_speed_policy_comparison.py \
  --config configs/V0_reward_ppo_speed_spielberg_analysis_same_idx0_gate.yaml \
  --model_path models/V0_reward_ppo_speed_spielberg_1000k_20260612.zip \
  --out_dir speed_policy_analysis_V0_same_idx0_gate \
  --print_every 100
```

---

## Train a new V0 model from scratch

```bash
python3 scripts/train_ppo_speed.py \
  --config configs/V0_reward_ppo_speed_spielberg_train.yaml \
  --model_name V0_reward_ppo_speed_spielberg_retrain_1000k_20260614
```

The output will be:

```text
models/V0_reward_ppo_speed_spielberg_retrain_1000k_20260614.zip
```

The train config enables randomized starts and light observation noise. This is useful for robustness, but it means training behavior will not be identical to fixed-start evaluation.

---

## Continue training from the current V0 model

For an additional 200k timesteps:

```bash
python3 scripts/train_ppo_speed.py \
  --config configs/V0_reward_ppo_speed_spielberg_train.yaml \
  --load_model_path models/V0_reward_ppo_speed_spielberg_1000k_20260612.zip \
  --total_timesteps 200000 \
  --model_name V0_reward_ppo_speed_spielberg_1200k_continued_20260614
```

By default, the Stable-Baselines3 timestep counter is not reset when continuing training. Use this only if you intentionally want a new counter:

```bash
--reset_num_timesteps
```

---

## TensorBoard

Training logs are written under:

```text
models/tensorboard/
```

View them with:

```bash
tensorboard --logdir models/tensorboard
```

---

## Preset shortcut commands

The preset shell scripts use:

```text
experiments/V0_reward_ppo_speed_spielberg_1000k_20260612.env
```

by default.

Rule baseline:

```bash
bash scripts/presets/eval_rule_fixed_start.sh
```

V0 PPO evaluation:

```bash
bash scripts/presets/eval_ppo_fixed_start_gate.sh
```

V0 PPO analysis:

```bash
bash scripts/presets/analyze_ppo_fixed_start_gate.sh
```

Train using the environment-file preset:

```bash
bash scripts/presets/train_spielberg_section_reward.sh
```

Override the environment file if needed:

```bash
CONFIG_FILE=experiments/V0_reward_ppo_speed_spielberg_1000k_20260612.env \
  bash scripts/presets/eval_ppo_fixed_start_gate.sh
```

The YAML-based commands above are preferred for clarity because they show the actual V0 config file explicitly.

---

## V0 environment design

The current environment is `F110SpeedEnv` in:

```text
rl_training/f110_speed_env.py
```

### Control pipeline

```text
simulator state
      ↓
raceline-relative feature extraction
      ↓
curvature-based rule speed
      ↓
PPO speed residual
      ↓
optional RL residual gate
      ↓
target speed rate limit
      ↓
pure pursuit steering on raceline
      ↓
f110_gym step
```

### Action

```text
action = [correction_action]
```

where:

```text
correction_action ∈ [-1, 1]
raw_delta_speed_mps = max_delta_speed_mps * correction_action
```

The residual is then multiplied by the RL gate scale:

```text
delta_speed_mps = rl_gate_scale * raw_delta_speed_mps
```

The final command is:

```text
target_speed = clip(rule_speed_mps + delta_speed_mps, min_speed, max_speed)
```

followed by a target-speed rate limit.

### Observation

The current observation vector is:

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

Optional training-side observation noise can be applied to:

```text
current_speed_mps
cross_track_error
heading_error
```

This is a lightweight robustness feature. It is not yet a full localization uncertainty model.

### Rule speed

The rule speed is curvature-based:

```text
rule_speed_mps = max_rule_speed / (1 + curvature_gain * abs(rule_curvature))
```

then clamped between:

```text
rule_min_speed_mps
rule_max_speed_mps
```

In the current V0 setup:

```text
rule_min_speed_mps = 1.0
rule_max_speed_mps = 6.0
physical command clamp = 1.0 to 10.0 m/s
max PPO residual = ±4.0 m/s
```

### RL gate

The RL gate is a runtime-style safety mechanism.

It fades the PPO residual in only when tracking is healthy:

```text
abs(cross_track_error) < rl_gate_enable_cte
abs(heading_error) < rl_gate_enable_heading
```

It fades the residual out when tracking becomes poor:

```text
abs(cross_track_error) > rl_gate_disable_cte
abs(heading_error) > rl_gate_disable_heading
```

This keeps the rule-based speed as the fallback when the car is not confidently tracking the raceline.

---

## V0 reward design

The current V0 reward is behavior-focused and speed-residual-based:

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

### Positive terms

```text
progress_reward = 5.0 * progress_m
lap_reward = lap_bonus if lap completed
早 finish bonus = 200.0 * remaining_ratio if lap completed early
```

### Tracking penalty

```text
tracking_penalty = 0.8 * abs(cross_track_error) + 0.25 * abs(heading_error)
```

### Curvature-speed section penalty

Instead of punishing speed using only one curvature point, V0 uses average absolute curvature over a forward section:

```text
curvature_section_abs = mean(abs(curvature[i + start : i + end]))
```

Current default:

```text
start = 2 points ahead
end   = 40 points ahead
```

The penalty is:

```text
curvature_speed_section_penalty =
    curvature_speed_section_weight * curvature_section_abs * target_speed^2
```

### Smoothness penalties

Target-speed smoothness:

```text
target_speed_smoothness_penalty =
    target_speed_smoothness_weight * abs(target_speed - previous_target_speed)
```

Residual smoothness:

```text
residual_smoothness_penalty =
    residual_smoothness_weight * abs(delta_speed_mps - previous_delta_speed_mps)
```

Residual excess only activates outside a free band:

```text
residual_excess = max(0, abs(delta_speed_mps) - residual_free_band_mps)
residual_excess_penalty = residual_excess_weight * residual_excess^2
```

This allows useful moderate residuals while discouraging constantly extreme residual commands.

### Failure penalties

```text
crash_penalty = crash_penalty_value if crashed

timeout_penalty = timeout_penalty_value if timeout without lap completion
```

Bad-tracking termination can also mark the rollout as crashed when tracking error becomes too large for too long.

---

## Current robustness features

V0 includes only lightweight robustness mechanisms:

```text
random start along raceline
small start lateral/yaw noise
small observation noise on speed / CTE / heading
bad-tracking termination
RL residual gate
```

It does **not** yet include the future full localization-uncertainty design:

```text
estimated pose vs true pose split
pose delay model
slow localization bias/drift
boundary deformation randomization
LiDAR/corridor observation noise
lateral offset safety clipping
```

Those belong to the future V1 speed+lateral pipeline.

---

## Recommended comparison workflow

For each new model:

1. Validate rule baseline.
2. Validate PPO model with the same fixed-start gate config.
3. Generate analysis plots/CSVs.
4. Compare at least:

```text
lap completed
executed steps
total reward
average target speed
crash / timeout / bad tracking
reward breakdown
residual magnitude and smoothness
curvature-speed penalty
tracking penalty
```

Do not judge the model only by final reward. A useful model should be faster or more robust than the rule baseline without causing worse tracking, crash risk, or command oscillation.
