# rl_training — ratio-assist PPO speed pipeline

This package trains and evaluates a **speed-only** PPO policy for RoboRacer/F1TENTH.
Steering remains pure pursuit; PPO only assists the rule-based raceline speed.

Current recommended flow:

```text
train in highspeed_sim_stress
  -> PPO learns an assist trend from an extreme simulator profile

apply to reserved_realistic
  -> the same ratio model is scaled by assist_gain for safer/faster use
```

The core model/environment logic is unchanged from the latest ratio-assist design. The main usability change is that normal tuning now happens in **one master YAML** and one runner script.

## Main files

```text
configs/ratio_assist/V1_reward_ratio_spielberg_master.yaml
scripts/ratio_assist/run_ratio_experiment.py
models/
```

Legacy physical-residual configs are kept under:

```text
configs/legacy_physical_mps/
```

Use them only for old `physical_mps` models. New competition-oriented models should use the ratio-assist master YAML.

## Model interface

The recommended mode is:

```yaml
residual_output_mode: speed_ratio
```

Observation:

```text
[current_speed_ratio, rule_speed_ratio, cte, heading_error,
 curv_short_abs, curv_mid_abs, curv_long_abs, previous_assist_ratio]
```

Action conversion:

```text
if action >= 0:
    assist_ratio = action * positive_assist_ratio * assist_gain
else:
    assist_ratio = action * negative_assist_ratio * assist_gain

delta_speed_mps = assist_ratio * rule_speed_mps
```

So the model learns a dimensionless trend. `assist_gain` controls how strongly that trend is applied.

## One-file tuning

Edit:

```text
configs/ratio_assist/V1_reward_ratio_spielberg_master.yaml
```

Most common tuning sections:

```yaml
paths:       # new map/raceline/corner CSV
model:       # output model name/path
shared:
  reward:    # reward weights
profiles:
  highspeed_sim_stress:
    speed:   # training speed envelope
    train:   # PPO training steps/noise/random starts
  reserved_realistic:
    speed:   # reserved/competition speed envelope
    rl:      # reserved assist_gain
```

The V1 reward values currently in the master YAML are the latest sensible ratio-reward values:

```yaml
curvature_speed_section_weight: 0.010
assist_smoothness_weight: 0.35
assist_free_band: 0.12
assist_excess_weight: 1.0
```

Tuning intuition:

```text
Still boost-only:
  increase assist_excess_weight, reduce assist_free_band, or increase curvature_speed_section_weight.

Too conservative / little speed gain:
  reduce assist_excess_weight, increase assist_free_band, or reduce curvature_speed_section_weight.

Speed command visibly oscillates:
  increase target_speed_smoothness_weight slightly.
```

## Rule-based benchmark

Always run the rule-based benchmark before training or judging PPO.

Highspeed rule benchmark:

```bash
cd /sim_ws/src/rl_training
python3 scripts/ratio_assist/run_ratio_experiment.py \
  --config configs/ratio_assist/V1_reward_ratio_spielberg_master.yaml \
  --stage rule-highspeed
```

Reserved rule benchmark:

```bash
python3 scripts/ratio_assist/run_ratio_experiment.py \
  --config configs/ratio_assist/V1_reward_ratio_spielberg_master.yaml \
  --stage rule-reserved
```

These call `scripts/evaluate_rule_based_speed.py` through a generated stage config.

## Train V1 500k model

```bash
python3 scripts/ratio_assist/run_ratio_experiment.py \
  --config configs/ratio_assist/V1_reward_ratio_spielberg_master.yaml \
  --stage train-highspeed
```

The output model goes directly to:

```text
models/V1_reward_ratio_ppo_speed_spielberg_500k_20260812.zip
```

To override the name without editing YAML:

```bash
python3 scripts/ratio_assist/run_ratio_experiment.py \
  --config configs/ratio_assist/V1_reward_ratio_spielberg_master.yaml \
  --stage train-highspeed \
  --model_name MyNewRatioModel_500k
```

## Continue training

```bash
python3 scripts/ratio_assist/run_ratio_experiment.py \
  --config configs/ratio_assist/V1_reward_ratio_spielberg_master.yaml \
  --stage continue-highspeed \
  --load_model_path models/V1_reward_ratio_ppo_speed_spielberg_500k_20260812.zip \
  --total_timesteps 500000 \
  --model_name V1_reward_ratio_ppo_speed_spielberg_1000k_continued
```

## Evaluate PPO in highspeed profile

```bash
python3 scripts/ratio_assist/run_ratio_experiment.py \
  --config configs/ratio_assist/V1_reward_ratio_spielberg_master.yaml \
  --stage eval-highspeed \
  --model_path models/V1_reward_ratio_ppo_speed_spielberg_500k_20260812.zip
```

## Compare PPO vs rule in highspeed profile

This creates CSV/PNG comparison plots.

```bash
python3 scripts/ratio_assist/run_ratio_experiment.py \
  --config configs/ratio_assist/V1_reward_ratio_spielberg_master.yaml \
  --stage compare-highspeed \
  --start_centerline_idx 0 \
  --model_path models/V1_reward_ratio_ppo_speed_spielberg_500k_20260812.zip \
  --out_dir analysis_V1_reward_ratio_highspeed_idx0
```

Useful outputs:

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

## Apply same model to reserved profile

Start with a conservative assist gain:

```bash
python3 scripts/ratio_assist/run_ratio_experiment.py \
  --config configs/ratio_assist/V1_reward_ratio_spielberg_master.yaml \
  --stage eval-reserved \
  --model_path models/V1_reward_ratio_ppo_speed_spielberg_500k_20260812.zip \
  --assist_gain 0.25
```

Compare against reserved rule baseline:

```bash
python3 scripts/ratio_assist/run_ratio_experiment.py \
  --config configs/ratio_assist/V1_reward_ratio_spielberg_master.yaml \
  --stage compare-reserved \
  --start_centerline_idx 0 \
  --model_path models/V1_reward_ratio_ppo_speed_spielberg_500k_20260812.zip \
  --assist_gain 0.25 \
  --out_dir analysis_V1_reward_ratio_reserved_idx0_gain025
```

Sweep reserved assist gains:

```bash
python3 scripts/ratio_assist/run_ratio_experiment.py \
  --config configs/ratio_assist/V1_reward_ratio_spielberg_master.yaml \
  --stage sweep-reserved \
  --model_path models/V1_reward_ratio_ppo_speed_spielberg_500k_20260812.zip \
  --assist_gains 0.10,0.25,0.50
```

## New map / new raceline

For a new map, edit only the `paths:` block first:

```yaml
paths:
  map_path: ../f1tenth_gym_ros/maps/MyNewMap
  map_ext: .png
  centerline_csv: ../centerline_tools/centerline_output/my_new_map/raceline_points_smooth.csv
  corner_csv: ../centerline_tools/centerline_output/my_new_map/corner_key_points_edited.csv
```

Then run the rule-based benchmark before training:

```bash
python3 scripts/ratio_assist/run_ratio_experiment.py \
  --config configs/ratio_assist/V1_reward_ratio_spielberg_master.yaml \
  --stage rule-highspeed
```

Only train PPO after the rule-based baseline completes safely.

## Low-level scripts

The original scripts are still available for debugging:

```text
scripts/evaluate_rule_based_speed.py
scripts/train_ppo_speed.py
scripts/evaluate_ppo_speed.py
scripts/analyze_speed_policy_comparison.py
```

The runner simply generates temporary YAMLs under:

```text
runs/<experiment_name>/generated_configs/
```

and calls the same low-level scripts.
