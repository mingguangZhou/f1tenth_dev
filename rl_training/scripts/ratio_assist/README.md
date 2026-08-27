# Ratio-assist runner

Use `run_ratio_experiment.py` as the main entry point. It reads one master YAML and calls the existing low-level scripts.

Default master YAML:

```text
configs/ratio_assist/V1_reward_ratio_spielberg_master.yaml
```

Example:

```bash
python3 scripts/ratio_assist/run_ratio_experiment.py \
  --config configs/ratio_assist/V1_reward_ratio_spielberg_master.yaml \
  --stage rule-highspeed
```

Supported stages:

```text
rule-highspeed       rule-based benchmark in highspeed profile
train-highspeed      train PPO in highspeed profile
continue-highspeed   continue PPO training from an existing model
eval-highspeed       evaluate PPO in highspeed profile
compare-highspeed    PPO-vs-rule plots/CSVs in highspeed profile
sweep-highspeed      compare several assist_gain values in highspeed profile
rule-reserved        rule-based benchmark in reserved profile
eval-reserved        apply trained PPO model in reserved profile
compare-reserved     PPO-vs-rule plots/CSVs in reserved profile
sweep-reserved       compare several assist_gain values in reserved profile
```
