# Ratio-assist configs

Default master config:

```text
V1_reward_ratio_spielberg_master.yaml
```

This one file replaces the older separate train/eval/analysis YAMLs for the ratio workflow.
Use it with:

```bash
python3 scripts/ratio_assist/run_ratio_experiment.py \
  --config configs/ratio_assist/V1_reward_ratio_spielberg_master.yaml \
  --stage rule-highspeed
```

Legacy physical-residual configs are intentionally separated under:

```text
configs/legacy_physical_mps/
```
