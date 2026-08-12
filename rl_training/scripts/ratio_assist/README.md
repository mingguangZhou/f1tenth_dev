# Ratio-assist shell wrappers

Run from anywhere inside the package. All extra arguments are forwarded to the underlying Python script, so you can override YAML values with CLI flags.

Examples:

```bash
bash scripts/ratio_assist/train_highspeed_ratio.sh --model_name ppo_highspeed_ratio_trend_v1_1000k
bash scripts/ratio_assist/eval_highspeed_ratio.sh models/ppo_highspeed_ratio_trend_v1_1000k.zip
bash scripts/ratio_assist/eval_reserved_ratio.sh models/ppo_highspeed_ratio_trend_v1_1000k.zip --assist_gain 0.25
bash scripts/ratio_assist/analyze_reserved_ratio_idx.sh 0 models/ppo_highspeed_ratio_trend_v1_1000k.zip analysis_reserved_ratio_idx0 --assist_gain 0.5
```
