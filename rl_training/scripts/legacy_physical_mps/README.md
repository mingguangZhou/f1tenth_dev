# Legacy physical-mps workflow

The shared Python scripts in `../` still support the legacy mode. Use configs under `configs/legacy_physical_mps/` for old models and old experiments.

Example:

```bash
python3 scripts/evaluate_ppo_speed.py \
  --config configs/legacy_physical_mps/highspeed_sim_stress_eval_fixed_start_gate.yaml \
  --model_path models/V0_reward_ppo_speed_spielberg_1000k_20260612.zip
```
