# Ratio-assist configs

These are the recommended configs for the current competition-oriented flow:

1. Train a trend model in `highspeed_sim_stress_ratio_train.yaml`.
2. Apply the same model to `reserved_realistic_ratio_eval_fixed_start_gate.yaml` or `reserved_realistic_ratio_analysis_same_idx0.yaml`.
3. Tune `assist_gain` on the reserved profile to choose safer/faster behavior without retraining.

The PPO action remains `[-1, 1]`, but in `speed_ratio` mode it is converted to a dimensionless assist ratio and then to the m/s residual expected by the simulator/runtime.
