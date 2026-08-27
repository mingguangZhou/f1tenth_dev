# Legacy physical-mps configs

These YAML files preserve the older pipeline where PPO action maps directly to a physical residual:

```text
delta_speed_mps = action * max_delta_speed_mps
```

They are kept for reproducing previous experiments and evaluating old saved models. New highspeed-train -> reserved-apply experiments should use `configs/ratio_assist/`.
