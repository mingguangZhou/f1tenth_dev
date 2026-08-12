#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
python3 scripts/train_ppo_speed.py \
  --config configs/ratio_assist/highspeed_sim_stress_ratio_train.yaml \
  "$@"
