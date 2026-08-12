#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
MODEL_PATH="${1:-models/ppo_highspeed_ratio_trend_v1.zip}"
shift || true
python3 scripts/evaluate_ppo_speed.py \
  --config configs/ratio_assist/reserved_realistic_ratio_eval_fixed_start_gate.yaml \
  --model_path "$MODEL_PATH" \
  "$@"
