#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
source scripts/presets/common_spielberg_section_reward.sh
MODEL_TO_EVAL=${1:-$MODEL_PATH}
OUT_DIR=${2:-speed_policy_analysis_section_reward_fixed_start_gate}

python3 scripts/analyze_speed_policy_comparison.py \
  --model_path "$MODEL_TO_EVAL" \
  --corner_csv "$CORNER_CSV" \
  --steps "$STEPS" \
  "${COMMON_ENV_ARGS[@]}" \
  "${NO_NOISE_ARGS[@]}" \
  --out_dir "$OUT_DIR" \
  --print_every 100
