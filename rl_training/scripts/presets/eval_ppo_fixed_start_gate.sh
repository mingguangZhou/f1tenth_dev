#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
source scripts/presets/common_spielberg_section_reward.sh
MODEL_TO_EVAL=${1:-$MODEL_PATH}

python3 scripts/evaluate_ppo_speed.py \
  --model_path "$MODEL_TO_EVAL" \
  --steps "$STEPS" \
  "${COMMON_ENV_ARGS[@]}" \
  "${NO_NOISE_ARGS[@]}"
