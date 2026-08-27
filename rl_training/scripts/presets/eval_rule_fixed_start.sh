#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
source scripts/presets/common_spielberg_section_reward.sh

python3 scripts/evaluate_rule_based_speed.py \
  --steps "$STEPS" \
  "${COMMON_ENV_ARGS[@]}" \
  "${NO_NOISE_ARGS[@]}"
