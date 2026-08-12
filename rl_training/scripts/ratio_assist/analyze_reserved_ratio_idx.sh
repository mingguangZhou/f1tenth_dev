#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
IDX="${1:-0}"
MODEL_PATH="${2:-models/ppo_highspeed_ratio_trend_v1.zip}"
OUT_DIR="${3:-analysis_reserved_ratio_idx${IDX}}"
shift 3 2>/dev/null || true
python3 scripts/analyze_speed_policy_comparison.py \
  --config configs/ratio_assist/reserved_realistic_ratio_analysis_same_idx0.yaml \
  --start_centerline_idx "$IDX" \
  --model_path "$MODEL_PATH" \
  --out_dir "$OUT_DIR" \
  "$@"
