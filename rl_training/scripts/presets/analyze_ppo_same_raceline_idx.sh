#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
source scripts/presets/common_spielberg_section_reward.sh
START_IDX=${1:-0}
MODEL_TO_EVAL=${2:-$MODEL_PATH}
OUT_DIR=${3:-speed_policy_analysis_section_reward_same_start_idx${START_IDX}}

read SX_FIXED SY_FIXED STHETA_FIXED <<< "$(
python3 - <<PY
import csv
csv_path = "$CENTERLINE_CSV"
idx = int("$START_IDX")
with open(csv_path, newline="") as f:
    rows = list(csv.DictReader(f))
if idx < 0 or idx >= len(rows):
    raise SystemExit(f"START_IDX {idx} out of range: 0..{len(rows)-1}")
row = rows[idx]
print(row["x"], row["y"], row["yaw"])
PY
)"

echo "Using raceline start index ${START_IDX}: sx=${SX_FIXED}, sy=${SY_FIXED}, stheta=${STHETA_FIXED}"

python3 scripts/analyze_speed_policy_comparison.py \
  --model_path "$MODEL_TO_EVAL" \
  --map_path "$MAP_PATH" \
  --map_ext "$MAP_EXT" \
  --centerline_csv "$CENTERLINE_CSV" \
  --corner_csv "$CORNER_CSV" \
  --sx "$SX_FIXED" \
  --sy "$SY_FIXED" \
  --stheta "$STHETA_FIXED" \
  --steps "$STEPS" \
  --min_speed "$MIN_SPEED" \
  --max_speed "$MAX_SPEED" \
  --rule_min_speed_mps "$RULE_MIN_SPEED_MPS" \
  --rule_max_speed_mps "$RULE_MAX_SPEED_MPS" \
  --max_delta_speed_mps "$MAX_DELTA_SPEED_MPS" \
  --max_speed_delta_per_step_mps "$MAX_SPEED_DELTA_PER_STEP_MPS" \
  --curvature_gain "$CURVATURE_GAIN" \
  --rule_curvature_lookahead_points "$RULE_CURVATURE_LOOKAHEAD_POINTS" \
  --model_curvature_short_points "$MODEL_CURVATURE_SHORT_POINTS" \
  --model_curvature_mid_points "$MODEL_CURVATURE_MID_POINTS" \
  --model_curvature_long_points "$MODEL_CURVATURE_LONG_POINTS" \
  --target_lap_steps "$TARGET_LAP_STEPS" \
  --reward_curvature_section_start_points "$REWARD_CURVATURE_SECTION_START_POINTS" \
  --reward_curvature_section_end_points "$REWARD_CURVATURE_SECTION_END_POINTS" \
  --curvature_speed_section_weight "$CURVATURE_SPEED_SECTION_WEIGHT" \
  --target_speed_smoothness_weight "$TARGET_SPEED_SMOOTHNESS_WEIGHT" \
  --crash_penalty_value "$CRASH_PENALTY_VALUE" \
  --timeout_penalty_value "$TIMEOUT_PENALTY_VALUE" \
  "${BAD_TRACKING_ARGS[@]}" \
  --bad_tracking_min_steps "$BAD_TRACKING_MIN_STEPS" \
  --bad_tracking_cte_threshold "$BAD_TRACKING_CTE_THRESHOLD" \
  --bad_tracking_heading_threshold "$BAD_TRACKING_HEADING_THRESHOLD" \
  "${RL_GATE_ARGS[@]}" \
  --rl_gate_enable_cte "$RL_GATE_ENABLE_CTE" \
  --rl_gate_enable_heading "$RL_GATE_ENABLE_HEADING" \
  --rl_gate_disable_cte "$RL_GATE_DISABLE_CTE" \
  --rl_gate_disable_heading "$RL_GATE_DISABLE_HEADING" \
  --rl_gate_enable_count "$RL_GATE_ENABLE_COUNT" \
  --rl_gate_disable_count "$RL_GATE_DISABLE_COUNT" \
  --rl_gate_fade_in_step "$RL_GATE_FADE_IN_STEP" \
  --rl_gate_fade_out_step "$RL_GATE_FADE_OUT_STEP" \
  "${NO_NOISE_ARGS[@]}" \
  --out_dir "$OUT_DIR" \
  --print_every 100
