#!/usr/bin/env bash
set -euo pipefail

CONFIG_FILE=${CONFIG_FILE:-experiments/spielberg_section_reward_v1.env}
if [ ! -f "$CONFIG_FILE" ]; then
  echo "Config file not found: $CONFIG_FILE" >&2
  exit 1
fi
source "$CONFIG_FILE"

BAD_TRACKING_ARGS=()
if [ "${ENABLE_BAD_TRACKING_TERMINATION:-0}" = "1" ]; then
  BAD_TRACKING_ARGS+=(--enable_bad_tracking_termination)
fi

RL_GATE_ARGS=()
if [ "${ENABLE_RL_GATE:-0}" = "1" ]; then
  RL_GATE_ARGS+=(--enable_rl_gate)
fi

RANDOM_START_ARGS=()
if [ "${RANDOM_START_ALONG_CENTERLINE:-0}" = "1" ]; then
  RANDOM_START_ARGS+=(--random_start_along_centerline)
fi

COMMON_ENV_ARGS=(
  --map_path "$MAP_PATH"
  --map_ext "$MAP_EXT"
  --centerline_csv "$CENTERLINE_CSV"
  --sx "$SX"
  --sy "$SY"
  --stheta "$STHETA"
  --min_speed "$MIN_SPEED"
  --max_speed "$MAX_SPEED"
  --rule_min_speed_mps "$RULE_MIN_SPEED_MPS"
  --rule_max_speed_mps "$RULE_MAX_SPEED_MPS"
  --max_delta_speed_mps "$MAX_DELTA_SPEED_MPS"
  --max_speed_delta_per_step_mps "$MAX_SPEED_DELTA_PER_STEP_MPS"
  --curvature_gain "$CURVATURE_GAIN"
  --rule_curvature_lookahead_points "$RULE_CURVATURE_LOOKAHEAD_POINTS"
  --model_curvature_short_points "$MODEL_CURVATURE_SHORT_POINTS"
  --model_curvature_mid_points "$MODEL_CURVATURE_MID_POINTS"
  --model_curvature_long_points "$MODEL_CURVATURE_LONG_POINTS"
  --target_lap_steps "$TARGET_LAP_STEPS"
  --reward_curvature_section_start_points "$REWARD_CURVATURE_SECTION_START_POINTS"
  --reward_curvature_section_end_points "$REWARD_CURVATURE_SECTION_END_POINTS"
  --curvature_speed_section_weight "$CURVATURE_SPEED_SECTION_WEIGHT"
  --target_speed_smoothness_weight "$TARGET_SPEED_SMOOTHNESS_WEIGHT"
  --crash_penalty_value "$CRASH_PENALTY_VALUE"
  --timeout_penalty_value "$TIMEOUT_PENALTY_VALUE"
  "${BAD_TRACKING_ARGS[@]}"
  --bad_tracking_min_steps "$BAD_TRACKING_MIN_STEPS"
  --bad_tracking_cte_threshold "$BAD_TRACKING_CTE_THRESHOLD"
  --bad_tracking_heading_threshold "$BAD_TRACKING_HEADING_THRESHOLD"
  "${RL_GATE_ARGS[@]}"
  --rl_gate_enable_cte "$RL_GATE_ENABLE_CTE"
  --rl_gate_enable_heading "$RL_GATE_ENABLE_HEADING"
  --rl_gate_disable_cte "$RL_GATE_DISABLE_CTE"
  --rl_gate_disable_heading "$RL_GATE_DISABLE_HEADING"
  --rl_gate_enable_count "$RL_GATE_ENABLE_COUNT"
  --rl_gate_disable_count "$RL_GATE_DISABLE_COUNT"
  --rl_gate_fade_in_step "$RL_GATE_FADE_IN_STEP"
  --rl_gate_fade_out_step "$RL_GATE_FADE_OUT_STEP"
)

TRAIN_RANDOM_ARGS=(
  "${RANDOM_START_ARGS[@]}"
  --start_lateral_noise_std "$START_LATERAL_NOISE_STD"
  --start_lateral_noise_max "$START_LATERAL_NOISE_MAX"
  --start_yaw_noise_std "$START_YAW_NOISE_STD"
  --start_yaw_noise_max "$START_YAW_NOISE_MAX"
  --obs_cte_noise_std "$OBS_CTE_NOISE_STD"
  --obs_heading_noise_std "$OBS_HEADING_NOISE_STD"
  --obs_speed_noise_std "$OBS_SPEED_NOISE_STD"
)

NO_NOISE_ARGS=(
  --start_lateral_noise_std 0.0
  --start_lateral_noise_max 0.0
  --start_yaw_noise_std 0.0
  --start_yaw_noise_max 0.0
  --obs_cte_noise_std 0.0
  --obs_heading_noise_std 0.0
  --obs_speed_noise_std 0.0
)
