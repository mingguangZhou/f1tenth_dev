#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
source scripts/presets/common_spielberg_section_reward.sh

python3 scripts/train_ppo_speed.py \
  "${COMMON_ENV_ARGS[@]}" \
  "${TRAIN_RANDOM_ARGS[@]}" \
  --max_episode_steps "$MAX_EPISODE_STEPS" \
  --learning_rate "$LEARNING_RATE" \
  --n_steps "$N_STEPS" \
  --batch_size "$BATCH_SIZE" \
  --gamma "$GAMMA" \
  --gae_lambda "$GAE_LAMBDA" \
  --clip_range "$CLIP_RANGE" \
  --ent_coef "$ENT_COEF" \
  --total_timesteps "$TOTAL_TIMESTEPS" \
  --model_dir "$MODEL_DIR" \
  --model_name "$MODEL_NAME"
