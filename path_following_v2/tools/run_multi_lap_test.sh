#!/usr/bin/env bash

# Record complete Spielberg laps through the configured obstacle map.
set -eo pipefail

LOCK_FILE="/tmp/f1tenth_runtime_trials.lock"
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  echo "Another simulator trial is already active (${LOCK_FILE})." >&2
  exit 1
fi

TARGET_LAPS=3
TIMEOUT_SEC=600
SAMPLE_HZ=20
OUTPUT_DIR="/sim_ws/src/path_following_v2/trial_logs/multi_lap"
RACELINE_CSV="/sim_ws/src/centerline_tools/output_backup/V0_reward_ppo_speed_spielberg_1000k_20260612/raceline_points_smooth.csv"

usage() {
  cat <<'EOF'
Usage: run_multi_lap_test.sh [--laps N] [--timeout SEC] [--sample-hz HZ]
                             [--output-dir DIR] [--raceline-csv PATH]

Run this inside the ROS container after starting one simulator and one autonomy
stack. The car, planner, and arbitrator are reset to the configured Spielberg
start before data collection. The command fails if the requested complete-lap
count is not reached before the timeout.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --laps)
      TARGET_LAPS="$2"
      shift 2
      ;;
    --timeout)
      TIMEOUT_SEC="$2"
      shift 2
      ;;
    --sample-hz)
      SAMPLE_HZ="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --raceline-csv)
      RACELINE_CSV="$2"
      shift 2
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! -f /opt/ros/foxy/setup.bash || ! -d /sim_ws ]]; then
  echo "Run this script inside the F1TENTH ROS container." >&2
  exit 1
fi
if [[ ! "${TARGET_LAPS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "--laps must be a positive integer" >&2
  exit 2
fi
if [[ ! -f "${RACELINE_CSV}" ]]; then
  echo "Raceline CSV not found: ${RACELINE_CSV}" >&2
  exit 1
fi

source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash

if ! pgrep -f '^/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch f1tenth_gym_ros gym_bridge_launch.py' >/dev/null; then
  echo "Simulator launch is not running." >&2
  exit 1
fi
if ! pgrep -f '^/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch oudtra_driver_bringup full_stack_sim_launch.py' >/dev/null; then
  echo "Autonomy launch is not running." >&2
  exit 1
fi

mkdir -p "${OUTPUT_DIR}"
label="spielberg_key_turns_${TARGET_LAPS}_laps"
csv_path="${OUTPUT_DIR}/${label}.csv"

python3 /sim_ws/src/path_following_v2/tools/obstacle_trial_logger.py \
  --trial "${label}" \
  --duration "${TIMEOUT_SEC}" \
  --sample-hz "${SAMPLE_HZ}" \
  --target-laps "${TARGET_LAPS}" \
  --raceline-csv "${RACELINE_CSV}" \
  --reset-x 17.0 \
  --reset-y 4.5 \
  --reset-yaw 0.7 \
  --output "${csv_path}"

summary_path="${OUTPUT_DIR}/${label}.summary.json"
python3 - "${summary_path}" <<'PY'
import json
import sys

path = sys.argv[1]
with open(path, "r", encoding="utf-8") as stream:
    summary = json.load(stream)
if not summary.get("target_laps_reached", False):
    raise SystemExit(
        f"multi-lap gate failed: completed {summary.get('completed_laps', 0)} "
        f"of {summary.get('target_laps', 0)} requested laps"
    )
print(
    f"multi-lap gate passed: {summary['completed_laps']} laps; "
    f"lap durations={summary['lap_durations_sec']}"
)
PY

echo "Multi-lap outputs: ${OUTPUT_DIR}"
