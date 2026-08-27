#!/usr/bin/env bash

# Run repeatable Spielberg obstacle trials while keeping the simulator alive.
set -eo pipefail

LOCK_FILE="/tmp/f1tenth_runtime_trials.lock"
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  echo "Another obstacle trial run is already active (${LOCK_FILE})." >&2
  exit 1
fi

TRIALS=5
DURATION_SEC=20
OBSTACLE_ID=4
OUTPUT_DIR="/sim_ws/src/path_following_v2/trial_logs"
AUTO_PID=""
REUSE_STACK=false

usage() {
  cat <<'EOF'
Usage: run_obstacle_trials.sh [--obstacle 1-4] [--trials N] [--duration SEC]
                              [--output-dir DIR] [--reuse-stack]

Run this inside the ROS container while `f1 sim --no-agents` is running. The
script starts one autonomy stack, waits for ROS discovery, then resets
planner/arbitrator and vehicle state before every trial. It records synchronized
planner, guard, arbitrator, Reactive, scan, and drive data.

Use --reuse-stack to test an already-running `f1 auto` process. In that mode,
the script resets but does not stop that autonomy stack.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --obstacle)
      OBSTACLE_ID="$2"
      shift 2
      ;;
    --trials)
      TRIALS="$2"
      shift 2
      ;;
    --duration)
      DURATION_SEC="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --reuse-stack)
      REUSE_STACK=true
      shift
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

source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash

case "${OBSTACLE_ID}" in
  1)
    START_X=22.206
    START_Y=7.539
    START_YAW=0.9723
    ;;
  2)
    START_X=14.913
    START_Y=25.983
    START_YAW=2.9183
    ;;
  3)
    START_X=-11.627
    START_Y=24.940
    START_YAW=-3.0987
    ;;
  4)
    START_X=-42.290
    START_Y=18.598
    START_YAW=2.4299
    ;;
  *)
    echo "--obstacle must be one of 1, 2, 3, or 4" >&2
    exit 2
    ;;
esac

YAW_Z=$(python3 -c "import math; print(math.sin(${START_YAW} / 2.0))")
YAW_W=$(python3 -c "import math; print(math.cos(${START_YAW} / 2.0))")

stop_car() {
  timeout 3 ros2 topic pub -1 /drive ackermann_msgs/msg/AckermannDriveStamped \
    '{drive: {speed: 0.0, steering_angle: 0.0}}' >/dev/null 2>&1 || true
}

reset_car() {
  timeout 3 ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: ${START_X}, y: ${START_Y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: ${YAW_Z}, w: ${YAW_W}}}}}" \
    >/dev/null
}

reset_arbitration() {
  timeout 3 ros2 topic pub -1 /drive_arbitration_v2/reset std_msgs/msg/Bool \
    '{data: true}' >/dev/null
}

reset_planner() {
  timeout 3 ros2 topic pub -1 /path_following_v2/reset std_msgs/msg/Bool \
    '{data: true}' >/dev/null
}

stop_existing_autonomy() {
  local pids
  pids=$(pgrep -f '^/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch oudtra_driver_bringup full_stack_sim_launch.py' || true)
  if [[ -n "${pids}" ]]; then
    kill -INT ${pids} 2>/dev/null || true
    for _ in $(seq 1 30); do
      if ! pgrep -f '^/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch oudtra_driver_bringup full_stack_sim_launch.py' >/dev/null; then
        break
      fi
      sleep 0.1
    done
    for pid in ${pids}; do
      if kill -0 "${pid}" 2>/dev/null; then
        kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
      fi
    done
  fi
}

stop_trial_autonomy() {
  if [[ -n "${AUTO_PID}" ]] && kill -0 "${AUTO_PID}" 2>/dev/null; then
    kill -INT -- "-${AUTO_PID}" 2>/dev/null || kill -INT "${AUTO_PID}" 2>/dev/null || true
    for _ in $(seq 1 30); do
      if ! kill -0 "${AUTO_PID}" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    if kill -0 "${AUTO_PID}" 2>/dev/null; then
      kill -TERM -- "-${AUTO_PID}" 2>/dev/null || kill -TERM "${AUTO_PID}" 2>/dev/null || true
    fi
    wait "${AUTO_PID}" 2>/dev/null || true
  fi
  AUTO_PID=""
  stop_car
}

cleanup() {
  stop_trial_autonomy
}
trap cleanup EXIT INT TERM

mkdir -p "${OUTPUT_DIR}"
if [[ "${REUSE_STACK}" == false ]]; then
  stop_existing_autonomy
fi
stop_car
reset_car

echo "Running ${TRIALS} obstacle-${OBSTACLE_ID} trials from (${START_X}, ${START_Y}, ${START_YAW})."
launch_log="${OUTPUT_DIR}/autonomy.launch.log"
if [[ "${REUSE_STACK}" == false ]]; then
  setsid ros2 launch oudtra_driver_bringup full_stack_sim_launch.py \
    >"${launch_log}" 2>&1 &
  AUTO_PID=$!
fi

ready=false
for _ in $(seq 1 100); do
  if pgrep -f '^/sim_ws/install/path_following_v2/lib/path_following_v2/local_trajectory_planner_node' >/dev/null &&
    pgrep -f '^/sim_ws/install/reactive_control_v2/lib/reactive_control_v2/lower_safety_controller' >/dev/null
  then
    ready=true
    break
  fi
  if [[ "${REUSE_STACK}" == false ]] && ! kill -0 "${AUTO_PID}" 2>/dev/null; then
    break
  fi
  sleep 0.1
done
if [[ "${ready}" != true ]]; then
  echo "Autonomy failed to become ready; see ${launch_log}" >&2
  exit 1
fi

# Discovery of high-rate scan/odometry publishers can lag process creation.
# Exclude that startup transient from obstacle trials.
sleep 3.0
reset_planner
reset_car
reset_arbitration
sleep 1.0

for trial in $(seq 1 "${TRIALS}"); do
  label="obstacle_${OBSTACLE_ID}_trial_${trial}"
  csv_path="${OUTPUT_DIR}/${label}.csv"

  python3 /sim_ws/src/path_following_v2/tools/obstacle_trial_logger.py \
    --trial "${label}" \
    --duration "${DURATION_SEC}" \
    --reset-x "${START_X}" \
    --reset-y "${START_Y}" \
    --reset-yaw "${START_YAW}" \
    --output "${csv_path}"
  echo "Completed ${label}."
done

echo "Trial outputs: ${OUTPUT_DIR}"
