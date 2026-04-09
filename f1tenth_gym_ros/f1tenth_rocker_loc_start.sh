#!/usr/bin/env bash
set -e

# --------------------------------------------
# User-defined settings
# --------------------------------------------
CONTAINER_NAME="f1tenth_gym_ros_rocker"
IMAGE_NAME="f1tenth_gym_ros_localization_ready"

# Resolve script directory so relative mounts behave correctly
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Paths (same structure as docker-compose)
GYM_PATH="${SCRIPT_DIR}"
SAFETY_BUBBLE_PATH="${SCRIPT_DIR}/../reactive_control/safety_bubble"
BOUNDARY_DETECTION_PATH="${SCRIPT_DIR}/../path_following/boundary_detection"
PATH_FOLLOWING_PATH="${SCRIPT_DIR}/../path_following/path_following"
DRIVE_ARBITRATION_PATH="${SCRIPT_DIR}/../drive_arbitration"
SLAM_TOOLBOX_PATH="${SCRIPT_DIR}/../slam_toolbox"
RANGE_LIBC_PATH="${SCRIPT_DIR}/../range_libc"
PARTICLE_FILTER_PATH="${SCRIPT_DIR}/../particle_filter"

# --------------------------------------------
# Notify paths
# --------------------------------------------
echo "Starting rocker container with:"
echo "  Host → Container mount:"
echo "  ${GYM_PATH} → /sim_ws/src/f1tenth_gym_ros"
echo "  ${SAFETY_BUBBLE_PATH} → /sim_ws/src/safety_bubble"
echo "  ${BOUNDARY_DETECTION_PATH} → /sim_ws/src/boundary_detection"
echo "  ${PATH_FOLLOWING_PATH} → /sim_ws/src/path_following"
echo "  ${DRIVE_ARBITRATION_PATH} → /sim_ws/src/drive_arbitration"
echo "  ${SLAM_TOOLBOX_PATH} → /sim_ws/src/slam_toolbox"
echo "  ${RANGE_LIBC_PATH} → /sim_ws/src/range_libc"
echo "  ${PARTICLE_FILTER_PATH} → /sim_ws/src/particle_filter"
echo ""

# --------------------------------------------
# Run rocker with fixed container name
# --------------------------------------------
rocker --nvidia --x11 \
    --name "${CONTAINER_NAME}" \
    --volume "${GYM_PATH}:/sim_ws/src/f1tenth_gym_ros" \
    --volume "${SAFETY_BUBBLE_PATH}:/sim_ws/src/safety_bubble" \
    --volume "${BOUNDARY_DETECTION_PATH}:/sim_ws/src/boundary_detection" \
    --volume "${PATH_FOLLOWING_PATH}:/sim_ws/src/path_following" \
    --volume "${DRIVE_ARBITRATION_PATH}:/sim_ws/src/drive_arbitration" \
    --volume "${SLAM_TOOLBOX_PATH}:/sim_ws/src/slam_toolbox" \
    --volume "${RANGE_LIBC_PATH}:/sim_ws/src/range_libc" \
    --volume "${PARTICLE_FILTER_PATH}:/sim_ws/src/particle_filter" \
    --volume "${GYM_PATH}/install_localization_env.sh:/sim_ws/install_localization_env.sh" \
    -- "${IMAGE_NAME}"
