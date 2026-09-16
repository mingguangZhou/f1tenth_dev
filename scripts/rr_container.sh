#!/usr/bin/env bash

set -euo pipefail

CONTAINER_NAME="${ROBORACER_CONTAINER:-f1tenth_gym_ros_rocker}"
WORKSPACE="${ROBORACER_WS:-/sim_ws}"

usage() {
    cat <<EOF
Usage:
  $0 status
  $0 shell
  $0 exec '<command>'

Environment overrides:
  ROBORACER_CONTAINER   Docker container name
                       default: f1tenth_gym_ros_rocker

  ROBORACER_WS          ROS workspace inside container
                       default: /sim_ws

Examples:
  $0 status

  $0 shell

  $0 exec 'source /opt/ros/foxy/setup.bash && cd /sim_ws && colcon list'

  $0 exec 'source /opt/ros/foxy/setup.bash && cd /sim_ws && \
           colcon build --packages-select drive_arbitration_v2'
EOF
}

container_running() {
    docker inspect \
        --format '{{.State.Running}}' \
        "${CONTAINER_NAME}" 2>/dev/null | grep -qx 'true'
}

require_container() {
    if ! container_running; then
        echo "ERROR: Docker container '${CONTAINER_NAME}' is not running." >&2
        echo "Start the localization-ready RoboRacer container first." >&2
        exit 1
    fi
}

case "${1:-}" in
    status)
        require_container

        echo "Container: ${CONTAINER_NAME}"
        docker inspect \
            --format 'Status={{.State.Status}} Image={{.Image}}' \
            "${CONTAINER_NAME}"

        docker exec "${CONTAINER_NAME}" \
            /bin/bash --noprofile --norc -c \
            'printf "ROS_DISTRO=%s\n" "${ROS_DISTRO:-unset}"'

        echo "Workspace: ${WORKSPACE}"
        ;;

    shell)
        require_container

        exec docker exec -it "${CONTAINER_NAME}" /bin/bash
        ;;

    exec)
        require_container

        if [[ $# -lt 2 ]]; then
            echo "ERROR: exec requires a command." >&2
            usage
            exit 2
        fi

        shift
        COMMAND="$*"

        exec docker exec "${CONTAINER_NAME}" \
            /bin/bash --noprofile --norc -c "${COMMAND}"
        ;;

    -h|--help|help|"")
        usage
        ;;

    *)
        echo "ERROR: Unknown command: $1" >&2
        usage
        exit 2
        ;;
esac