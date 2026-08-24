#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
COMPOSE_FILE="${ROOT_DIR}/f1tenth_gym_ros/docker-compose_loc.yml"
GPU_COMPOSE_FILE="${ROOT_DIR}/f1tenth_gym_ros/docker-compose_gpu.yml"
SIM_CONTAINER="f1tenth_gym_ros_loc"
NOVNC_URL="http://localhost:8080/vnc.html"
RUN_MODE="gpu"
COMPOSE_FILES=()

usage() {
  cat <<'EOF'
Usage: ./dk.sh [--gpu|--cpu] <command>

Rendering mode:
  --gpu      NVIDIA GPU + host X11 window (default).
  --cpu      Software rendering + browser/noVNC.

Docker lifecycle:
  start      Start Docker services and enter the ROS container.
  up         Start Docker services in the background.
  enter      Enter the already-running ROS container.
  image      Build the simulator Docker image only.
  restart    Stop services, start them, and enter the ROS container.
  stop       Stop Docker services; keep build/install volumes.
  status     Show container and ROS launch status.
  logs       Follow Docker service logs.
  run CMD    Run an arbitrary command in the ROS container.
  help       Show this help.

Inside the container:
  f1 build sim     Build simulator + RViz launch package.
  f1 build auto    Build the autonomy stack.
  f1 sim           Run IFAC with 3 static obstacles + 1 moving agent.
  f1 sim --no-agents
                   Run the same IFAC map without the moving agent.
  f1 auto          Run autonomy stack; Ctrl+C returns to the shell.
EOF
}

require_docker() {
  if ! command -v docker >/dev/null 2>&1; then
    echo "Required command not found: docker" >&2
    exit 1
  fi
}

configure_compose() {
  COMPOSE_FILES=(-f "${COMPOSE_FILE}")
  if [[ "${RUN_MODE}" == "gpu" ]]; then
    COMPOSE_FILES+=(-f "${GPU_COMPOSE_FILE}")
  fi
}

compose() {
  docker compose "${COMPOSE_FILES[@]}" "$@"
}

sim_is_running() {
  docker inspect -f '{{.State.Running}}' "${SIM_CONTAINER}" 2>/dev/null \
    | grep -q '^true$'
}

require_gpu() {
  if ! command -v nvidia-smi >/dev/null 2>&1 || ! nvidia-smi -L >/dev/null 2>&1; then
    echo "GPU mode is the default, but no working NVIDIA GPU was detected." >&2
    echo "Use CPU mode instead: ./dk.sh --cpu start" >&2
    exit 1
  fi

  if ! docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q 'nvidia'; then
    echo "The NVIDIA Docker runtime is not installed or configured." >&2
    echo "Install NVIDIA Container Toolkit, or use: ./dk.sh --cpu start" >&2
    exit 1
  fi

  if [[ -z "${DISPLAY:-}" ]]; then
    echo "GPU mode needs a host X11 DISPLAY for accelerated RViz." >&2
    echo "Use CPU/noVNC mode instead: ./dk.sh --cpu start" >&2
    exit 1
  fi
}

container_mode() {
  docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "${SIM_CONTAINER}" \
    2>/dev/null | sed -n 's/^F1TENTH_RENDER_MODE=//p' | head -1
}

up() {
  if [[ "${RUN_MODE}" == "gpu" ]]; then
    require_gpu
    if command -v xhost >/dev/null 2>&1; then
      xhost +SI:localuser:root >/dev/null
    fi
    compose stop novnc >/dev/null 2>&1 || true
    compose up -d --remove-orphans sim
    echo "Docker services are running in GPU mode."
    echo "RViz will open as a host X11 window on DISPLAY=${DISPLAY}."
  else
    compose up -d --remove-orphans sim novnc
    echo "Docker services are running in CPU mode."
    echo "RViz/noVNC: ${NOVNC_URL}"
  fi
}

enter() {
  if ! sim_is_running; then
    echo "The ROS container is not running. Start it with:" >&2
    echo "  ./dk.sh up" >&2
    exit 1
  fi

  compose exec sim bash -lc '
    cd /sim_ws
    source /opt/ros/foxy/setup.bash
    if [[ -f install/local_setup.bash ]]; then
      source install/local_setup.bash
    fi
    echo "Container ready. Try: f1 help"
    exec bash
  '

  echo "Exited the shell; Docker services are still running."
  echo "Stop them explicitly with: ./dk.sh stop"
}

start() {
  up
  enter
}

build_image() {
  compose build sim
}

stop() {
  compose stop
  echo "Docker services stopped; the ROS build/install volumes were preserved."
}

status() {
  compose ps

  if sim_is_running; then
    echo
    echo "Active rendering mode: $(container_mode)"
    echo
    echo "ROS launch processes:"
    docker top "${SIM_CONTAINER}" -eo pid,cmd 2>/dev/null \
      | grep -E 'PID|ros2 launch|gym_bridge|rviz2|path_following|reactive|arbitr|raceline' \
      || true
  fi
}

main() {
  require_docker
  configure_compose

  case "${1:-help}" in
    start)
      start
      ;;
    up | startd)
      up
      ;;
    enter | shell)
      enter
      ;;
    image)
      build_image
      ;;
    restart)
      stop
      start
      ;;
    stop | down)
      stop
      ;;
    status | ps)
      status
      ;;
    logs)
      compose logs -f
      ;;
    run)
      shift
      if [[ $# -eq 0 ]]; then
        echo "Usage: ./dk.sh run <command> [arguments...]" >&2
        exit 1
      fi
      if ! sim_is_running; then
        echo "The ROS container is not running. Start it with: ./dk.sh up" >&2
        exit 1
      fi
      compose exec sim "$@"
      ;;
    help | -h | --help)
      usage
      ;;
    *)
      echo "Unknown command: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
}

POSITIONAL=()
for argument in "$@"; do
  case "${argument}" in
    --gpu)
      RUN_MODE="gpu"
      ;;
    --cpu)
      RUN_MODE="cpu"
      ;;
    *)
      POSITIONAL+=("${argument}")
      ;;
  esac
done

main "${POSITIONAL[@]}"
