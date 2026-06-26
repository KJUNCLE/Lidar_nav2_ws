#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"

IMAGE="${IMAGE:-nav2:latest}"
CONTAINER_NAME="${CONTAINER_NAME:-nav2}"
HOST_WORKSPACE="${HOST_WORKSPACE:-$WORKSPACE_ROOT}"
CONTAINER_WORKSPACE="${CONTAINER_WORKSPACE:-/workspace/$(basename "$HOST_WORKSPACE")}"
ENABLE_X11="${ENABLE_X11:-1}"
ENABLE_GPU="${ENABLE_GPU:-0}"
DISPLAY_VALUE="${DISPLAY:-}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<EOF
Usage:
  ./scripts/docker_run.sh

Environment overrides:
  IMAGE=nav2/lidar:v1
  CONTAINER_NAME=nav2
  HOST_WORKSPACE=$WORKSPACE_ROOT
  CONTAINER_WORKSPACE=/workspace/$(basename "$WORKSPACE_ROOT")
  ENABLE_X11=1
  ENABLE_GPU=0
  DISPLAY=:0
EOF
  exit 0
fi

if [[ ! -d "$HOST_WORKSPACE" ]]; then
  echo "error: HOST_WORKSPACE does not exist: $HOST_WORKSPACE" >&2
  exit 1
fi

if docker ps -a --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
  echo "error: container already exists: $CONTAINER_NAME" >&2
  echo "remove it first: docker rm -f $CONTAINER_NAME" >&2
  exit 1
fi

GPU_ARGS=()
if [[ "$ENABLE_GPU" == "1" ]] && command -v nvidia-smi >/dev/null 2>&1; then
  GPU_ARGS+=(--gpus all)
fi

X11_ARGS=()
XHOST_GRANTED=0
if [[ "$ENABLE_X11" == "1" ]]; then
  if [[ -z "$DISPLAY_VALUE" ]]; then
    echo "warning: DISPLAY is empty; RViz GUI will not be available in the container." >&2
  elif [[ ! -d /tmp/.X11-unix ]]; then
    echo "warning: /tmp/.X11-unix does not exist; RViz GUI will not be available in the container." >&2
  else
    X11_ARGS+=(
      -e "DISPLAY=$DISPLAY_VALUE"
      -e "QT_X11_NO_MITSHM=1"
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    )

    if [[ -n "${XAUTHORITY:-}" && -f "$XAUTHORITY" ]]; then
      X11_ARGS+=(-e XAUTHORITY=/tmp/.docker.xauth -v "$XAUTHORITY:/tmp/.docker.xauth:ro")
    elif [[ -f "${HOME:-}/.Xauthority" ]]; then
      X11_ARGS+=(-e XAUTHORITY=/tmp/.docker.xauth -v "$HOME/.Xauthority:/tmp/.docker.xauth:ro")
    fi

    if command -v xhost >/dev/null 2>&1; then
      if xhost +SI:localuser:root >/dev/null 2>&1; then
        XHOST_GRANTED=1
      else
        echo "warning: failed to grant X11 access to local root. If RViz fails, run: xhost +SI:localuser:root" >&2
      fi
    else
      echo "warning: xhost not found. If RViz fails, install x11-xserver-utils on the host." >&2
    fi
  fi
fi

cleanup() {
  if [[ "$XHOST_GRANTED" == "1" ]] && command -v xhost >/dev/null 2>&1; then
    xhost -SI:localuser:root >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

DRI_ARGS=()
if [[ -d /dev/dri ]]; then
  DRI_ARGS+=(--device /dev/dri)
fi

ROS_ARGS=()
if [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
  ROS_ARGS+=(-e "ROS_DOMAIN_ID=$ROS_DOMAIN_ID")
fi
if [[ -n "${RMW_IMPLEMENTATION:-}" ]]; then
  ROS_ARGS+=(-e "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION")
fi

docker run -it \
  --name "$CONTAINER_NAME" \
  --network host \
  --ipc host \
  "${X11_ARGS[@]}" \
  "${ROS_ARGS[@]}" \
  -v "$HOST_WORKSPACE:$CONTAINER_WORKSPACE:rw" \
  -w "$CONTAINER_WORKSPACE" \
  "${GPU_ARGS[@]}" \
  "${DRI_ARGS[@]}" \
  "$IMAGE" \
  /bin/bash
