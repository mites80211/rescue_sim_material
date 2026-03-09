#!/usr/bin/env bash
set -euo pipefail

IMAGE="${IMAGE:-erebus-webots:latest}"
WORLD_PATH="${1:-/opt/erebus/game/worlds/world1.wbt}"
WEB_PORT="${2:-1234}"
WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
SUPERVISOR_URL="http://localhost:${WEB_PORT}/robot_windows/MainSupervisorWindow/MainSupervisorWindow.html?name=robot"

if ! [[ "${WEB_PORT}" =~ ^[0-9]+$ ]]; then
  echo "Porta non valida: ${WEB_PORT}"
  exit 1
fi

if ! docker info >/dev/null 2>&1; then
  echo "Docker non accessibile per l'utente corrente."
  echo "Usa: sudo --preserve-env=DISPLAY,WAYLAND_DISPLAY,XDG_RUNTIME_DIR $0 \"${WORLD_PATH}\" \"${WEB_PORT}\""
  exit 1
fi

DOCKER_ARGS=(
  run
  --rm
  -it
  --user "$(id -u):$(id -g)"
  -p "${WEB_PORT}:1234"
  --env WEBOTS_HOME=/usr/local/webots
  --env LANG=en_US.UTF-8
  --env LC_ALL=en_US.UTF-8
  --volume "${WORKSPACE_DIR}:/workspace"
  --device /dev/dri:/dev/dri
)

if [[ -n "${WAYLAND_DISPLAY:-}" && -n "${XDG_RUNTIME_DIR:-}" ]]; then
  DOCKER_ARGS+=(--env "WAYLAND_DISPLAY=${WAYLAND_DISPLAY}")
  DOCKER_ARGS+=(--env XDG_RUNTIME_DIR=/tmp/xdg-runtime-dir)
  DOCKER_ARGS+=(--volume "${XDG_RUNTIME_DIR}:/tmp/xdg-runtime-dir")
fi

if [[ -n "${DISPLAY:-}" ]]; then
  DOCKER_ARGS+=(--env "DISPLAY=${DISPLAY}")
  DOCKER_ARGS+=(--volume /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi

echo "MainSupervisor URL: ${SUPERVISOR_URL}"
if command -v xdg-open >/dev/null 2>&1; then
  (xdg-open "${SUPERVISOR_URL}" >/dev/null 2>&1 || true) &
fi

docker "${DOCKER_ARGS[@]}" \
  "${IMAGE}" \
  /usr/local/webots/webots "${WORLD_PATH}"
