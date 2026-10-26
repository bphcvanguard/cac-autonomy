#!/usr/bin/env bash
# One entry point for the dev container.
#   ./scripts/dev.sh up      build (first time) and start the container
#   ./scripts/dev.sh sh      open a shell inside it
#   ./scripts/dev.sh build   colcon build inside it
#   ./scripts/dev.sh test    build + test, exactly like CI
#   ./scripts/dev.sh down    stop it
set -euo pipefail
cd "$(dirname "$0")/.."
export CAC_UID="$(id -u)" CAC_GID="$(id -g)"
COMPOSE=(docker compose -f docker/compose.yaml)

# Use the GPU service only if the NVIDIA container runtime is installed.
if docker info 2>/dev/null | grep -qi 'nvidia'; then SVC=dev; else SVC=dev-cpu; fi

case "${1:-}" in
  up)
    xhost +local: >/dev/null 2>&1 || true   # let the container open Gazebo/RViz windows
    "${COMPOSE[@]}" up -d --build "$SVC"
    echo "Started $SVC. Next: ./scripts/dev.sh sh" ;;
  sh)
    "${COMPOSE[@]}" exec "$SVC" bash ;;
  build)
    "${COMPOSE[@]}" exec "$SVC" bash -lc 'colcon build --symlink-install' ;;
  test)
    "${COMPOSE[@]}" exec "$SVC" bash -lc \
      'colcon build --symlink-install && colcon test && colcon test-result --verbose' ;;
  down)
    "${COMPOSE[@]}" down ;;
  *)
    sed -n '2,7p' "$0"; exit 1 ;;
esac
