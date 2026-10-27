#!/usr/bin/env bash
# One entry point for the dev container.
#   ./scripts/dev.sh up      build (first time) and start the container
#   ./scripts/dev.sh sh      open a shell inside it
#   ./scripts/dev.sh build   colcon build inside it
#   ./scripts/dev.sh test    build + test, exactly like CI
#   ./scripts/dev.sh info    which graphics mode you got, CPU/RAM, ROS version
#   ./scripts/dev.sh down    stop it
# Graphics mode is picked automatically. Force one with CAC_GPU=nvidia|dri|none.
set -euo pipefail
cd "$(dirname "$0")/.."
export CAC_UID="$(id -u)" CAC_GID="$(id -g)"
COMPOSE=(docker compose -f docker/compose.yaml)

# nvidia: NVIDIA GPU + Container Toolkit      -> service dev
# dri:    Intel/AMD (or NVIDIA w/o toolkit)   -> service dev-cpu
# none:   no /dev/dri (VMs, old PCs)          -> service dev-sw (software rendering)
render_node="$(ls /dev/dri/renderD* 2>/dev/null | head -1 || true)"
if [ -n "${CAC_GPU:-}" ]; then
  MODE="$CAC_GPU"
elif docker info 2>/dev/null | grep -qi 'runtimes:.*nvidia' && nvidia-smi -L >/dev/null 2>&1; then
  MODE=nvidia
elif [ -n "$render_node" ]; then
  MODE=dri
else
  MODE=none
fi
case "$MODE" in
  nvidia) SVC=dev ;;
  dri)    SVC=dev-cpu ;;
  none)   SVC=dev-sw ;;
  *) echo "CAC_GPU must be nvidia, dri or none (got '$MODE')"; exit 1 ;;
esac
if [ "$MODE" = dri ] && [ -z "$render_node" ]; then
  echo "CAC_GPU=dri but this machine has no /dev/dri/renderD*. Use CAC_GPU=none."; exit 1
fi
# The container user must be in the host's render/video groups to open /dev/dri.
export CAC_RENDER_GID="$( [ -n "$render_node" ] && stat -c %g "$render_node" || getent group render | cut -d: -f3 || echo 109)"
export CAC_VIDEO_GID="$(getent group video | cut -d: -f3 || echo 44)"

# For sh/build/test/info, talk to whichever service is already running.
running="$("${COMPOSE[@]}" ps --status running --services 2>/dev/null | grep -E '^dev(-cpu|-sw)?$' | head -1 || true)"
need_running() {
  [ -n "$running" ] || { echo "Container not running. Start it with: ./scripts/dev.sh up"; exit 1; }
}

case "${1:-}" in
  up)
    if [ -n "$running" ] && [ "$running" != "$SVC" ]; then
      echo "Switching from $running to $SVC"; "${COMPOSE[@]}" stop "$running"
    fi
    xhost +local: >/dev/null 2>&1 || true   # let the container open Gazebo/RViz windows
    "${COMPOSE[@]}" up -d --build "$SVC"
    echo "Started $SVC (graphics: $MODE). Next: ./scripts/dev.sh sh" ;;
  sh)
    need_running; "${COMPOSE[@]}" exec "$running" bash ;;
  build)
    need_running; "${COMPOSE[@]}" exec "$running" bash -lc 'colcon build --symlink-install' ;;
  test)
    need_running; "${COMPOSE[@]}" exec "$running" bash -lc \
      'colcon build --symlink-install && colcon test && colcon test-result --verbose' ;;
  info)
    need_running; "${COMPOSE[@]}" exec "$running" bash -lc '
      echo "service:  '"$running"'"
      echo "ROS:      $ROS_DISTRO ($RMW_IMPLEMENTATION)"
      echo "CPU:      $(nproc) cores, $(grep -m1 "model name" /proc/cpuinfo | cut -d: -f2 | xargs)"
      echo "RAM:      $(free -g | awk "/Mem:/{print \$2}") GB"
      echo "Graphics: $(glxinfo -B 2>/dev/null | grep -oP "renderer string: \K.*" || echo "no display (is DISPLAY set?)")"
      glxinfo -B 2>/dev/null | grep -q llvmpipe && echo "          llvmpipe = software rendering on the CPU. Works; see README \"Slow or old PC\"." || true' ;;
  down)
    "${COMPOSE[@]}" down ;;
  *)
    sed -n '2,9p' "$0"; exit 1 ;;
esac
