#!/bin/bash
# Local demo run WITH the Gazebo GUI (X11). Logs land in ./run_logs, plots too.
#   ./scripts/run_local.sh            # GUI (default)
#   HEADLESS=1 ./scripts/run_local.sh # no GUI
#
# Mounts the repo source into the image so it always runs the current code
# (no rebuild needed after editing a node/launch/param).
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE="${IMAGE:-drone_system:latest}"
HEADLESS="${HEADLESS:-0}"

# If the docker group isn't active in this shell yet, re-exec through it.
if ! docker info >/dev/null 2>&1 && [ -z "${_SG_WRAP:-}" ]; then
    export _SG_WRAP=1
    exec sg docker -c "bash '$REPO/scripts/run_local.sh' $*"
fi

mkdir -p "$REPO/run_logs"
xhost +local:root >/dev/null 2>&1 || true

docker run --rm -it \
    --net=host \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$REPO/run_logs:/root/run/logs" \
    -v "$REPO/drone_system/drone_system:/root/ros2_ws/src/drone_system/drone_system" \
    -v "$REPO/drone_system/launch:/root/ros2_ws/src/drone_system/launch" \
    -v "$REPO/drone_system/config:/root/ros2_ws/src/drone_system/config" \
    -v "$REPO/tools:/root/tools" \
    -v "$REPO/scripts:/root/scripts" \
    "$IMAGE" bash -lc \
      "cd /root/run && ros2 launch drone_system full_stack.launch.py headless:=${HEADLESS}"
