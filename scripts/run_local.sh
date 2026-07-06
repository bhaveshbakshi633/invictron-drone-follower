#!/bin/bash
# Local demo run WITH the Gazebo GUI (X11) and a VISIBLE car box. Logs -> ./run_logs.
#   ./scripts/run_local.sh            # GUI + visible car (default)
#   HEADLESS=1 ./scripts/run_local.sh # no GUI (still logs), no car box
#
# STOPPING: press Ctrl-C in this terminal -- it force-removes the container, which
# kills every node + PX4 + Gazebo at once (robust even if ros2 launch's own teardown
# stalls on the stubborn PX4/gz processes). From another terminal you can also run:
#     ./scripts/stop.sh          (or:  docker rm -f drone_gui)
#
# Mounts the repo source into the image so it always runs the CURRENT code (no
# rebuild needed after editing a node / launch / param).
REPO="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE="${IMAGE:-ghcr.io/bhaveshbakshi633/drone_system:latest}"
HEADLESS="${HEADLESS:-0}"
NAME="${CONTAINER_NAME:-drone_gui}"
# Show the visible car box whenever the GUI is up (headless=0). Override with CAR_VIZ.
CAR_VIZ="${CAR_VIZ:-$([ "$HEADLESS" = "0" ] && echo 1 || echo 0)}"

# If the docker group isn't active in this shell yet, re-exec through it.
if ! docker info >/dev/null 2>&1 && [ -z "${_SG_WRAP:-}" ]; then
    export _SG_WRAP=1
    exec sg docker -c "bash '$REPO/scripts/run_local.sh' $*"
fi

# Get the image if it isn't local yet: pull the pre-built one, else source-build.
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    docker pull "$IMAGE" 2>/dev/null || {
        IMAGE=drone_system:latest
        docker image inspect "$IMAGE" >/dev/null 2>&1 || "$REPO/scripts/build_image.sh"
    }
fi

mkdir -p "$REPO/run_logs"
xhost +local:root >/dev/null 2>&1 || true

# --- clean shutdown ---------------------------------------------------------
# Force-remove the container on Ctrl-C / exit. Because the whole stack lives inside
# the container, `docker rm -f` kills every process (nodes + PX4 + gz + agent) at
# once -- this is what makes Ctrl-C actually work here.
docker rm -f "$NAME" >/dev/null 2>&1 || true
_stopped=0
stop() { [ "$_stopped" = 1 ] && return 0; _stopped=1
         echo; echo "[run_local] stopping '$NAME' ..."; docker rm -f "$NAME" >/dev/null 2>&1 || true;
         xhost -local:root >/dev/null 2>&1 || true; }
trap 'stop; exit 0' INT TERM
trap stop EXIT

echo "[run_local] starting GUI demo (headless=$HEADLESS car_viz=$CAR_VIZ, container '$NAME')."
echo "[run_local] >>> press Ctrl-C in THIS terminal to stop everything <<<"
docker run --rm -t --name "$NAME" \
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
      "cd /root/run && exec ros2 launch drone_system full_stack.launch.py headless:=${HEADLESS} car_viz:=${CAR_VIZ}" &
CID=$!
wait "$CID"   # Ctrl-C interrupts this wait -> the INT trap force-removes the container
