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
# Fresh logs per demo session -- otherwise events.log accumulates across runs and
# old failure-drill lines show up in a new session's log (confusing to read).
rm -f "$REPO/run_logs/events.log" "$REPO/run_logs/telemetry.jsonl" "$REPO/run_logs/launch.log"
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

# GPU rendering for the gz GUI: on a box with the NVIDIA container toolkit this uses
# the GPU (smooth fps AND leaves the CPU for physics, so PX4 arms). Without it, falls
# back to software rendering -- much slower; the sim RTF can drop to ~0.2, so the drone
# can take up to ~arm_ready_timeout_s to arm. Force with GPU=1 / GPU=0.
# NOTE: we deliberately do NOT use --net=host -- it routes PX4<->agent<->nodes DDS over
# the host interface, where discovery can silently fail and PX4 never becomes ready.
GPU="${GPU:-auto}"
if [ "$GPU" = "auto" ]; then
    if docker run --rm --gpus all "$IMAGE" true >/dev/null 2>&1; then GPU=1; else GPU=0; fi
fi
if [ "$GPU" = "1" ]; then
    RENDER=(--gpus all -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all)
    echo "[run_local] GPU rendering (NVIDIA container toolkit detected)."
else
    RENDER=(-e LIBGL_ALWAYS_SOFTWARE=1)
    echo "[run_local] SOFTWARE rendering (no GPU toolkit) -- GUI will be slow; drone may take ~1 min to arm."
fi

echo "[run_local] starting GUI demo (headless=$HEADLESS car_viz=$CAR_VIZ, container '$NAME')."
echo "[run_local] >>> press Ctrl-C in THIS terminal to stop everything <<<"
docker run --rm -t --name "$NAME" \
    "${RENDER[@]}" \
    -e DISPLAY="${DISPLAY:-:0}" \
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
