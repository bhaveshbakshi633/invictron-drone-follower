#!/bin/bash
# Interactive shell INSIDE the stack's container -- ROS2, PX4, gz and the built
# workspace are already sourced (entrypoint), so the assignment's single command
# works verbatim at the prompt:
#
#     ros2 launch drone_system full_stack.launch.py
#
# Logs land in ./run_logs on the host. Stop a launch with Ctrl-C; if PX4/gz linger
# (known SITL teardown quirk), just exit the shell -- the container dies with it
# (--rm) and everything is reaped. From outside: ./scripts/stop.sh.
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE="${IMAGE:-ghcr.io/bhaveshbakshi633/drone_system:latest}"
NAME="${CONTAINER_NAME:-drone_shell}"

if ! docker info >/dev/null 2>&1 && [ -z "${_SG_WRAP:-}" ]; then
    export _SG_WRAP=1
    exec sg docker -c "bash '$REPO/scripts/shell.sh' $*"
fi
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    docker pull "$IMAGE" 2>/dev/null || {
        IMAGE=drone_system:latest
        docker image inspect "$IMAGE" >/dev/null 2>&1 || "$REPO/scripts/build_image.sh"
    }
fi

mkdir -p "$REPO/run_logs"
docker rm -f "$NAME" >/dev/null 2>&1 || true
echo "[shell] entering the stack container. Try:  ros2 launch drone_system full_stack.launch.py"
exec docker run --rm -it --name "$NAME" \
    -v "$REPO/run_logs:/root/run/logs" \
    -w /root/run \
    "$IMAGE" bash
