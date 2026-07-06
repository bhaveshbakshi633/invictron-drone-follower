#!/bin/bash
# One command from a fresh clone.
#
#   ./scripts/run.sh          build the image if needed, then run the 60-second
#                             headless integration test + pass/fail gate. Works
#                             on ANY Linux host with Docker -- no X server.
#   GUI=1 ./scripts/run.sh    instead launch the Gazebo GUI demo (needs a Linux
#                             desktop with an X11 display).
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
# Prefer the pre-built image from GHCR (no PX4 compile). Override with IMAGE=<tag>,
# or set IMAGE=drone_system:latest to force a local source build.
PREBUILT="ghcr.io/bhaveshbakshi633/drone_system:latest"
export IMAGE="${IMAGE:-$PREBUILT}"   # exported so run_local.sh (GUI path) reuses it

if ! docker info >/dev/null 2>&1; then
    echo "[run] cannot talk to Docker. Install Docker Engine and either add your" >&2
    echo "      user to the 'docker' group (newgrp docker) or run with sudo." >&2
    exit 1
fi

# Get the image: use it if already local, else PULL the pre-built one, else BUILD.
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "[run] fetching pre-built image '$IMAGE' (no compile needed)…"
    if ! docker pull "$IMAGE"; then
        echo "[run] pre-built image unavailable -- building from source" \
             "(first build compiles PX4, ~20-30 min)…"
        IMAGE=drone_system:latest
        docker image inspect "$IMAGE" >/dev/null 2>&1 || "$REPO/scripts/build_image.sh"
    fi
fi

if [ "${GUI:-0}" = "1" ]; then
    exec "$REPO/scripts/run_local.sh"
fi

echo "[run] 60-second headless integration test (no X server needed)…"
mkdir -p "$REPO/run_logs"
exec docker run --rm -v "$REPO/run_logs:/root/run/logs" "$IMAGE" bash /root/scripts/run_ci.sh
