#!/bin/bash
# Headless 60-second integration run + pass/fail gate. Used by CI and by hand.
# Launches the full stack, lets it fly for ~60 s, tears down, then checks:
#   * drone stayed above 1 m, and
#   * no ERROR events in the final 30 s.
# NOTE: no `set -u` -- ROS/ament setup.bash references unbound vars by design.
set -o pipefail

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
export PX4_DIR="${PX4_DIR:-/opt/PX4-Autopilot}"

RUN_SECONDS="${RUN_SECONDS:-60}"
WORK=/root/run
mkdir -p "$WORK/logs"
cd "$WORK"

# Start each run from clean logs so the gate never sees a previous run's data.
rm -f "$WORK/logs/events.log" "$WORK/logs/telemetry.jsonl"

echo "[run_ci] launching full stack headless for ${RUN_SECONDS}s ..."
ros2 launch drone_system full_stack.launch.py headless:=1 \
    > "$WORK/logs/launch.log" 2>&1 &
LAUNCH_PID=$!

sleep "$((RUN_SECONDS + 5))"   # +5s startup slack (agent + PX4 boot)

echo "[run_ci] stopping ..."
# Stop telemetry FIRST, before tearing down the flight. The gate fails on ANY
# airborne sample < 1 m; during teardown PX4 loses its OFFBOARD setpoint stream and
# may descend. Ending telemetry before the flight is torn down means a genuinely
# good run is never failed by the shutdown descent. telemetry.jsonl is line-buffered,
# so no completed row is lost when the writer is stopped abruptly.
pkill -f telemetry_logger 2>/dev/null || true
sleep 1
kill -INT "$LAUNCH_PID" 2>/dev/null || true
sleep 6
kill -KILL "$LAUNCH_PID" 2>/dev/null || true
pkill -f px4 2>/dev/null || true
pkill -f MicroXRCEAgent 2>/dev/null || true

echo "[run_ci] evaluating run ..."
if [ ! -s "$WORK/logs/telemetry.jsonl" ]; then
    echo "[run_ci] FAIL: no telemetry produced — the stack did not fly."
    exit 1
fi
python3 /root/tools/ci_check.py \
    --telemetry "$WORK/logs/telemetry.jsonl" \
    --events "$WORK/logs/events.log" \
    --min-alt 1.0 --window 30
exit $?
