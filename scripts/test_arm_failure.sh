#!/bin/bash
# Proves the ARM-FAILURE CLEAN-SHUTDOWN path (spec: "PX4 fails to arm -> Retry 3
# times, then shut down cleanly"). We force every arm attempt to be withheld and
# assert the spec's requirements on the node that owns arming:
#   (1) exactly 1 initial + 3 retries = 4 arm attempts are made,
#   (2) an ERROR (with ISO-8601 timestamp) is logged, and
#   (3) px4_interface SHUTS DOWN CLEANLY -- the process exits after logging, with no
#       hang and no crash-loop.
# On that clean exit the launch's OnProcessExit -> Shutdown handler tears the ROS
# nodes down; the PX4 SITL + Gazebo + agent processes are reaped by the caller
# (run_ci.sh, or `docker run --rm` in the packaged path), exactly as in a real run.
#
# The waits are EVENT-DRIVEN (they watch events.log), never time-based, so they can
# never race the launch's staggered node startup.
set -o pipefail
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
export PX4_DIR="${PX4_DIR:-/opt/PX4-Autopilot}"

WORK=/root/run; mkdir -p "$WORK/logs"; cd "$WORK"
rm -f "$WORK/logs/events.log"

# Override ONLY force_arm_fail_n (>=4 withholds every attempt); keep all other params.
SHARE="$(ros2 pkg prefix drone_system)/share/drone_system/config/params.yaml"
PARAMS=/tmp/arm_fail_params.yaml
sed 's/force_arm_fail_n: 0/force_arm_fail_n: 5/' "$SHARE" > "$PARAMS"
grep -q 'force_arm_fail_n: 5' "$PARAMS" || { echo "[arm_test] could not inject fault param"; exit 1; }

echo "[arm_test] launching with forced arm failure (headless) ..."
ros2 launch drone_system full_stack.launch.py headless:=1 params:="$PARAMS" \
    > "$WORK/logs/arm_launch.log" 2>&1 &
LPID=$!

cleanup() {  # tear the rest down the way run_ci.sh / docker --rm would
    kill -INT "$LPID" 2>/dev/null; sleep 3; kill -9 "$LPID" 2>/dev/null
    wait "$LPID" 2>/dev/null || true
    pkill -f px4 2>/dev/null; pkill -f 'gz sim' 2>/dev/null
    pkill -f MicroXRCEAgent 2>/dev/null; pkill -f drone_system 2>/dev/null; true
}
fail() {
    echo "[arm_test] FAIL: $1"
    echo "--- events.log (arm/shutdown) ---"; grep -iE 'arm|shutdown|ready' "$WORK/logs/events.log" 2>/dev/null || echo "(events.log empty)"
    echo "--- arm_launch.log tail ---"; tail -15 "$WORK/logs/arm_launch.log" 2>/dev/null || true
    cleanup; exit 1
}

ERR_RE="PX4 failed to arm after 4 attempts"
IFACE_RE="lib/drone_system/px4_interface"

# EVENT-DRIVEN wait for the arm-failure ERROR. It can only be logged AFTER
# px4_interface starts (launch TimerAction), becomes ready, makes its 4 attempts,
# and decides to shut down -- so this cannot race startup. Budget is generous for a
# slow runner (PX4 boot + EKF warm-up + 4 x retry_delay). We abort early only if the
# launch process itself dies without ever logging the ERROR.
logged=0
for i in $(seq 180); do
    if grep -q "$ERR_RE" "$WORK/logs/events.log" 2>/dev/null; then logged=1; break; fi
    if ! kill -0 "$LPID" 2>/dev/null; then
        sleep 2
        grep -q "$ERR_RE" "$WORK/logs/events.log" 2>/dev/null && logged=1
        break
    fi
    sleep 1
done
[ "$logged" = "1" ] || fail "arm-failure ERROR never logged (px4_interface did not reach the retry->shutdown path within budget)"
echo "[arm_test] arm-failure ERROR logged after ~${i}s"

# (1) exactly 4 attempts, all fault-injected.
ATT=$(grep -c "FAULT_INJECTED=withhold_arm" "$WORK/logs/events.log" 2>/dev/null || echo 0)
[ "$ATT" -eq 4 ] || fail "expected 4 arm attempts (1 initial + 3 retries), found $ATT"

# (2) ERROR with an ISO-8601 timestamp announcing the clean shutdown.
grep -qE '^[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9:.+-]+ \| ERROR \| px4_interface \| PX4 failed to arm after 4 attempts' \
    "$WORK/logs/events.log" || fail "arm-failure ERROR (with ISO timestamp) not found"

# (3) px4_interface shuts down cleanly: the process exits shortly after logging.
dead=0
for i in $(seq 15); do
    pgrep -f "$IFACE_RE" >/dev/null || { dead=1; break; }
    sleep 1
done
[ "$dead" = "1" ] || fail "px4_interface still running 15 s after logging its shutdown -> not a clean exit"

echo "[arm_test] PASS: 4 attempts, ERROR logged with ISO timestamp, px4_interface shut down cleanly"
grep -E "arm_attempt|failed to arm" "$WORK/logs/events.log"

# Informational: whether the launch's OnProcessExit -> Shutdown has already reaped
# the ROS nodes (PX4 SITL/gz can stall the launch's own teardown; the caller reaps
# them regardless -- this line is diagnostics, not a pass/fail gate).
sleep 3
REM=$(pgrep -af 'lib/drone_system/(car_sim|follower|health_monitor|telemetry_logger)' | wc -l)
echo "[arm_test] (info) ROS nodes still up post-shutdown: $REM (reaped by caller/container on teardown)"

cleanup
exit 0
