#!/bin/bash
# Demonstrate the failure-handling paths on a LIVE run, on demand.
#
# The clean 60 s CI run never trips them (no gap, no jump, arm succeeds first try,
# RTF stays healthy) — so the recovery code is real + unit-tested but not *shown*
# in a nominal log. This triggers each failure against a running stack so you can
# watch the recovery happen (and capture it in events.log).
#
# Usage (with the full stack already running, e.g. run_ci.sh or full_stack.launch):
#   scripts/demo_failures.sh car_gap     # kill /car/position -> drone hovers
#   scripts/demo_failures.sh jump        # inject a >5 m teleport -> discard + hold
#   scripts/demo_failures.sh rtf         # how to force RTF < 0.8
#   scripts/demo_failures.sh arm         # how to force an arm failure
#
# After triggering, inspect:  python3 tools/log_summary.py <logdir>/events.log
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null || true
source /root/ros2_ws/install/setup.bash 2>/dev/null || true

case "${1:-}" in
  car_gap)
    echo "[demo] killing car_sim -> /car/position stops -> follower should hover + WARN"
    pkill -f "car_sim" && echo "[demo] car_sim killed. Watch events.log for:"
    echo "        WARNING | follower | car_gap_ms=... action=hover"
    echo "[demo] restart it to show recovery:  ros2 run drone_system car_sim"
    ;;
  jump)
    echo "[demo] injecting a >5 m teleport on /car/position -> follower should DISCARD + hold"
    ros2 topic pub --once /car/position geometry_msgs/msg/PoseStamped \
      '{header: {frame_id: "map"}, pose: {position: {x: 200.0, y: 200.0, z: 0.0}, orientation: {w: 1.0}}}'
    echo "[demo] done. Watch events.log for:"
    echo "        WARNING | follower | jump_rejected delta_m=... action=discard_hold_last_valid"
    ;;
  rtf)
    secs="${2:-20}"
    echo "[demo] pegging all ${secs}s CPUs to force RTF < 0.8 (health_monitor warns every 5 s)"
    for _ in $(seq "$(nproc)"); do yes > /dev/null & done
    sleep "$secs"
    pkill -x yes 2>/dev/null || true
    echo "[demo] load released -> RTF should recover. Watch events.log for:"
    echo "        WARNING | health_monitor | rtf=0.xx below 0.8 ...  (every 5 s), then recovery"
    ;;
  arm)
    echo "[demo] Arming is a one-time startup event, so this is a RELAUNCH demo."
    echo "  px4_interface's 'force_arm_fail_n' param withholds the arm command for the"
    echo "  first N attempts, so PX4 never arms and the retry -> clean-shutdown fires."
    echo "  Set it >= 4 (1 initial + arm_max_retries=3) in config/params.yaml, then relaunch:"
    echo ""
    echo "     # config/params.yaml -> px4_interface: force_arm_fail_n: 5"
    echo "     ros2 launch drone_system full_stack.launch.py"
    echo ""
    echo "  Watch events.log for 4x 'arm_attempt=.. FAULT_INJECTED=withhold_arm' then:"
    echo "        ERROR | px4_interface | PX4 failed to arm after 4 attempts (1 initial + 3 retries)..."
    ;;
  *)
    echo "usage: $0 {car_gap|jump|rtf|arm}"; exit 2 ;;
esac
