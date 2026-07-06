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
    echo "[demo] RTF < 0.8 fires when the sim can't keep real-time. To force it:"
    echo "  * software rendering:   LIBGL_ALWAYS_SOFTWARE=1 before launching Gazebo, or"
    echo "  * CPU load:             run 'stress -c \$(nproc)' in the container during the run"
    echo "  Watch events.log every 5 s for:"
    echo "        WARNING | health_monitor | rtf=0.xx below 0.8 ..."
    ;;
  arm)
    echo "[demo] Arm-retry (3x then clean shutdown) is hard to force in clean SITL."
    echo "  To exercise it: block arming so pre-flight checks never pass, e.g. set an"
    echo "  impossible arm gate, or kill the PX4 EKF feed so arming_state never reaches"
    echo "  ARMED. Watch events.log for 3x 'arm_attempt' then 'arm_failed action=shutdown'."
    echo "  (Also covered directly by the unit-tested arm-retry state machine.)"
    ;;
  *)
    echo "usage: $0 {car_gap|jump|rtf|arm}"; exit 2 ;;
esac
