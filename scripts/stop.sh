#!/bin/bash
# Force-stop the drone system from ANY terminal -- the reliable "off switch".
#
#   ./scripts/stop.sh
#
# It removes the demo containers (which kills every node + PX4 + Gazebo + agent
# inside them at once) and, for a native (non-Docker) run, signals the stray
# processes directly. Safe to run even if nothing is running.

# 1) Docker demo containers (run_local.sh / run.sh use these names).
for c in drone_gui drone_ci drone_demo; do
    docker rm -f "$c" >/dev/null 2>&1 && echo "[stop] removed container '$c'"
done

# 2) Native run (ros2 launch on the host): signal the stack's processes by name.
#    SIGINT first (graceful), then SIGKILL anything that ignores it.
pat='drone_system/lib/drone_system|full_stack.launch|px4_sitl|gz sim|MicroXRCEAgent|micro_ros'
if pgrep -f "$pat" >/dev/null 2>&1; then
    pkill -INT  -f "$pat" 2>/dev/null || true
    sleep 2
    pkill -KILL -f "$pat" 2>/dev/null || true
    echo "[stop] signalled native ros2/px4/gz processes"
fi

echo "[stop] done."
