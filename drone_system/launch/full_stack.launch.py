"""full_stack.launch.py -- ONE command brings up the entire system:

    ros2 launch drone_system full_stack.launch.py

It starts, in order:
  1. micro-XRCE-DDS agent  (PX4 <-> ROS2 bridge on udp:8888)
  2. PX4 SITL + Gazebo      (x500 quadrotor; headless by default for CI)
  3. car_sim, follower, px4_interface, health_monitor, telemetry_logger

Launch arguments:
  headless:=1|0          run Gazebo without a GUI (default 1; set 0 for a demo)
  px4_dir:=<path>        PX4-Autopilot checkout (default $PX4_DIR or /opt/PX4-Autopilot)
  params:=<path>         override the parameter file
  car_viz:=1             spawn a visible car box in the Gazebo GUI (demo; needs headless:=0)

Nodes run on the wall clock (use_sim_time:=false): PX4's uXRCE-DDS bridge
publishes no /clock, so there is no ROS sim-time source; PX4 messages carry their
own sim timestamps where we need them. If px4_interface exits (e.g. an
unrecoverable arm failure), its OnProcessExit handler emits a launch Shutdown that
stops the ROS nodes; the packaged run (`docker run --rm` / run_ci.sh) reaps the
PX4 SITL + Gazebo + agent processes on teardown.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction, EmitEvent,
    RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    LaunchConfiguration, EnvironmentVariable, PythonExpression)
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("drone_system")
    default_params = os.path.join(pkg_share, "config", "params.yaml")

    headless = LaunchConfiguration("headless")
    px4_dir = LaunchConfiguration("px4_dir")
    params = LaunchConfiguration("params")
    # PX4's uXRCE-DDS bridge does NOT publish /clock, so there is no sim-time
    # source for ROS2. Nodes therefore run on wall clock -- the standard PX4+ROS2
    # setup. PX4 messages carry their own sim timestamps where we need them.
    use_sim_time = {"use_sim_time": False}

    def node(exe, name):
        return Node(package="drone_system", executable=exe, name=name,
                    output="screen", parameters=[params, use_sim_time])

    # 1) uXRCE-DDS agent (PX4 <-> ROS2)
    agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888", "-v", "2"],  # -v 2: below default (4), keeps agent logs quiet
        output="screen", name="uxrce_agent")

    # 2) PX4 SITL + Gazebo. ALWAYS boot the gz SERVER headless (HEADLESS=1). If PX4
    #    is told HEADLESS=0 it tries to bring up the gz GUI *itself*, which fails to
    #    get a render context in a container / on a headless host -- and then the gz
    #    server never feeds sensor data, so PX4's pre-arm checks never pass and the
    #    drone never arms. The GUI, when requested (headless:=0), is attached
    #    separately below as a `gz sim -g` client that connects to this server.
    px4 = ExecuteProcess(
        cmd=["bash", "-c",
             'cd "$PX4_DIR" && HEADLESS=1 make px4_sitl gz_x500 < /dev/null'],
        # Spawn the drone JUST ABOVE the ground (z=0.2 m). PX4's gz bridge nudges a
        # model spawned at z<=0 sharply "upwards" (it ends up resting ~1.7 m in the
        # air); spawning at a small positive z avoids that nudge, so the drone sits on
        # the ground and takes off from there.
        additional_env={"PX4_DIR": px4_dir, "PX4_GZ_MODEL_POSE": "0,0,0.2"},
        output="screen", name="px4_sitl")

    # 3) our nodes -- staggered so PX4/agent are up first
    px4_iface = node("px4_interface", "px4_interface")
    our_nodes = TimerAction(period=6.0, actions=[
        node("car_sim", "car_sim"),
        node("follower", "follower"),
        px4_iface,
        node("health_monitor", "health_monitor"),
        node("telemetry_logger", "telemetry_logger"),
        # OPTIONAL visible car box (car_viz:=1, default off -> CI/headless untouched).
        # MUST live in THIS TimerAction -- a separate action later in the
        # LaunchDescription silently never fires. car_viz retries the spawn until gz
        # is up; pure visualization, the follower never reads it.
        Node(package="drone_system", executable="car_viz", name="car_viz",
             output="screen",
             parameters=[{"world": LaunchConfiguration("car_viz_world")}],
             condition=IfCondition(LaunchConfiguration("car_viz"))),
        # GUI client (headless:=0 only): PX4 runs the gz server-only, so nothing
        # opens a window -- attach the GUI here. Wait for the server's world to
        # exist, then run `gz sim -g`. Kept in THIS TimerAction so it actually
        # fires (a separate later action silently never does).
        ExecuteProcess(
            cmd=["bash", "-c",
                 'for i in $(seq 90); do gz service -l 2>/dev/null | '
                 'grep -q "/world/.*/create" && break; sleep 1; done; exec gz sim -g'],
            output="screen", name="gz_gui",
            condition=IfCondition(PythonExpression(["'", headless, "' == '0'"]))),
    ])

    # If px4_interface dies (unrecoverable arm failure), stop the whole launch.
    shutdown_on_iface_exit = RegisterEventHandler(
        OnProcessExit(target_action=px4_iface,
                      on_exit=[EmitEvent(event=Shutdown(reason="px4_interface exited"))]))

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="1"),
        DeclareLaunchArgument(
            "px4_dir",
            default_value=EnvironmentVariable("PX4_DIR",
                                              default_value="/opt/PX4-Autopilot")),
        DeclareLaunchArgument("params", default_value=default_params),
        DeclareLaunchArgument("car_viz", default_value="0"),
        DeclareLaunchArgument("car_viz_world", default_value="default"),
        agent,
        TimerAction(period=2.0, actions=[px4]),
        our_nodes,
        shutdown_on_iface_exit,
    ])
