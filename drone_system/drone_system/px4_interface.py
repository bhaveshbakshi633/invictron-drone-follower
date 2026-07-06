"""px4_interface -- the only node that talks to PX4.

Consumes /drone/waypoint (ENU, from the follower) and drives the PX4 SITL flight
stack over the uXRCE-DDS bridge (px4_msgs):

    arm  ->  takeoff to takeoff_altitude_m  ->  stream OFFBOARD position setpoints

All PX4-specific coupling (topics, NED frame, offboard handshake) lives HERE, so
the follower stays a clean ENU waypoint producer.

Failure handling:
  * PX4 fails to arm  -> retry arm_max_retries times, then log ERROR and shut down
                         the whole launch cleanly (disarm + rclpy shutdown).

Frames: ROS is ENU (x=E, y=N, z=Up); PX4 is NED (x=N, y=E, z=Down).
    x_ned = y_enu,  y_ned = x_enu,  z_ned = -z_enu
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)

from .logutil import get_event_logger

# PX4 enum values (kept explicit so we don't depend on constant names that have
# drifted across px4_msgs releases).
ARMING_STATE_ARMED = 2       # VehicleStatus.ARMING_STATE_ARMED
NAV_STATE_OFFBOARD = 14      # VehicleStatus.NAVIGATION_STATE_OFFBOARD
CMD_ARM_DISARM = 400         # VEHICLE_CMD_COMPONENT_ARM_DISARM
CMD_DO_SET_MODE = 176        # VEHICLE_CMD_DO_SET_MODE


def enu_to_ned(x, y, z):
    return (y, x, -z)


class Px4Interface(Node):
    # High-level state machine
    INIT, ARMING, TAKEOFF, FOLLOW, SHUTDOWN = range(5)

    def __init__(self):
        super().__init__("px4_interface")

        self.declare_parameter("takeoff_altitude_m", 20.0)
        self.declare_parameter("arm_max_retries", 3)
        self.declare_parameter("arm_retry_delay_s", 2.0)
        self.declare_parameter("offboard_rate_hz", 20.0)
        self.declare_parameter("min_altitude_check_m", 1.0)
        self.declare_parameter("log_dir", "logs")

        self.takeoff_alt = float(self.get_parameter("takeoff_altitude_m").value)
        self.max_retries = int(self.get_parameter("arm_max_retries").value)
        self.retry_delay = float(self.get_parameter("arm_retry_delay_s").value)
        rate = float(self.get_parameter("offboard_rate_hz").value)
        self.rate = rate
        self._warmup_limit = int(rate * 60)  # give PX4 ~60 s to become arm-ready
        log_dir = self.get_parameter("log_dir").value
        self.log = get_event_logger("px4_interface", log_dir)

        # PX4 out topics are BEST_EFFORT / VOLATILE -- match or we get nothing.
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub_ocm = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10)

        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self._on_status, px4_qos)
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position",
            self._on_local_pos, px4_qos)
        self.create_subscription(
            PoseStamped, "/drone/waypoint", self._on_waypoint, 10)

        # State
        self.state = self.INIT
        self.arming_state = 0
        self.nav_state = 0
        self.preflight_ok = False
        self.cur_ned = None                 # (n, e, d) current position
        self.target_ned = (0.0, 0.0, -self.takeoff_alt)   # climb straight up
        self.target_yaw = 0.0
        self.target_vel = (0.0, 0.0, 0.0)   # setpoint velocity feed-forward (NED)
        self._prev_target = None            # finite-diff state for the waypoint stream
        self._prev_target_t = None
        self.setpoint_count = 0
        self.arm_attempts = 0
        self._last_arm_action = None        # sim-time of last arm/mode command

        self.dt = 1.0 / rate
        self.create_timer(self.dt, self._loop)
        self.log.info(
            f"started takeoff_alt={self.takeoff_alt}m arm_retries={self.max_retries} "
            f"offboard_rate={rate}Hz")

    # -- subscriptions --------------------------------------------------------
    def _on_status(self, msg: VehicleStatus):
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state
        self.preflight_ok = bool(getattr(msg, "pre_flight_checks_pass", True))

    def _on_local_pos(self, msg: VehicleLocalPosition):
        if msg.xy_valid or msg.z_valid:
            self.cur_ned = (msg.x, msg.y, msg.z)

    def _on_waypoint(self, msg: PoseStamped):
        n, e, d = enu_to_ned(
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        if self.state == self.FOLLOW:
            # Finite-difference the moving waypoint to get its velocity, sent as
            # feed-forward in the setpoint (see _send_setpoint).
            t = self._now_s()
            if self._prev_target is not None:
                dt = t - self._prev_target_t
                if dt > 1e-3:
                    a = 0.3   # low-pass so a noisy waypoint doesn't inject jerk
                    self.target_vel = tuple(
                        (1 - a) * pv + a * (c - p) / dt
                        for pv, c, p in zip(self.target_vel, (n, e, d), self._prev_target))
            self._prev_target = (n, e, d)
            self._prev_target_t = t
            self.target_ned = (n, e, d)

    # -- helpers --------------------------------------------------------------
    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _stamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _send_offboard_mode(self):
        m = OffboardControlMode()
        m.timestamp = self._stamp_us()
        m.position = True
        self.pub_ocm.publish(m)

    def _send_setpoint(self):
        sp = TrajectorySetpoint()
        sp.timestamp = self._stamp_us()
        nan = float("nan")
        sp.position = [float(self.target_ned[0]),
                       float(self.target_ned[1]),
                       float(self.target_ned[2])]
        # Velocity FEED-FORWARD (root fix for the tracking lag): the setpoint's
        # own velocity, finite-diffed from the moving waypoint stream, so PX4's
        # position controller LEADS the moving target instead of trailing it --
        # fixing the lag at the source (position-only setpoints) rather than
        # compensating it upstream. Zero during the straight-up climb.
        sp.velocity = [float(self.target_vel[0]), float(self.target_vel[1]),
                       float(self.target_vel[2])]
        sp.acceleration = [nan, nan, nan]
        sp.jerk = [nan, nan, nan]
        sp.yaw = float(self.target_yaw)
        self.pub_sp.publish(sp)

    def _send_cmd(self, command, p1=0.0, p2=0.0):
        cmd = VehicleCommand()
        cmd.timestamp = self._stamp_us()
        cmd.command = command
        cmd.param1 = float(p1)
        cmd.param2 = float(p2)
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        self.pub_cmd.publish(cmd)

    def _altitude(self):
        return -self.cur_ned[2] if self.cur_ned is not None else 0.0

    # -- main 20 Hz loop ------------------------------------------------------
    def _loop(self):
        if self.state == self.SHUTDOWN:
            return

        # OFFBOARD requires a continuous setpoint stream at all times.
        self._send_offboard_mode()
        self._send_setpoint()
        self.setpoint_count += 1

        if self.state == self.INIT:
            # Do NOT burn our 3 arm attempts during EKF/GPS warm-up. Wait until
            # PX4 has buffered setpoints, a valid position, and pre-arm checks
            # pass -- THEN start the (spec) 3-attempt arm sequence.
            ready = (self.setpoint_count >= 40
                     and self.cur_ned is not None
                     and self.preflight_ok)
            if ready:
                self.log.info("px4_ready action=begin_arming")
                self._attempt_arm()
                self.state = self.ARMING
            elif self.setpoint_count > self._warmup_limit:
                self.log.error(
                    "PX4 never became ready to arm (pre-arm checks never "
                    "passed), shutting down cleanly")
                self._clean_shutdown()
            elif self.setpoint_count % 40 == 0:
                self.log.info(
                    f"waiting_for_px4 preflight={self.preflight_ok} "
                    f"pos_valid={self.cur_ned is not None} "
                    f"elapsed_s={self.setpoint_count / self.rate:.0f}")
            return

        if self.state == self.ARMING:
            if self.arming_state == ARMING_STATE_ARMED:
                self.log.info(f"armed=true attempt={self.arm_attempts}")
                self.state = self.TAKEOFF
                return
            # Not armed yet -- retry after the delay elapses.
            if (self._now_s() - self._last_arm_action) >= self.retry_delay:
                if self.arm_attempts >= self.max_retries:
                    self.log.error(
                        f"arm failed after {self.arm_attempts} attempts, "
                        f"shutting down cleanly")
                    self._clean_shutdown()
                    return
                self._attempt_arm()
            return

        if self.state == self.TAKEOFF:
            if self._altitude() >= (self.takeoff_alt - 0.5):
                self.log.info(f"takeoff_complete altitude_m={self._altitude():.1f}")
                self.state = self.FOLLOW
            return

        # self.state == FOLLOW: target_ned is driven by /drone/waypoint callback.

    def _attempt_arm(self):
        self.arm_attempts += 1
        # Set mode OFFBOARD (base_mode=1, custom_main_mode=6) then arm.
        self._send_cmd(CMD_DO_SET_MODE, p1=1.0, p2=6.0)
        self._send_cmd(CMD_ARM_DISARM, p1=1.0)
        self._last_arm_action = self._now_s()
        self.log.info(f"arm_attempt={self.arm_attempts}/{self.max_retries}")

    def _clean_shutdown(self):
        self.state = self.SHUTDOWN
        self._send_cmd(CMD_ARM_DISARM, p1=0.0)  # best-effort disarm
        self.log.info("clean_shutdown initiated")
        # Tear down the process so the launch's on_exit can stop the rest.
        raise SystemExit(1)


def main(args=None):
    rclpy.init(args=args)
    node = Px4Interface()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
