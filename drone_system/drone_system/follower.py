"""follower -- the node this assignment is really about.

It sits BETWEEN two topics and owns the following policy:

    in:   /car/position   (geometry_msgs/PoseStamped)  -- the only car info it may use
    out:  /drone/waypoint (geometry_msgs/PoseStamped)  -- where the drone should go

Policy: trail the car by `offset_distance_m` metres, along the car's direction of
travel, at `follow_altitude_m` metres altitude, emitted at a steady 50 Hz so the
PX4 OFFBOARD interface downstream always has a fresh setpoint.

Failure handling (every threshold comes from config/params.yaml):
  * /car/position gap > car_timeout_ms      -> hover in place, WARNING w/ timestamp
  * single-step jump > jump_threshold_m     -> discard sample, hold last valid, WARN
It never reads Gazebo ground-truth -- only the /car/position topic.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from .logutil import get_event_logger


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class Follower(Node):
    def __init__(self):
        super().__init__("follower")

        self.declare_parameter("offset_distance_m", 5.0)
        self.declare_parameter("follow_altitude_m", 20.0)
        self.declare_parameter("car_timeout_ms", 200)
        self.declare_parameter("jump_threshold_m", 5.0)
        self.declare_parameter("update_rate_hz", 50.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("log_dir", "logs")

        self.offset = float(self.get_parameter("offset_distance_m").value)
        self.altitude = float(self.get_parameter("follow_altitude_m").value)
        self.timeout_ms = float(self.get_parameter("car_timeout_ms").value)
        self.jump_m = float(self.get_parameter("jump_threshold_m").value)
        rate = float(self.get_parameter("update_rate_hz").value)
        self.frame_id = self.get_parameter("frame_id").value
        log_dir = self.get_parameter("log_dir").value
        self.log = get_event_logger("follower", log_dir)

        # State
        self.last_valid = None       # (x, y) last accepted car position
        self.last_recv = None        # rclpy Time of last ACCEPTED sample
        self.heading = None          # (hx, hy) smoothed unit travel direction
        self.waypoint = None         # (x, y, z) last commanded setpoint
        self.hovering = False

        self.sub = self.create_subscription(
            PoseStamped, "/car/position", self._on_car, 10)
        self.pub = self.create_publisher(PoseStamped, "/drone/waypoint", 10)
        self.create_timer(1.0 / rate, self._on_timer)
        self.log.info(
            f"started offset={self.offset}m alt={self.altitude}m "
            f"timeout={self.timeout_ms}ms jump_gate={self.jump_m}m rate={rate}Hz"
        )

    # -- inbound car position -------------------------------------------------
    def _on_car(self, msg: PoseStamped):
        x, y = msg.pose.position.x, msg.pose.position.y
        now = self.get_clock().now()

        # Re-acquisition after a dropout: if /car/position has been silent longer
        # than the hover timeout, the car has legitimately travelled far, so the
        # single-step jump gate must NOT apply to the first sample back -- else
        # every recovered sample is rejected against a stale last_valid and the
        # drone latches into permanent hover even though the stream is fine again.
        reacquire = (self.last_recv is not None
                     and (now - self.last_recv).nanoseconds / 1e6 > self.timeout_ms)

        if self.last_valid is not None and not reacquire:
            # Jump rejection during CONTINUOUS tracking: a single implausible leap
            # is discarded and we hold last valid. We intentionally do NOT refresh
            # last_recv on a rejected sample, so a stream of garbage still trips
            # the hover timeout (which then re-acquires via the branch below).
            d = _dist((x, y), self.last_valid)
            if d > self.jump_m:
                self.log.warning(
                    f"jump_rejected delta_m={d:.2f} threshold={self.jump_m} "
                    f"action=discard_hold_last_valid")
                return

            # Update smoothed heading from this valid step.
            vx, vy = x - self.last_valid[0], y - self.last_valid[1]
            n = math.hypot(vx, vy)
            if n > 1e-3:
                hx, hy = vx / n, vy / n
                if self.heading is None:
                    self.heading = (hx, hy)
                else:
                    a = 0.3  # low-pass so heading doesn't jitter on noise
                    mx = (1 - a) * self.heading[0] + a * hx
                    my = (1 - a) * self.heading[1] + a * hy
                    m = math.hypot(mx, my) or 1.0
                    self.heading = (mx / m, my / m)
        elif reacquire:
            # Fresh lock after the gap; the old heading is stale, so drop it and
            # rebuild from the next continuous step.
            self.heading = None
            self.log.info("car_reacquired after gap action=reset_hold")

        self.last_valid = (x, y)
        self.last_recv = now
        if self.hovering:
            self.hovering = False
            self.log.info("car_stream_recovered action=resume_follow")

    # -- steady 50 Hz setpoint emission --------------------------------------
    def _on_timer(self):
        if self.last_recv is None:
            return  # no car fix yet; px4_interface handles pre-flight

        gap_ms = (self.get_clock().now() - self.last_recv).nanoseconds / 1e6
        if gap_ms > self.timeout_ms:
            if not self.hovering:
                self.hovering = True
                self.log.warning(
                    f"car_gap_ms={gap_ms:.0f} threshold={self.timeout_ms} "
                    f"action=hover")
            if self.waypoint is not None:
                self._publish(self.waypoint)  # hold position -> hover
            return

        hx, hy = self.heading if self.heading is not None else (1.0, 0.0)
        tx = self.last_valid[0] - self.offset * hx
        ty = self.last_valid[1] - self.offset * hy
        self.waypoint = (tx, ty, self.altitude)
        self._publish(self.waypoint)

    def _publish(self, wp):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(wp[0])
        msg.pose.position.y = float(wp[1])
        msg.pose.position.z = float(wp[2])
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Follower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
