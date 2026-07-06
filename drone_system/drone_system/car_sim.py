"""car_sim -- a scripted "car" that drives a repeating closed loop.

Publishes only ONE thing the rest of the system is allowed to use:
    /car/position   geometry_msgs/PoseStamped

Deliberately does NOT touch Gazebo ground-truth or model state. This node is the
*source* of the car's motion (it decides where the car goes), so publishing that
same trajectory as the car's "sensed" position is legitimate -- the follower
downstream only ever sees this topic, never Gazebo internals. Set noise_stddev_m
> 0 to make this behave like a noisy real sensor (see ANALYSIS.md Q1).
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from .logutil import get_event_logger


class CarSim(Node):
    def __init__(self):
        super().__init__("car_sim")

        self.declare_parameter("path_type", "figure8")
        self.declare_parameter("path_scale_m", 20.0)
        self.declare_parameter("speed_mps", 2.5)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("noise_stddev_m", 0.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("log_dir", "logs")

        self.path_type = self.get_parameter("path_type").value
        self.scale = float(self.get_parameter("path_scale_m").value)
        self.speed = float(self.get_parameter("speed_mps").value)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.noise = float(self.get_parameter("noise_stddev_m").value)
        self.frame_id = self.get_parameter("frame_id").value
        log_dir = self.get_parameter("log_dir").value

        self.log = get_event_logger("car_sim", log_dir)
        self._rng = __import__("random").Random(0)  # deterministic noise

        # Angular rate so tangential speed ~= speed_mps on the loop.
        self.w = self.speed / max(self.scale, 1e-3)
        self.theta = 0.0

        self.pub = self.create_publisher(PoseStamped, "/car/position", 10)
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self._tick)
        self.log.info(
            f"started path={self.path_type} scale={self.scale}m "
            f"speed={self.speed}mps rate={rate}Hz noise={self.noise}m"
        )

    def _shape(self, t: float):
        """Return (x, y) on the chosen closed loop at parameter t."""
        if self.path_type == "circle":
            return self.scale * math.cos(t), self.scale * math.sin(t)
        # figure-8 (lemniscate of Gerono): closed, self-intersecting loop
        return self.scale * math.sin(t), self.scale * math.sin(t) * math.cos(t)

    def _tick(self):
        self.theta += self.w * self.dt
        x, y = self._shape(self.theta)
        # Heading = tangent to the path (direction of travel), from a tiny look-ahead
        # along the SAME clean trajectory (computed before noise so it never jitters).
        xa, ya = self._shape(self.theta + 1e-3)
        yaw = math.atan2(ya - y, xa - x)
        if self.noise > 0.0:
            x += self._rng.gauss(0.0, self.noise)
            y += self._rng.gauss(0.0, self.noise)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        # yaw-only quaternion so the car faces where it is going (the GUI box uses this;
        # the follower ignores orientation and reads only position).
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CarSim()
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
