"""telemetry_logger -- single writer of logs/telemetry.jsonl (feeds plot_run.py).

Keeping ONE process responsible for the telemetry file guarantees clean,
non-interleaved JSON Lines. It samples the latest values at a fixed rate and
writes one row per tick:

    {"t", "wall", "car_x", "car_y", "drone_x", "drone_y", "drone_z",
     "rtf", "msg_dt_ms"}

Drone position comes from PX4's /fmu/out/vehicle_local_position (NED) and is
converted to the same ENU frame as the car so the XY paths overlay correctly.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from px4_msgs.msg import VehicleLocalPosition

from .logutil import get_event_logger, TelemetryWriter


class TelemetryLogger(Node):
    def __init__(self):
        super().__init__("telemetry_logger")

        self.declare_parameter("sample_rate_hz", 20.0)
        self.declare_parameter("log_dir", "logs")
        rate = float(self.get_parameter("sample_rate_hz").value)
        log_dir = self.get_parameter("log_dir").value

        self.log = get_event_logger("telemetry_logger", log_dir)
        self.tw = TelemetryWriter(log_dir)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Latest values
        self.car = None          # (x, y) ENU
        self.drone = None        # (x, y, z) ENU
        self.rtf = None
        self._last_car_stamp = None
        self.msg_dt_ms = None

        self.create_subscription(PoseStamped, "/car/position", self._on_car, 10)
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position",
            self._on_drone, px4_qos)
        self.create_subscription(Float32, "/health/rtf", self._on_rtf, 10)

        self.create_timer(1.0 / rate, self._on_tick)
        self.log.info(f"started sample_rate={rate}Hz -> {self.tw.path}")

    def _on_car(self, msg: PoseStamped):
        self.car = (msg.pose.position.x, msg.pose.position.y)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_car_stamp is not None:
            self.msg_dt_ms = (stamp - self._last_car_stamp) * 1000.0
        self._last_car_stamp = stamp

    def _on_drone(self, msg: VehicleLocalPosition):
        # NED (n, e, d) -> ENU (x=e, y=n, z=-d)
        self.drone = (msg.y, msg.x, -msg.z)

    def _on_rtf(self, msg: Float32):
        self.rtf = float(msg.data)

    def _on_tick(self):
        if self.car is None and self.drone is None:
            return
        t = self.get_clock().now().nanoseconds / 1e9
        row = {
            "t": round(t, 3),
            "car_x": self.car[0] if self.car else None,
            "car_y": self.car[1] if self.car else None,
            "drone_x": self.drone[0] if self.drone else None,
            "drone_y": self.drone[1] if self.drone else None,
            "drone_z": self.drone[2] if self.drone else None,
            "rtf": self.rtf,
            "msg_dt_ms": round(self.msg_dt_ms, 2) if self.msg_dt_ms is not None else None,
        }
        self.tw.write(**row)

    def destroy_node(self):
        self.tw.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryLogger()
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
