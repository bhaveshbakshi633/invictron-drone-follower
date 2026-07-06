"""health_monitor -- watches the Gazebo real-time factor (RTF).

RTF = (advance in simulated time) / (advance in wall-clock time). We measure it
WITHOUT any Gazebo-specific API and WITHOUT /clock (PX4's uXRCE-DDS bridge does
not publish one). Instead we read the simulation timestamp that PX4 SITL stamps
onto every message it emits: PX4 runs in *lockstep* with Gazebo, so that clock
advances at exactly the Gazebo sim rate. Comparing its progression against a real
wall clock therefore yields the true real-time factor -- if Gazebo bogs down, PX4
sim time falls behind wall time and RTF drops below 1.0.

  * subscribes /fmu/out/vehicle_local_position (BEST_EFFORT) purely for its
    ``.timestamp`` field (microseconds of PX4/Gazebo sim time)
  * publishes /health/rtf (std_msgs/Float32) for telemetry_logger to record
  * while RTF < rtf_min, logs a WARNING every rtf_warn_period_s until it recovers

Sampling runs on a wall-clock thread on purpose: a ROS-time timer would itself
stall exactly when the sim slows down, which is the moment we need to report.
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32
from px4_msgs.msg import VehicleLocalPosition

from .logutil import get_event_logger


class HealthMonitor(Node):
    def __init__(self):
        super().__init__("health_monitor")

        self.declare_parameter("rtf_min", 0.8)
        self.declare_parameter("rtf_warn_period_s", 5.0)
        self.declare_parameter("rtf_sample_period_s", 1.0)
        self.declare_parameter("log_dir", "logs")

        self.rtf_min = float(self.get_parameter("rtf_min").value)
        self.warn_period = float(self.get_parameter("rtf_warn_period_s").value)
        self.sample_period = float(self.get_parameter("rtf_sample_period_s").value)
        log_dir = self.get_parameter("log_dir").value
        self.log = get_event_logger("health_monitor", log_dir)

        self.pub = self.create_publisher(Float32, "/health/rtf", 10)

        # PX4 out topics are BEST_EFFORT / VOLATILE -- match or we get nothing.
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position",
            self._on_local_pos, px4_qos)

        # Latest PX4 sim-time stamp (microseconds). Written in the ROS executor
        # thread, read in the sampling thread; a plain int assignment is atomic
        # under the GIL, so no lock is needed.
        self._sim_us = 0

        self._last_warn_wall = 0.0
        self._stop = False
        self._thread = threading.Thread(target=self._sample_loop, daemon=True)
        self._thread.start()
        self.log.info(
            f"started rtf_min={self.rtf_min} warn_period={self.warn_period}s "
            f"sample_period={self.sample_period}s")

    def _on_local_pos(self, msg: VehicleLocalPosition):
        # msg.timestamp is PX4/Gazebo sim time in microseconds (lockstep clock).
        self._sim_us = int(msg.timestamp)

    def _sim_time_s(self):
        return self._sim_us / 1e6

    def _sample_loop(self):
        # Wait for the PX4 sim clock to start ticking (first message in).
        while not self._stop and self._sim_us == 0:
            time.sleep(0.1)
        if self._stop:
            return
        prev_wall = time.monotonic()
        prev_sim = self._sim_time_s()

        while not self._stop and rclpy.ok():
            time.sleep(self.sample_period)
            now_wall = time.monotonic()
            now_sim = self._sim_time_s()
            d_wall = now_wall - prev_wall
            d_sim = now_sim - prev_sim
            prev_wall, prev_sim = now_wall, now_sim
            if d_wall <= 0:
                continue
            if d_sim < 0:            # sim clock reset -- skip this sample
                continue
            rtf = d_sim / d_wall

            self.pub.publish(Float32(data=float(rtf)))

            if rtf < self.rtf_min:
                if (now_wall - self._last_warn_wall) >= self.warn_period:
                    self.log.warning(
                        f"Gazebo is running slow: real-time factor {rtf:.2f} is below "
                        f"the {self.rtf_min} minimum; simulation is degraded. "
                        f"rtf={rtf:.2f} min={self.rtf_min} sim_running_slow")
                    self._last_warn_wall = now_wall

    def destroy_node(self):
        self._stop = True
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()
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
