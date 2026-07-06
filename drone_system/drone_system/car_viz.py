"""car_viz -- OPTIONAL visible box for the car in the Gazebo GUI (demo only).

The car's authoritative position is /car/position (published by car_sim). This node
adds NOTHING the follower may use -- it only SPAWNS a box in Gazebo and moves it to
match /car/position so the GUI actually shows a car. The follower still consumes only
the topic, so the "no Gazebo ground-truth" constraint stays intact.

Version-robust via the `gz` service CLI. Spawn is one-time (blocking + fully logged so
any error is visible); pose updates are fire-and-forget (never stall the node / RTF).
Params: world, model_name, update_rate_hz, box_size_m, gz_bin.

Run standalone next to a live stack (nothing else needs relaunching):
    ros2 run drone_system car_viz --ros-args -p world:=default
"""

import os
import subprocess
import tempfile

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

SDF = """<?xml version='1.0'?>
<sdf version='1.8'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <visual name='v'>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <ambient>0.9 0.15 0.1 1</ambient><diffuse>0.9 0.15 0.1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class CarViz(Node):
    def __init__(self):
        super().__init__("car_viz")
        self.declare_parameter("world", "default")
        self.declare_parameter("model_name", "car_box")
        self.declare_parameter("update_rate_hz", 10.0)
        self.declare_parameter("box_size_m", [2.0, 1.0, 0.6])
        self.declare_parameter("gz_bin", "gz")

        self.world = self.get_parameter("world").value
        self.name = self.get_parameter("model_name").value
        rate = float(self.get_parameter("update_rate_hz").value)
        sz = list(self.get_parameter("box_size_m").value)
        self.sx, self.sy, self.sz = float(sz[0]), float(sz[1]), float(sz[2])
        self.gz = self.get_parameter("gz_bin").value

        self.sdf_path = os.path.join(tempfile.gettempdir(), f"{self.name}.sdf")
        with open(self.sdf_path, "w") as f:
            f.write(SDF.format(name=self.name, sx=self.sx, sy=self.sy, sz=self.sz))

        self.pos = None
        self.spawned = False
        self.create_subscription(PoseStamped, "/car/position", self._on_car, 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"car_viz world={self.world} name={self.name} rate={rate}Hz sdf={self.sdf_path}")

    def _on_car(self, msg):
        self.pos = (msg.pose.position.x, msg.pose.position.y, self.sz / 2.0)

    def _tick(self):
        if self.pos is None:
            return
        x, y, z = self.pos
        if not self.spawned:
            # retry until gz is up and the spawn actually succeeds
            self.spawned = self._spawn(x, y, z)
            return
        self._set_pose(x, y, z)

    def _spawn(self, x, y, z):
        req = (f'sdf_filename: "{self.sdf_path}" name: "{self.name}" '
               f'pose: {{position: {{x: {x:.3f} y: {y:.3f} z: {z:.3f}}}}}')
        cmd = [self.gz, "service", "-s", f"/world/{self.world}/create",
               "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
               "--timeout", "3000", "--req", req]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=6)
            ok = (r.returncode == 0 and "true" in r.stdout.lower())
            if ok:
                self.get_logger().info(f"car box spawned in world '{self.world}'")
            else:
                self.get_logger().warn(
                    f"spawn not ready (gz booting?) rc={r.returncode} out={r.stdout.strip()!r}",
                    throttle_duration_sec=5.0)
            return ok
        except Exception as e:
            self.get_logger().warn(f"spawn retry (gz not ready?): {e}",
                                   throttle_duration_sec=5.0)
            return False

    def _set_pose(self, x, y, z):
        req = (f'name: "{self.name}" position: {{x: {x:.3f} y: {y:.3f} z: {z:.3f}}} '
               f'orientation: {{w: 1.0}}')
        cmd = [self.gz, "service", "-s", f"/world/{self.world}/set_pose",
               "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
               "--timeout", "200", "--req", req]
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.get_logger().warn(f"set_pose failed: {e}", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = CarViz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
