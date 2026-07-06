import os
from glob import glob

from setuptools import setup

package_name = "drone_system"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Make `ros2 launch drone_system ...` and the params file discoverable.
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Bhavesh Bakshi",
    maintainer_email="bhaveshbakshi633@gmail.com",
    description="Simulated PX4/Gazebo drone that follows a simulated car.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "car_sim = drone_system.car_sim:main",
            "car_viz = drone_system.car_viz:main",
            "follower = drone_system.follower:main",
            "px4_interface = drone_system.px4_interface:main",
            "health_monitor = drone_system.health_monitor:main",
            "telemetry_logger = drone_system.telemetry_logger:main",
        ],
    },
)
