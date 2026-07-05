#!/bin/bash
# Build the reproducible ROS2 Humble + PX4 SITL image.
set -e
cd "$(dirname "$0")/.."
docker build -f docker/Dockerfile -t drone_system:latest .
echo "built image: drone_system:latest"
