#!/usr/bin/env bash
set -eo pipefail

cd /home/heximiao/hexi/ros2/slam/lidar
source /opt/ros/jazzy/setup.bash

echo "python: $(python3 --version)"
echo "ros dist path: ${AMENT_PREFIX_PATH:-}"
python3 -c 'import rclpy; import sensor_msgs; print("ros python deps ok")'
python3 -c 'import serial; print("pyserial ok")'

colcon build --symlink-install --packages-select lidar_py_pkg
source install/setup.bash
ros2 pkg prefix lidar_py_pkg
ros2 pkg executables lidar_py_pkg
