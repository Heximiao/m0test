#!/usr/bin/env bash
set -eo pipefail

cd /home/ubuntu/hexi/ros2/lidar
source /opt/ros/lyrical/setup.bash
source install/setup.bash

LOG=/tmp/lidar_py_node.log
: > "$LOG"

ros2 launch lidar_py_pkg lidar_py.launch.py port:=/dev/ttyACM0 baudrate:=115200 packet_format:=ros2-3B >"$LOG" 2>&1 &
NODE_PID=$!

cleanup() {
  kill "$NODE_PID" >/dev/null 2>&1 || true
  wait "$NODE_PID" >/dev/null 2>&1 || true
}
trap cleanup EXIT

sleep 4
echo "=== topics ==="
ros2 topic list | sort

echo "=== one /scan sample ==="
set +e
timeout 8 ros2 topic echo --once /scan sensor_msgs/msg/LaserScan
ECHO_STATUS=$?
set -e

echo "=== node log ==="
cat "$LOG"

exit "$ECHO_STATUS"
