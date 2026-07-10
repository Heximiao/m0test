#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
echo "ROS_DISTRO=${ROS_DISTRO:-unknown}"
echo "=== apt candidates ==="
apt-cache search '^ros-jazzy-' | grep -Ei 'slam|toolbox|nav2|navigation|map-server|amcl|robot-localization|joint-state|xacro|tf2|laser|cartographer|mapper|localization|teleop|twist|rviz' | sort || true
echo "=== installed ros packages ==="
ros2 pkg list | grep -E '^(rviz2|tf2_ros|robot_state_publisher|joint_state_publisher|xacro|nav2_map_server|nav2_amcl|nav2_lifecycle_manager|slam_toolbox|robot_localization|teleop_twist_keyboard|teleop_twist_joy|laser_geometry)$' || true
