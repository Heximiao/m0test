import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("lidar_py_pkg")
    rviz_config = os.path.join(pkg_share, "rviz", "lidar_view.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument("packet_format", default_value="ros2-3B"),
            DeclareLaunchArgument("frame_id", default_value="laser"),
            Node(
                package="lidar_py_pkg",
                executable="lidar_py_node",
                name="lidar_py_node",
                output="screen",
                parameters=[
                    {
                        "port": LaunchConfiguration("port"),
                        "baudrate": LaunchConfiguration("baudrate"),
                        "packet_format": LaunchConfiguration("packet_format"),
                        "frame_id": LaunchConfiguration("frame_id"),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
