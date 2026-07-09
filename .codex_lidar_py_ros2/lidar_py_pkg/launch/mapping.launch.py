import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def maybe_start_slam(context, *args, **kwargs):
    pkg_share = get_package_share_directory("lidar_py_pkg")
    use_slam = LaunchConfiguration("use_slam").perform(context).lower()
    if use_slam not in ("1", "true", "yes", "on"):
        rviz_config = os.path.join(pkg_share, "rviz", "lidar_view.rviz")
        return [
            LogInfo(msg="SLAM disabled; showing lidar in RViz only."),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]

    try:
        get_package_share_directory("slam_toolbox")
    except PackageNotFoundError:
        rviz_config = os.path.join(pkg_share, "rviz", "lidar_view.rviz")
        return [
            LogInfo(
                msg=(
                    "slam_toolbox is not installed in this ROS2 environment. "
                    "Lidar and RViz will still start; install slam_toolbox to build maps."
                )
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]

    slam_params = os.path.join(pkg_share, "config", "slam_toolbox_mapping.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "mapping.rviz")
    return [
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument("packet_format", default_value="ros2-3B"),
            DeclareLaunchArgument("laser_frame", default_value="laser"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("use_slam", default_value="true"),
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
                        "frame_id": LaunchConfiguration("laser_frame"),
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser_tf",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    LaunchConfiguration("base_frame"),
                    LaunchConfiguration("laser_frame"),
                ],
            ),
            OpaqueFunction(function=maybe_start_slam),
        ]
    )
