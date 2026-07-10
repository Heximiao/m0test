import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def maybe_start_slam(context, *args, **kwargs):
    pkg_share = get_package_share_directory("lidar_py_pkg")
    use_slam = LaunchConfiguration("use_slam").perform(context).lower()
    use_rviz = LaunchConfiguration("use_rviz")
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
                condition=IfCondition(use_rviz),
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
                condition=IfCondition(use_rviz),
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
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_slam",
            output="screen",
            parameters=[
                {
                    "autostart": True,
                    "node_names": ["slam_toolbox"],
                }
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument("odom_port", default_value="/dev/ttyAMA0"),
            DeclareLaunchArgument("odom_baudrate", default_value="115200"),
            DeclareLaunchArgument("packet_format", default_value="ros2-3B"),
            DeclareLaunchArgument("laser_frame", default_value="laser"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("use_odom", default_value="true"),
            DeclareLaunchArgument("use_slam", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
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
                package="lidar_py_pkg",
                executable="wheel_odom_node",
                name="wheel_odom_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_odom")),
                parameters=[
                    {
                        "port": LaunchConfiguration("odom_port"),
                        "baudrate": LaunchConfiguration("odom_baudrate"),
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "base_frame": LaunchConfiguration("base_frame"),
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
