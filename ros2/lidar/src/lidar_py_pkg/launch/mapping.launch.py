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
                    "bond_timeout": 0.0,
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
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("use_foxglove", default_value="true"),
            DeclareLaunchArgument("use_camera", default_value="true"),
            DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
            DeclareLaunchArgument("camera_width", default_value="640"),
            DeclareLaunchArgument("camera_height", default_value="480"),
            DeclareLaunchArgument("camera_framerate", default_value="15.0"),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_foxglove")),
                parameters=[{"address": "0.0.0.0", "port": 8765}],
            ),
            Node(
                package="lidar_py_pkg",
                executable="camera_node",
                name="camera",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_camera")),
                parameters=[
                    {
                        "device": LaunchConfiguration("camera_device"),
                        "width": LaunchConfiguration("camera_width"),
                        "height": LaunchConfiguration("camera_height"),
                        "framerate": LaunchConfiguration("camera_framerate"),
                    }
                ],
            ),
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
                executable="base_driver_node",
                name="base_driver_node",
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
                    "--x", "0.025", "--y", "0.0", "--z", "0.0",
                    "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
                    "--frame-id", LaunchConfiguration("base_frame"),
                    "--child-frame-id", LaunchConfiguration("laser_frame"),
                ],
            ),
            OpaqueFunction(function=maybe_start_slam),
        ]
    )
