import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def foxglove_actions(context):
    enabled = LaunchConfiguration("use_foxglove").perform(context).lower()
    if enabled not in {"1", "true", "yes", "on"}:
        return [LogInfo(msg="Foxglove bridge disabled by use_foxglove=false")]

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as connection:
        connection.settimeout(0.25)
        bridge_is_running = connection.connect_ex(("127.0.0.1", 8765)) == 0

    if bridge_is_running:
        return [LogInfo(msg="Reusing the Foxglove bridge already listening on port 8765")]

    return [
        LogInfo(msg="Starting Foxglove bridge on port 8765"),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{"address": "0.0.0.0", "port": 8765}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("device", default_value="/dev/video0"),
            DeclareLaunchArgument("width", default_value="640"),
            DeclareLaunchArgument("height", default_value="480"),
            DeclareLaunchArgument("framerate", default_value="15.0"),
            DeclareLaunchArgument("processing_scale", default_value="0.5"),
            DeclareLaunchArgument("detection_interval", default_value="2"),
            DeclareLaunchArgument("publish_raw", default_value="false"),
            DeclareLaunchArgument("uart_port", default_value="/dev/ttyAMA0"),
            DeclareLaunchArgument("enable_motors", default_value="false"),
            DeclareLaunchArgument("use_foxglove", default_value="true"),
            Node(
                package="rectangle_vision",
                executable="rectangle_vision_node",
                name="rectangle_vision",
                output="screen",
                parameters=[
                    {
                        "device": LaunchConfiguration("device"),
                        "width": LaunchConfiguration("width"),
                        "height": LaunchConfiguration("height"),
                        "framerate": LaunchConfiguration("framerate"),
                        "processing_scale": LaunchConfiguration("processing_scale"),
                        "detection_interval": LaunchConfiguration("detection_interval"),
                        "publish_raw": LaunchConfiguration("publish_raw"),
                    }
                ],
            ),
            Node(
                package="rectangle_vision",
                executable="gimbal_uart_node",
                name="gimbal_uart",
                output="screen",
                parameters=[
                    {
                        "port": LaunchConfiguration("uart_port"),
                        "image_width": LaunchConfiguration("width"),
                        "image_height": LaunchConfiguration("height"),
                        "enable_motors": LaunchConfiguration("enable_motors"),
                    }
                ],
            ),
            OpaqueFunction(function=foxglove_actions),
        ]
    )
