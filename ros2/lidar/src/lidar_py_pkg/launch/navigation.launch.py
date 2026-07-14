import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("lidar_py_pkg")
    params_file = os.path.join(package_share, "config", "nav2_params.yaml")
    default_map = "/home/heximiao/hexi/ros2/slam/maps/my_first_map.yaml"

    map_file = LaunchConfiguration("map")
    use_rviz = LaunchConfiguration("use_rviz")
    use_foxglove = LaunchConfiguration("use_foxglove")
    lifecycle_nodes = [
        "map_server",
        "amcl",
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("use_foxglove", default_value="true"),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[{"address": "0.0.0.0", "port": 8765}],
                condition=IfCondition(use_foxglove),
            ),
            Node(
                package="lidar_py_pkg",
                executable="lidar_py_node",
                name="lidar_py_node",
                output="screen",
                parameters=[{"port": "/dev/ttyACM0", "frame_id": "laser"}],
            ),
            Node(
                package="lidar_py_pkg",
                executable="base_driver_node",
                name="base_driver_node",
                output="screen",
                parameters=[{"port": "/dev/ttyAMA0"}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser_tf",
                arguments=[
                    "--x", "0.025", "--y", "0.0", "--z", "0.0",
                    "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
                    "--frame-id", "base_link", "--child-frame-id", "laser",
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[params_file, {"yaml_filename": map_file}],
            ),
            Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen", parameters=[params_file]),
            Node(package="nav2_controller", executable="controller_server", name="controller_server", output="screen", parameters=[params_file]),
            Node(package="nav2_smoother", executable="smoother_server", name="smoother_server", output="screen", parameters=[params_file]),
            Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen", parameters=[params_file]),
            Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output="screen", parameters=[params_file]),
            Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="screen", parameters=[params_file]),
            Node(package="nav2_waypoint_follower", executable="waypoint_follower", name="waypoint_follower", output="screen", parameters=[params_file]),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{"autostart": True, "node_names": lifecycle_nodes}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", os.path.join(package_share, "rviz", "nav2_low_load.rviz")],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
