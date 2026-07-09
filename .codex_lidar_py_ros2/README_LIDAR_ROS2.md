# Lidar ROS2 Usage

Workspace:

```bash
cd /home/ubuntu/hexi/ros2/lidar
source /opt/ros/lyrical/setup.bash
source install/setup.bash
```

Start only the lidar node:

```bash
ros2 launch lidar_py_pkg lidar_py.launch.py
```

View the lidar in RViz:

```bash
ros2 launch lidar_py_pkg view_lidar.launch.py
```

Start the mapping bringup:

```bash
ros2 launch lidar_py_pkg mapping.launch.py
```

Current defaults:

- Serial port: `/dev/ttyACM0`
- Baudrate: `115200`
- Scan topic: `/scan`
- Laser frame: `laser`
- Base frame: `base_link`

Useful checks:

```bash
ros2 topic list
ros2 topic echo --once /scan
ros2 topic hz /scan
```

Notes:

- `rviz2` is installed.
- `slam_toolbox` is not available from the current ROS2 Lyrical apt source.
- `mapping.launch.py` will start lidar, static `base_link -> laser` TF, and RViz. If `slam_toolbox` is installed later, the same launch file will start it automatically.
- Real mapping also needs robot motion information, usually an `odom -> base_link` transform from wheel odometry or another localization source.
