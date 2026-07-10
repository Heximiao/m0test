# Lidar ROS2 Usage

This workspace runs the serial lidar, opens RViz2, and can start
`slam_toolbox` for mapping. The current Raspberry Pi setup uses ROS2 Jazzy.

Workspace setup:

```bash
cd /home/heximiao/hexi/ros2/slam/lidar
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

If this terminal was already open before installing ROS packages, run the two
`source` commands again so RViz can find its runtime libraries.

Start only the lidar node:

```bash
ros2 launch lidar_py_pkg lidar_py.launch.py
```

View the lidar in RViz:

```bash
ros2 launch lidar_py_pkg view_lidar.launch.py
```

On the Raspberry Pi remote desktop, RViz may need software OpenGL rendering:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch lidar_py_pkg view_lidar.launch.py
```

Start the mapping bringup:

```bash
ros2 launch lidar_py_pkg mapping.launch.py
```

Remote desktop mapping command:

```bash
DISPLAY=:10.0 LIBGL_ALWAYS_SOFTWARE=1 ros2 launch lidar_py_pkg mapping.launch.py
```

Current defaults:

- Serial port: `/dev/ttyACM0`
- Baudrate: `115200`
- Scan topic: `/scan`
- Laser frame: `laser`
- Base frame: `base_link`
- Mapping RViz fixed frame: `laser` for now, so `/scan` is visible even before
  wheel odometry is publishing `odom -> base_link`.

Useful checks:

```bash
ros2 topic list
ros2 topic echo --once /scan
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo base_link laser
```

Notes:

- `rviz2` is installed.
- `slam_toolbox`, `navigation2`, and `nav2_bringup` are installed on the
  Raspberry Pi.
- `mapping.launch.py` starts the lidar node, static `base_link -> laser` TF,
  `slam_toolbox`, and RViz.
- If RViz starts but no scan is visible, check that the fixed frame is `laser`
  and the LaserScan topic is `/scan`.
- Real mapping needs robot motion information, usually an `odom -> base_link`
  transform from wheel odometry. The CCS firmware already reports encoder
  telemetry as `L=... R=... LD=... RD=...`; a ROS2 odometry node can parse the
  cumulative `L/R` counts and publish `/odom` plus `odom -> base_link`.
- After wheel odometry is available, switch the mapping RViz fixed frame back to
  `map` to view the growing occupancy grid cleanly.
