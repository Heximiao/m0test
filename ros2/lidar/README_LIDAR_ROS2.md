# 激光雷达 ROS2 建图说明

这个目录是树莓派上的 ROS2 Jazzy 雷达建图工作区。它负责读取串口激光雷达，
发布 `/scan`，读取单片机 UART2 发来的 `ODO` 里程计，发布 `/odom` 和 TF，
再用 `slam_toolbox` 建图并在 RViz 里显示 `/map`。

## 启动建图

在树莓派上运行：

```bash
cd /home/heximiao/hexi/ros2/slam/lidar
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lidar_py_pkg mapping.launch.py
```

这个 launch 会同时启动：

- 激光雷达节点，发布 `/scan`
- 里程计节点 `wheel_odom_node`，读取 MCU 的 `ODO` 行并发布 `/odom`
- 动态 TF：`odom -> base_link`
- 静态 TF：`base_link -> laser`
- `slam_toolbox`
- `slam_toolbox` 生命周期管理器
- RViz

当小车、树莓派和雷达一起真实移动时，建图会自动开始。ROS2 本身不需要再发
`BASE 20` 才能建图；`BASE 20`、`DIST ...` 这些只是发给单片机的运动命令。

如果通过远程桌面打开 RViz，并且图形显示有问题，可以用软件 OpenGL：

```bash
cd /home/heximiao/hexi/ros2/slam/lidar
source /opt/ros/jazzy/setup.bash
source install/setup.bash
DISPLAY=:10.0 LIBGL_ALWAYS_SOFTWARE=1 ros2 launch lidar_py_pkg mapping.launch.py
```

RViz 里建议确认：

- `Global Options -> Fixed Frame` 是 `map`
- `Map` 的 topic 是 `/map`
- `LaserScan` 的 topic 是 `/scan`
- `Global Status` 显示 `Ok`

如果能看到一大片地图栅格，说明 `/map` 已经发布成功。小车静止时地图可能只是
一片较空的栅格，这是正常的。真正有意义的地图需要树莓派和雷达跟着小车一起移动。

## 保存地图

真实跑完一圈后再保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/hexi/ros2/slam/lidar/my_map
```

会生成：

```text
my_map.yaml
my_map.pgm
```

## 只看雷达

只启动雷达节点：

```bash
ros2 launch lidar_py_pkg lidar_py.launch.py
```

启动雷达并打开 RViz：

```bash
ros2 launch lidar_py_pkg view_lidar.launch.py
```

远程桌面下如果 RViz 显示异常，可以加软件 OpenGL：

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch lidar_py_pkg view_lidar.launch.py
```

## 当前默认配置

- 雷达串口：`/dev/ttyACM0`
- 串口波特率：`115200`
- MCU 里程计串口：`/dev/ttyAMA0`
- 雷达话题：`/scan`
- 里程计话题：`/odom`
- 地图话题：`/map`
- 雷达坐标系：`laser`
- 车体坐标系：`base_link`
- 建图时 TF 链路：`map -> odom -> base_link -> laser`

## 串口接线

建图只需要 MCU 把里程计发给树莓派：

```text
MCU PA23 / UART2 TX -> 树莓派 GPIO15 RXD / 物理 10 脚
MCU GND             -> 树莓派 GND
```

当前固件也打开了 MCU PA24 / UART2 RX，所以同一组 UART2 也可以用于视觉寻迹：

```text
树莓派 GPIO14 TXD / 物理 8 脚 -> MCU PA24 / UART2 RX
```

树莓派需要在 `/boot/firmware/config.txt` 里有：

```text
enable_uart=1
```

在当前测试过的树莓派 5 上，GPIO14/15 对应的串口设备是 `/dev/ttyAMA0`。

## 建图和视觉寻迹的关系

现在建议二选一运行：

- 建图：运行 `ros2 launch lidar_py_pkg mapping.launch.py`
- 视觉寻迹：运行 `/home/heximiao/hexi/opencv/line_follow_opencv.py`

不要同时运行建图里的 `wheel_odom_node` 和 OpenCV 视觉寻迹脚本，因为它们都会使用
树莓派同一个 UART `/dev/ttyAMA0`。如果后面要同时跑，需要再做一个串口桥节点，让同
一个程序统一读 `ODO` 并写 `LINE` / `LTURN`。

视觉寻迹当前默认串口已经是 `/dev/ttyAMA0`，在树莓派上可以这样启动：

```bash
cd /home/heximiao/hexi/opencv
python3 line_follow_opencv.py --show
```

无窗口运行：

```bash
cd /home/heximiao/hexi/opencv
python3 line_follow_opencv.py --no-show
```

启动后看到下面这种输出，说明视觉程序识别到线并正在向 MCU 发送命令：

```text
serial opened: /dev/ttyAMA0 @ 115200
TX: LINE 1 104 M=S P=40 FPS: 15.0
```

## 常用检查

检查时要保持建图 launch 还在运行。也就是一个终端运行：

```bash
cd /home/heximiao/hexi/ros2/slam/lidar
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lidar_py_pkg mapping.launch.py
```

另开一个终端：

```bash
source /opt/ros/jazzy/setup.bash
source /home/heximiao/hexi/ros2/slam/lidar/install/setup.bash
```

然后一个一个运行检查命令：

```bash
ros2 topic list
ros2 topic echo --once /scan
ros2 topic echo --once /odom
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo --once /map
ros2 node info /slam_toolbox
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo map odom
```

注意：`ros2 topic hz ...` 会一直运行，直到按 `Ctrl-C` 停止。不要把后面的检查命令
一次性粘在它后面。

如果启动建图的终端已经按了 `Ctrl-C`，那么 `/scan`、`/odom`、`/map` 和 TF 都会消失。
这种情况下要先重新启动 `ros2 launch lidar_py_pkg mapping.launch.py`。

## 排查提示

- RViz 里看建图时，Fixed Frame 用 `map`。
- RViz 里只看雷达时，Fixed Frame 可以用 `laser`。
- 如果没有 `/scan`，先检查雷达是否在 `/dev/ttyACM0`。
- 如果没有 `/odom`，先检查 MCU PA23 是否接到树莓派 GPIO15 RXD，以及串口是否是 `/dev/ttyAMA0`。
- 如果有 `/scan` 和 `/odom` 但没有 `/map`，检查：

```bash
ros2 node info /slam_toolbox
```

正常情况下，`/slam_toolbox` 应该订阅 `/scan`，并发布 `/map`、`/map_metadata`、`/pose` 和 `/tf`。

- 如果视觉寻迹有 `TX: LINE ...` 但轮子不动，先确认是否看到：

```text
serial opened: /dev/ttyAMA0 @ 115200
```

再确认已经烧录打开 UART2 RX 的单片机固件，并且树莓派 GPIO14 TXD 已接到 MCU PA24 RX。
