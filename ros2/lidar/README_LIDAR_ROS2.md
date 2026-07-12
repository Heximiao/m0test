# 激光雷达 ROS2 建图说明

这个目录是树莓派上的 ROS2 Jazzy 雷达建图工作区。它负责读取串口激光雷达，
发布 `/scan`，读取单片机 UART2 发来的 `ODO` 里程计，发布 `/odom` 和 TF，
再用 `slam_toolbox` 建图并在 RViz 里显示 `/map`。

## 启动建图

建图启动同时启动底盘驱动节点 `base_driver_node`。该节点从 `/dev/ttyAMA0` 读取
`ODO`，发布 `/odom` 和 `odom -> base_link`，并把 Nav2 的 `/cmd_vel` 转换为
MCU 的 `NAV <linear_mm_s> <angular_deg_s>` 命令。MCU 在约 400 ms 未收到新导航命令时自动停车。

底盘协议也支持直接设置左右轮目标速度：

```text
NAV 50 0       # 线速度 50 mm/s，角速度 0 deg/s
WHEELS 10 10   # 左右轮目标，单位 counts/20ms
STOP           # 立即停止
```

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

当前已经保存并用于导航的地图是：

```text
/home/heximiao/hexi/ros2/slam/maps/my_first_map.yaml
/home/heximiao/hexi/ros2/slam/maps/my_first_map.pgm
```

地图 YAML 中的 `image` 可以使用相对路径，因此 YAML 和 PGM 应放在同一目录，移动或复制时要一起操作。

## 使用已有地图导航

建图和导航是两个独立模式，不要同时运行：

- 建图模式运行 `mapping.launch.py`，由 `slam_toolbox` 实时生成 `/map`。
- 导航模式运行 `navigation.launch.py`，由 `map_server` 加载已经保存的地图，并由 AMCL 定位。

完成建图和保存地图后，先在建图终端按 `Ctrl-C`，确认 `slam_toolbox` 已退出，再启动导航：

```bash
cd /home/heximiao/hexi/ros2/slam/lidar
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lidar_py_pkg navigation.launch.py
```

默认加载：

```text
/home/heximiao/hexi/ros2/slam/maps/my_first_map.yaml
```

加载其他地图时通过 `map` 参数指定 YAML 的绝对路径：

```bash
ros2 launch lidar_py_pkg navigation.launch.py \
  map:=/home/heximiao/hexi/ros2/slam/maps/another_map.yaml
```

导航 launch 会同时启动：

- 激光雷达节点，发布 `/scan`
- 底盘驱动节点，发布 `/odom` 和 `odom -> base_link`，订阅 `/cmd_vel`
- 静态 TF：`base_link -> laser`，雷达位于驱动轮轴线前方 `0.025 m`
- `map_server`，加载 PGM/YAML 地图
- AMCL，发布 `map -> odom`
- Nav2 全局规划器、DWB 局部控制器、行为树和恢复行为
- RViz

### 在 RViz 中开始导航

启动后按下面顺序操作：

1. 等待地图、雷达点和机器人轮廓显示出来。
2. 点击顶部的 `2D Pose Estimate`。
3. 在地图中小车当前所在位置按住鼠标，向车头方向拖动后松开。
4. 等待雷达轮廓与地图障碍大致重合。
5. 点击顶部的 `Nav2 Goal`。
6. 在地图可通行区域按住鼠标，拖出目标朝向后松开。

初始位置未设置前出现以下日志是正常的：

```text
AMCL cannot publish a pose or update the transform.
Please set the initial pose...
```

设置初始位置后，正常情况下会看到：

```text
Managed nodes are active
```

此时 TF 链路应为：

```text
map -> odom -> base_link -> laser
```

如果树莓派不需要打开 RViz，可以无界面启动：

```bash
ros2 launch lidar_py_pkg navigation.launch.py use_rviz:=false
```

### 导航参数与安全限制

当前配置按实车尺寸使用：

- 车体安全半径：`0.11 m`
- 障碍膨胀半径：`0.20 m`
- 最大线速度：`0.10 m/s`
- 最大角速度：`1.2 rad/s`
- 雷达相对驱动轮轴线：前方 `0.025 m`，位于中心线，零度朝车头
- MCU 和树莓派驱动均带约 `0.4 s` 速度命令超时停车保护

第一次落地导航应在空旷区域测试，目标先设置在车前方 `0.3～0.5 m`，并准备随时按
`Ctrl-C` 或断开电机电源。

### 导航状态检查

保持 `navigation.launch.py` 运行，另开终端：

```bash
source /opt/ros/jazzy/setup.bash
source /home/heximiao/hexi/ros2/slam/lidar/install/setup.bash

ros2 topic echo --once /map
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic info -v /cmd_vel
ros2 run tf2_ros tf2_echo map base_link
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
```

设置初始位置后，Nav2 生命周期节点应显示 `active [3]`。如果一直提示没有 `map` TF，说明还没有
设置 `2D Pose Estimate`，或者 AMCL 没有收到 `/scan`。

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
