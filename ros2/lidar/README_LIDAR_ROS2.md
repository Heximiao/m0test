# 激光雷达 ROS 2 建图与导航

本目录是树莓派上的 ROS 2 Jazzy 工作空间源码副本。`lidar_py_pkg` 负责：

- 从 `/dev/ttyACM0` 读取激光雷达并发布 `/scan`
- 从 `/dev/ttyAMA0` 读取 MCU 里程计并发布 `/odom`
- 发布 `odom -> base_link` 和 `base_link -> laser` TF
- 将 Nav2 的 `/cmd_vel` 转换为 MCU 的 `NAV` 指令
- 使用 `slam_toolbox` 建图
- 使用 Nav2、AMCL 和已保存地图导航

## 环境与构建

树莓派工作空间路径：

```bash
cd /home/heximiao/hexi/ros2/slam/lidar
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

修改源码或配置后需要重新执行 `colcon build --symlink-install`。

## 建图

```bash
cd /home/heximiao/hexi/ros2/slam/lidar
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lidar_py_pkg mapping.launch.py
```

该 launch 会启动雷达、底盘驱动、里程计、TF、`slam_toolbox` 和 RViz。建图时应确认：

- `/scan`、`/odom` 和 `/map` 正常发布
- TF 链为 `map -> odom -> base_link -> laser`
- RViz 的 Fixed Frame 为 `map`

## 保存地图

推荐把地图统一保存到 `~/hexi/ros2/slam/maps`：

```bash
mkdir -p ~/hexi/ros2/slam/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/hexi/ros2/slam/maps/my_map1
```

成功后生成：

```text
~/hexi/ros2/slam/maps/my_map1.yaml
~/hexi/ros2/slam/maps/my_map1.pgm
```

YAML 中的 `image` 通常使用相对路径，因此 YAML 和 PGM 应保持在同一目录。

## 使用已有地图导航

建图与导航不能同时运行。先退出建图 launch，再执行：

```bash
cd /home/heximiao/hexi/ros2/slam/lidar
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lidar_py_pkg navigation.launch.py \
  map:=/home/heximiao/hexi/ros2/slam/maps/my_map1.yaml
```

导航 launch 会启动雷达、底盘驱动、地图服务器、AMCL、Nav2 和 Foxglove Bridge。Foxglove Bridge 默认监听所有网卡的 TCP `8765` 端口，RViz 默认关闭。

在 RViz 中按以下顺序操作：

1. 点击 `2D Pose Estimate`。
2. 在地图上设置小车当前真实位置和朝向。
3. 等待激光轮廓与地图障碍物基本重合。
4. 点击 `Nav2 Goal`，设置目标位置和最终朝向。
5. 确认 Navigation 面板最终显示 `Feedback: reached`。

初始位姿未设置时出现以下提示属于正常现象：

```text
AMCL cannot publish a pose or update the transform.
Please set the initial pose...
```

无界面启动：

```bash
ros2 launch lidar_py_pkg navigation.launch.py \
  map:=/home/heximiao/hexi/ros2/slam/maps/my_map1.yaml \
  use_rviz:=false
```

如需恢复 RViz 并临时关闭 Foxglove Bridge：

```bash
ros2 launch lidar_py_pkg navigation.launch.py \
  map:=/home/heximiao/hexi/ros2/slam/maps/my_map1.yaml \
  use_rviz:=true \
  use_foxglove:=false
```

## Foxglove 远程导航地图

Windows 上的 Foxglove Studio 不需要 WSL，也不依赖 ROS 2 DDS 直接穿过热点。导航 launch 启动后，在 Foxglove 中添加 ROS 2 实时连接：

树莓派需要安装 Jazzy 对应的 Bridge 包：

```bash
sudo apt install ros-jazzy-foxglove-bridge
```

本机已安装时无需重复执行。Foxglove Bridge 也可以单独启动：

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

正常使用本工程时直接启动 `navigation.launch.py` 即可，不要再单独启动第二个 Bridge，否则会争用 `8765` 端口。

在 Windows Foxglove 中输入：

```text
ws://192.168.137.53:8765
```

如果树莓派的 IP 地址改变，只需把地址中的 IP 替换为新 IP，例如：

```text
ws://192.168.137.88:8765
```

端口仍为 `8765`，除非同时修改了 `navigation.launch.py` 中 Foxglove Bridge 的 `port` 参数。可从 Windows 验证端口：

```powershell
Test-NetConnection 192.168.137.53 -Port 8765
```

在 Foxglove 的 3D 面板中：

1. 将固定参考系设为 `map`。
2. 显示 `/map`、`/scan`、`/tf`、`/odom`、`/plan` 和 `/local_plan`。
3. 在“发布”中把 `2D 位姿估计` 的话题设为 `/initialpose`。
4. 在小车实际位置按住鼠标，朝车头方向拖动后松开，发布初始位置和朝向。
5. 初始位姿发布成功后，TF 链应变为 `map -> odom -> base_link -> laser`。

`2D 位姿估计` 的 X/Y 偏差表示位置不确定度，方位角偏差表示朝向不确定度。默认的 `0.5`、`0.5`、`0.26179939`（约 15°）可以直接使用；人工定位较准确时可使用 `0.2`、`0.2`、`0.1`，不要全部设为零。

Foxglove 的 `2D 位姿` 可以向 `/goal_pose` 发布 `geometry_msgs/msg/PoseStamped`，但当前 Nav2 导航入口是 `NavigateToPose` Action。未添加话题到 Action 的转发节点前，发布 `/goal_pose` 只会产生目标位姿消息，不保证车辆开始导航；此时使用 RViz 的 `Nav2 Goal`。

如果 Foxglove 顶部连接变红，先确认导航 launch 仍在运行，再检查：

```bash
ss -lntp | grep 8765
```

Windows 热点重新连接或树莓派重启后，Foxglove 可能需要手动重新连接数据源。

## 远程桌面与低负载 RViz

xrdp 通常使用软件渲染。Nav2 默认 RViz 配置可能占满一个以上 CPU 核心，导致远程桌面卡死。

`navigation.launch.py` 已改为使用 `rviz/nav2_low_load.rviz`：

- 帧率由 30 FPS 降为 1 FPS
- 关闭仅用于显示的 LaserScan 和 PointCloud2 图层
- 保留地图、路径、机器人和 Navigation 面板
- 不影响 Nav2 实际订阅雷达和执行避障

如果 Nav2 已经运行但 RViz 被关闭，可单独重新打开低负载界面：

```bash
source /opt/ros/jazzy/setup.bash
source ~/hexi/ros2/slam/lidar/install/setup.bash
rviz2 -d ~/hexi/ros2/slam/lidar/install/lidar_py_pkg/share/lidar_py_pkg/rviz/nav2_low_load.rviz
```

不要重复启动多套 `navigation.launch.py` 或多个 RViz，否则同名节点和 action server 可能发生冲突。

## 当前导航参数

主要参数位于 `src/lidar_py_pkg/config/nav2_params.yaml`：

- 控制频率：10 Hz
- 最大线速度：0.10 m/s
- 最大角速度：1.2 rad/s
- 车体安全半径：0.11 m
- 障碍膨胀半径：0.20 m
- 局部代价地图：更新 5 Hz，发布 2 Hz
- 全局代价地图：更新 1 Hz，发布 0.5 Hz
- BT action 默认等待：2000 ms
- 服务等待：5000 ms

这些设置已针对树莓派负载做过调整。此前过短的 action 等待会导致导航只行驶几秒便被取消。

## 底盘串口协议

树莓派向 MCU 发送：

```text
NAV 50 0       # 线速度 50 mm/s，角速度 0 deg/s
WHEELS 10 10   # 左右轮目标速度，单位 counts/20ms
STOP           # 立即停止
```

MCU 应持续发送：

```text
ODO L=<left_count> R=<right_count>
```

底盘驱动订阅 `/cmd_vel`，约 0.4 秒未收到新指令时会发送 `STOP`。串口已设置读写超时，退出 launch 时不会长期卡在关闭串口阶段。

串口接线：

```text
MCU PA23 / UART2 TX -> 树莓派 GPIO15 RXD（物理 10 脚）
树莓派 GPIO14 TXD   -> MCU PA24 / UART2 RX（物理 8 脚）
MCU GND             -> 树莓派 GND
```

树莓派 `/boot/firmware/config.txt` 需要包含：

```text
enable_uart=1
```

## 常用检查命令

保持对应 launch 正在运行，并在另一个终端执行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/hexi/ros2/slam/lidar/install/setup.bash

ros2 node list
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic info -v /cmd_vel
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
```

典型问题：

- 没有 `/scan`：检查 `/dev/ttyACM0` 和雷达供电。
- 没有 `/odom`：检查 `/dev/ttyAMA0`、MCU UART2 和 `ODO` 数据。
- 没有 `map -> odom`：使用 `2D Pose Estimate` 设置初始位姿。
- 行驶数秒后 `aborted`：检查 `controller_server` 和 `bt_navigator` 日志中的 action timeout、`Failed to make progress` 或 `No valid trajectories`。
- 远程桌面卡死：关闭高负载 RViz，改用 `nav2_low_load.rviz`。

按一次 `Ctrl-C` 后应等待节点依次退出，不要连续反复按键。
