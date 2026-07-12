# Car PID Serial Tuner

Python serial helper for receiving car telemetry and sending PID tuning commands.

## Install

```powershell
python -m pip install -r requirements.txt
```

## Run

```powershell
python serial_tuner.py
```

## Expected MCU telemetry format

The parser accepts any text line containing `key=value` pairs. Recommended line:

```text
L=1234 R=1238 LD=8 RD=7 ERR=1 OUT=-0.35 KP=0.35 KI=0.02 KD=0.08 BASE=8.00
```

Common keys:

- `L`, `R`: total left/right encoder counts
- `TARGET`: shared target speed, derived from `BASE / 100`
- `LD`, `RD`: left/right speed feedback in counts per control period
- `ERR`: speed difference, `LD - RD`
- `OUT`: right PWM output minus left PWM output
- `LO`, `RO`: left/right final PWM output
- `KP`, `KI`, `KD`: current PID gains
- `BASE`: shared target speed in counts per control period

The plot shows `TARGET`, `LD`, `RD`, and `ERR`. The left and right speed
feedback lines should converge toward the target line.

## Commands sent to MCU

连接 `serial_tuner.py` 后可使用方向键驾驶：上/下前进后退，左/右原地转向；松开全部方向键立即发送 `STOP`，组合按键支持斜向运动。

The UI sends newline-terminated ASCII commands:

```text
PID 0.35 0.02 0.08
BASE 8.0
GET
```

## Raw UART dump

Use this to check whether a UART is receiving any bytes before debugging a
higher-level parser or ROS2 node. This is useful for checking MCU UART2
odometry before moving the wire to the Raspberry Pi.

```powershell
python uart_raw_dump.py --port COMx --baudrate 115200 --seconds 10
```

For MCU UART2 odometry, connect only:

```text
MCU PA23 / UART2 TX -> USB-TTL RX
MCU GND             -> USB-TTL GND
```

Expected odometry lines look like:

```text
ODO L=1234 R=1230 LD=0 RD=0
```

After this works on the PC, the Raspberry Pi mapping side should use:

```bash
cd /home/heximiao/hexi/ros2/slam/lidar
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lidar_py_pkg mapping.launch.py
```

The Raspberry Pi wiring for odometry is MCU PA23 / UART2 TX to GPIO15 RXD
physical pin 10, plus common GND. For visual line following, Raspberry Pi
GPIO14 TXD physical pin 8 can also connect to MCU PA24 / UART2 RX, but run the
line-follow script and ROS2 odometry reader one mode at a time.
