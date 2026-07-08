# MSPM0 双轮小车控制工程

这是一个基于 TI MSPM0G3507 LaunchPad 的 CCS 工程，用 TB6612 驱动左右两个直流减速电机，通过霍尔编码器做双轮速度闭环，并支持 OpenMV/树莓派视觉巡线、基础里程运动命令、MPU6050 姿态遥测、SPI LCD 状态显示和 PC 上位机在线调参。

当前固件默认速度环参数：

```text
Base = 20
Kp = 4.0
Ki = 0.2
Kd = 0.4
```

`BASE` 的单位不是 PWM 百分比，而是每个 20 ms 控制周期内的编码器增量。例如 `BASE 20` 表示目标速度约为 `20 counts / 20 ms`。

## 当前功能

- 左右轮编码器计数、测速和一阶低通滤波。
- 20 ms 双轮速度闭环，左右轮使用同一组 PID 参数。
- 速度目标斜坡、PWM 手动测试斜坡、前馈起步占空比和左右轮前馈比例校准。
- UART0 调试/调参，默认 `115200 8N1`，100 ms 遥测。
- OpenMV UART2 接收 `LINE`/`LTURN` 视觉命令，实现自动巡线和路口原地转弯。
- 里程运动命令：速度、距离、角速度、转角。
- MPU6050 软件 I2C 姿态读取，优先 DMP，失败时回退到加速度计姿态。
- SPI LCD 初始化显示，屏幕上电后显示项目状态和右轮 PWM 引脚提示。
- LCD 二级菜单，支持文件列表、车辆状态和车辆控制。
- Python 上位机曲线显示、PID/BASE 下发和 CSV 保存。
- 树莓派/OpenCV 版本巡线脚本，可通过串口向 MCU 发送同样的视觉命令。

## 运行节拍

| 周期 | 任务 |
| --- | --- |
| 1 ms | SysTick 系统时基 |
| 20 ms | 速度控制更新 |
| 20 ms | MPU6050 姿态输出尝试 |
| 100 ms | 小车状态遥测 |
| 500 ms | LaunchPad 状态 LED 翻转 |
| 30 s | 心跳 `RUN` 输出 |

## 控制量说明

| 名称 | 来源 | 说明 |
| --- | --- | --- |
| `BASE` | 命令/遥测 | 共享基础目标速度，遥测中按 `*100` 输出，单位为 `counts/20ms` |
| `LD` / `RD` | 遥测 | 左/右轮滤波后的实际速度，按 `*100` 输出 |
| `ERR` | 遥测 | `LD - RD`，表示左右轮速度差，不是目标速度误差 |
| `LO` / `RO` | 遥测 | 左/右轮最终 PWM 输出，按 `*100` 输出，单位为百分比 |
| `OUT` | 遥测 | `RO - LO`，左右轮输出差 |
| `LT` / `RT` | 遥测 | 左/右轮当前目标速度，按 `*100` 输出 |
| `LINE` / `LV` | 遥测 | 巡线偏差和视觉数据是否有效 |
| `MO` / `MB` | 遥测 | 运动控制模式和是否忙碌 |
| `KP/KI/KD` | PID 参数 | 速度环 PID 参数，遥测中按 `*1000` 输出 |

速度输出核心逻辑在 `app/app_car_control.c`：

```text
PWM = 前馈输出 + PID(目标速度 - 实际速度)
```

主要前馈和限幅宏：

```c
SPEED_LOOP_START_DUTY_PERCENT
SPEED_LOOP_FEED_FORWARD_PERCENT_PER_COUNT
LEFT_SPEED_FEED_FORWARD_SCALE
RIGHT_SPEED_FEED_FORWARD_SCALE
MAX_DUTY_PERCENT
```

如果平均速度追不上目标，优先调 `SPEED_LOOP_FEED_FORWARD_PERCENT_PER_COUNT`；如果某一侧轮子长期偏快或偏慢，优先微调 `LEFT_SPEED_FEED_FORWARD_SCALE` 或 `RIGHT_SPEED_FEED_FORWARD_SCALE`。

## 串口命令

所有命令都是 ASCII 文本，以换行结束；固件会自动忽略行首空格、制表符和 `>`，并把小写转成大写。

### 速度环和调试

```text
PID <kp> <ki> <kd>
BASE <speed_counts_per_period>
STOP
GET
ENCZERO
TELE 0|1
QUIET 0|1
```

命令会自动忽略行首空格、制表符和 `>`，并转成大写后再解析。`PID` 更新左右轮共用的速度环参数；`BASE` 设置共享目标速度并退出手动电机/运动控制；`STOP` 停止电机、清除运动和手动模式；`GET` 输出当前状态；`ENCZERO` 清零左右编码器计数；`TELE 1/0` 打开或关闭 100 ms 遥测；`QUIET 1` 等价于关闭遥测。

常用调试流程：

```text
TELE 1
BASE 20
PID 4.0 0.2 0.4
GET
STOP
```

### 手动电机测试

```text
PWM <left_duty_percent> <right_duty_percent> [duration_ms]
MOTOR <duty_percent> [duration_ms]
PULSE <duty_percent> <duration_ms>
```

`PWM` 分别设置左右轮开环占空比，`MOTOR` 给左右轮同一个开环占空比，`PULSE` 是短时同占空比脉冲测试。`MOTOR` 和 `PWM` 默认运行 2000 ms，最长 10000 ms；持续时间填 `0` 时会一直保持手动模式，直到 `STOP` 或新的控制命令。`PULSE` 最长 2000 ms。占空比会被限制在 `0%` 到 `85%`。

### 运动命令

```text
DRIVE <linear_mm_s> <angular_deg_s>
SPEED <linear_mm_s>
DIST <distance_mm> <linear_mm_s>
TURN <angle_deg> [angular_deg_s]
ANGLE <angle_deg> [angular_deg_s]
MSTOP
```

`DRIVE` 同时给线速度和角速度，`SPEED` 只给直线速度，`DIST` 按距离运行，`TURN`/`ANGLE` 按角度原地转向；`TURN`/`ANGLE` 不写角速度时默认 `90 deg/s`。收到运动命令后，固件会停止 `BASE` 目标并返回 `OK MOTION`；`MSTOP` 只停止运动控制模式。

运动换算参数在 `app/app_motion_control.c`：

- 车轮直径：`48.0 mm`
- 轮距：`129.0 mm`
- 电机编码器：`11 PPR`
- 减速比：`20:1`
- A/B 相计数倍率：`2`
- 最大线速度：`100 mm/s`
- 最大角速度：`120 deg/s`

### 视觉巡线

```text
LINE <valid> <error_pixels>
LTURN <angle_deg>
```

`LINE 1 error` 表示视觉识别有效，`error` 为黑线目标点相对画面中心的横向偏差；`LINE 0 0` 表示暂时丢线。固件收到有效巡线数据后，会在没有显式运动命令时自动以前进基础速度巡线。

`LTURN` 用于路口原地转弯，当前按 90 度约 `275` 个编码器 count 换算，默认转弯速度 `15 counts/20ms`，开环转弯占空比 `15%`，转弯后会延迟约 `1900 ms` 再恢复巡线。

### MPU6050

```text
MPU
MPUINIT
MPUZERO
```

`MPU` 输出初始化状态、I2C 引脚状态、DMP/FIFO 调试信息和失败计数。`MPUINIT` 重新初始化 MPU6050 和 DMP。`MPUZERO` 清零回退姿态算法的参考角。

### W25Q64 图片存储

```text
FLASHID
IMG_SAVE <name> <width> <height> <size> <crc32>
IMG_WRITE <id> <width> <height> <size> <crc32>
IMG_LIST
IMG_INFO
IMG_SHOW <id>
IMG_SLOT_SHOW <slot>
IMG_DELETE <id>
IMG_DEFRAG
```

图片数据通过 UART0 发送，格式为 RGB565 原始像素流。`IMG_SAVE` 自动分配图片 ID，名称最长 15 个非空白字符；`IMG_WRITE` 覆盖指定 ID，`id` 范围为 `0..254`；`IMG_SHOW` 从索引中按 ID 显示图片；`IMG_SLOT_SHOW` 兼容旧版固定槽位地址；`IMG_INFO` 输出容量、已用空间、剩余空间和图片数量；`IMG_LIST` 输出所有索引项；`IMG_DEFRAG` 会把有效图片向前整理，回收删除图片留下的碎片空间。推荐使用 `upper_pc/image_sender.py` 或上位机工具发送图片，不手工敲二进制数据。

## 遥测格式

开启 `TELE 1` 后，固件每 100 ms 输出一行键值对，例如：

```text
L=1234 R=1230 LD=1987 RD=1975 ERR=12 OUT=-35 LO=2100 RO=2065 LT=2000 RT=2000 KP=4000 KI=200 KD=400 BASE=2000 LINE=0 LV=0 MM=0 DG=0 MO=0 MB=0
```

姿态模块会单独输出：

```text
ATT SRC=DMP PITCH=12 ROLL=-34 YAW=9000
```

其中 `PITCH/ROLL/YAW` 按 `*100` 输出，单位为度。心跳行格式如下：

```text
RUN ms=30000 L=1234 R=1230 RXDROP=0 TXDROP=0 LED=PB22
```

## 引脚连接

| 功能 | 引脚 | 代码/SysConfig 名称 |
| --- | --- | --- |
| 左轮 PWM / TB6612 PWMD | PA8 | `PWM_LEFT` / TIMA0 CCP0 |
| 左轮方向 / TB6612 DIN1, DIN2 | PA25, PA31 | `GPIO_MOTOR_A_AIN1`, `GPIO_MOTOR_A_AIN2` |
| 右轮 PWM / TB6612 PWMA | PB4 | `PWM_RIGHT` / TIMA1 CCP0 |
| 右轮方向 / TB6612 AIN1, AIN2 | PB16, PB13 | `GPIO_MOTOR_B_BIN1`, `GPIO_MOTOR_B_BIN2` |
| TB6612 STBY | PA27 | `GPIO_MOTOR_A_STBY` |
| 左轮编码器 E4A, E4B | PB2, PB3 | `LEFT_C0`, `LEFT_C1` |
| 右轮编码器 E1A, E1B | PB0, PB1 | `RIGHT_C0`, `RIGHT_C1` |
| UART0 调试 TX, RX | PA10, PA11 | `UART_DEBUG` |
| OpenMV UART2 TX, RX | PA23, PA24 | `UART_OPENMV` |
| OpenMV P4 TX -> MCU RX | PA24 | 接 MCU `UART2 RX` |
| OpenMV P5 RX <- MCU TX | PA23 | 接 MCU `UART2 TX` |
| 树莓派 GPIO14 TX -> MCU RX | PA24 | 可替代 OpenMV TX |
| 树莓派 GPIO15 RX <- MCU TX | PA23 | 可替代 OpenMV RX |
| MPU6050 SCL, SDA | PA1, PA0 | 软件 I2C，开漏上拉 |
| LCD SPI MOSI, SCLK | PB8, PB9 | `SPI_LCD`，16 MHz |
| LCD RES, DC, CS, BLK | PB10, PB11, PB14, PB26 | `GPIO_LCD` |
| W25Q64 CS, SCLK, MOSI, MISO | PA12, PA13, PA14, PA15 | 软件 SPI，`GPIO_W25Q64` |
| 菜单按键 上翻, 下翻, 返回/确认 | PA16, PA17, PA21 | `GPIO_MENU_KEYS`，内部上拉，按下接地 |
| 状态 LED | PB22 | `GPIO_STATUS_LED_PB22_LED` |

左右轮命名以车尾看向车头为准，并与 `main.c` 的当前接线注释一致：左轮接 TB6612 的 `PWMD/DIN1/DIN2`，右轮接 TB6612 的 `PWMA/AIN1/AIN2`。代码里 `AO_Control()` 输出左轮 PWM（`PWM_LEFT`/PA8）并控制 PA25、PA31 两个方向脚，`BO_Control()` 输出右轮 PWM（`PWM_RIGHT`/PB4）并控制 PB16、PB13 两个方向脚；这里的 `GPIO_MOTOR_A/B` 是工程里的 GPIO 分组名，不等同于物理左右轮命名。编码器极性在 `hw/hw_encoder.c` 中处理，前进方向计数为正。`PB9` 当前用于 LCD SCLK，不再是右轮 PWM。

## LCD 菜单

三个按键均为一端接 GPIO、一端接 GND，GPIO 使用内部上拉，按下为低电平：

```text
PA16: 上翻
PA17: 下翻
PA21: 返回/确认，短按返回，长按确认
```

一级菜单包括 `File List`、`Vehicle Status` 和 `Vehicle Control`。`File List` 会读取 W25Q64 图片索引，显示图片 ID、名称和尺寸，长按确认可显示选中图片；`Vehicle Status` 显示左右编码器、运动模式、忙碌状态和当前运动目标；`Vehicle Control` 提供停止电机、编码器清零、`BASE 0`、`BASE 7`、遥测开关等快捷操作。

## 上位机工具

```powershell
cd upper_pc
python -m pip install -r requirements.txt
python serial_tuner.py
```

图片写入 W25Q64：

```powershell
python upper_pc/image_sender.py path\to\image.png --port COMx --show
```

脚本会把 PNG/JPG 转成 `320x170` RGB565，先发送 `IMG_SAVE` 命令自动分配图片 ID，再通过 UART0 发送二进制像素数据。需要覆盖指定 ID 时可加 `--slot 3`。固件端支持 `FLASHID`、`IMG_SAVE <name> <width> <height> <size> <crc32>`、`IMG_WRITE <id> <width> <height> <size> <crc32>`、`IMG_LIST`、`IMG_INFO`、`IMG_DELETE <id>`、`IMG_DEFRAG` 和 `IMG_SHOW <id>`。

上位机主要用于连接 COM 口、下发 `PID`/`BASE`/`GET`、绘制 `TARGET`、`LD`、`RD`、`ERR` 曲线并保存 CSV。更详细说明见 `upper_pc/README.md`。

## 视觉脚本

OpenMV 脚本：

```text
openmv/xunjitest1.py
```

该脚本在 QVGA 画面中做黑线二值化，输出 `LINE <valid> <error>`；检测到直角/T 字路口并满足触发条件后，会连续发送 `LTURN <angle>`。

树莓派/OpenCV 脚本：

```powershell
python opencv/line_follow_opencv.py --serial COMx --baudrate 115200
```

Linux/树莓派上默认摄像头为 `/dev/video0`，串口为 `/dev/ttyAMA0`。调试时可加 `--dry-run` 只看识别结果，不打开串口；加 `--no-show` 关闭窗口显示。

## 文件结构

```text
gpio_toggle_output/
├── .ccsproject                  CCS 工程描述文件
├── .cproject                    Eclipse/CDT C 工程配置
├── .project                     Eclipse 工程元数据
├── .clangd                      clangd 配置
├── gpio_toggle_output.syscfg    TI SysConfig 外设配置源文件
├── main.c                       程序入口、任务调度和中断入口
├── README.md                    当前工程说明
├── app/                         应用层逻辑
│   ├── app_car_control.c/.h     小车主控制、速度环、串口命令、遥测、巡线融合
│   ├── app_image_store.c/.h     W25Q64 图片索引、写入、列表、显示和删除
│   ├── app_line_follow.c/.h     解析 LINE 数据并生成巡线转向修正
│   ├── app_menu.c/.h            LCD 二级菜单、按键扫描和快捷控制
│   ├── app_motion_control.c/.h  距离、速度、转角等运动命令换算
│   ├── app_mpu6050_attitude.c/.h MPU6050 姿态初始化、读取和命令处理
│   └── app_util.h               常用缩放、限幅和绝对值工具
├── bsp/                         板级支持层
│   ├── bsp_tb6612.c/.h          TB6612 电机驱动
│   └── mpu6050/                 MPU6050、DMP 和软件 I2C 驱动
├── hw/                          MCU 外设封装
│   ├── hw_encoder.c/.h          编码器 GPIO 中断计数
│   ├── hw_lcd.c/.h              SPI LCD 初始化、绘图和字符串显示
│   ├── hw_spi.c/.h              LCD SPI 写总线封装
│   ├── hw_uart.c/.h             UART0 调试串口
│   ├── hw_openmv_uart.c/.h      UART2 OpenMV/视觉串口
│   ├── hw_w25q64.c/.h           W25Q64 软件 SPI Flash 读写擦除
│   ├── lcdfont.h                LCD 中文字库
│   └── lcdfont_ascii.h          LCD ASCII 字库
├── mid/
│   └── mid_pid.c/.h             通用 PID 算法
├── openmv/
│   └── xunjitest1.py            OpenMV 巡线和路口识别脚本
├── opencv/
│   └── line_follow_opencv.py    树莓派/OpenCV 巡线脚本
├── upper_pc/
│   ├── image_sender.py          图片转 RGB565 并通过 UART 写入 W25Q64
│   ├── serial_tuner.py          PC 串口调参和曲线工具
│   ├── requirements.txt         Python 依赖
│   └── README.md                上位机说明
├── targetConfigs/               CCS 调试/烧录目标配置
└── Debug/                       CCS Debug 构建输出，可重新生成
```

可以忽略或重新生成的目录包括 `.git/`、`.agents/`、`.settings/`、`.ti_appdata/`、`Debug/` 和 Python 的 `__pycache__/`。

## 编译和烧录

推荐在 Code Composer Studio 中导入本工程，直接执行 Build Project，然后 Debug/Load 到 MSPM0G3507 LaunchPad。

主要构建产物：

```text
Debug/gpio_toggle_output.out
Debug/gpio_toggle_output.hex
```

如果使用仓库当前的 CCS 自动生成 makefile，也可以在 `Debug` 目录执行：

```powershell
gmake all
```

## 调参建议

- 平均速度低于目标：优先增大 `SPEED_LOOP_FEED_FORWARD_PERCENT_PER_COUNT`。
- 起步吃力：适当增大 `SPEED_LOOP_START_DUTY_PERCENT`。
- 速度到目标但抖动：适当降低 `Kp`，或小幅增加 `Kd`。
- 长时间存在静态误差：小幅增加 `Ki`。
- 左右轮某一侧长期偏快：微调 `LEFT_SPEED_FEED_FORWARD_SCALE` 或 `RIGHT_SPEED_FEED_FORWARD_SCALE`。
- 巡线过猛：降低 `LINE_TURN_KP` 或 `LINE_TURN_LIMIT_COUNTS`。
- 巡线速度太快或太慢：调整 `LINE_FOLLOW_BASE_COUNTS` 和 `LINE_FOLLOW_MAX_COUNTS`。
