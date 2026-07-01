# MSPM0 双轮小车 PID 控制工程

这是一个基于 TI MSPM0G3507 LaunchPad 的 CCS 工程，用 TB6612 驱动左右两个直流减速电机，并用霍尔编码器做左右轮速度闭环。当前速度环已经按实车调到一组比较稳定的参数：

```text
Base = 20
Kp = 4.0
Ki = 0.2
Kd = 0.4
```

`BASE` 的单位不是 PWM 百分比，而是每 20 ms 控制周期的编码器增量，所以上位机里 `Base=20` 表示目标速度约为 `20 counts / 20 ms`。

## 当前功能

- 左右轮编码器计数和测速。
- 20 ms 周期速度闭环控制。
- 左右轮独立 PID 速度控制。
- 前馈补偿和左右轮前馈比例校准。
- UART0 串口遥测和在线调参，默认 `115200 8N1`，遥测周期 `100 ms`。
- Python 上位机曲线显示、PID 下发、CSV 保存。
- 基础巡线和运动命令接口。

## 关键控制量说明

| 名称 | 位置 | 作用 |
| --- | --- | --- |
| `BASE` | 上位机/串口命令 | 设置左右轮共同目标速度，单位 `counts/20ms` |
| `LD` | 遥测 | 左轮滤波后的实际速度，单位 `counts/20ms` |
| `RD` | 遥测 | 右轮滤波后的实际速度，单位 `counts/20ms` |
| `ERR` | 遥测 | `LD - RD`，显示左右轮速度差，不是目标速度误差 |
| `LO` | 遥测 | 左轮最终 PWM 输出，单位百分比 |
| `RO` | 遥测 | 右轮最终 PWM 输出，单位百分比 |
| `OUT` | 遥测 | `RO - LO`，左右轮输出差 |
| `Kp/Ki/Kd` | PID 参数 | 左右轮速度 PID 共用同一组参数 |

速度输出的核心公式在 `app/app_car_control.c`：

```text
PWM = 前馈输出 + PID(目标速度 - 实际速度)
```

前馈输出由这些宏控制：

```c
SPEED_LOOP_START_DUTY_PERCENT
SPEED_LOOP_FEED_FORWARD_PERCENT_PER_COUNT
LEFT_SPEED_FEED_FORWARD_SCALE
RIGHT_SPEED_FEED_FORWARD_SCALE
```

如果平均速度追不上目标，优先调 `SPEED_LOOP_FEED_FORWARD_PERCENT_PER_COUNT`；如果只有某一边轮子长期偏快/偏慢，优先调对应的 `LEFT/RIGHT_SPEED_FEED_FORWARD_SCALE`。

## 串口命令

```text
PID <kp> <ki> <kd>
BASE <speed_counts_per_period>
STOP
GET
ENCZERO
TELE 0|1
PWM <left_duty_percent> <right_duty_percent> [duration_ms]
MOTOR <duty_percent> [duration_ms]
PULSE <duty_percent> <duration_ms>
DRIVE <linear_mm_s> <angular_deg_s>
SPEED <linear_mm_s>
DIST <distance_mm> <linear_mm_s>
TURN <angle_deg> <angular_deg_s>
ANGLE <angle_deg> <angular_deg_s>
MSTOP
```

常用调试流程：

```text
TELE 1
BASE 20
PID 4.0 0.2 0.4
GET
STOP
```

## 上位机工具

```powershell
cd upper_pc
python -m pip install -r requirements.txt
python serial_tuner.py
```

曲线默认关注 `TARGET`、`LD`、`RD`、`ERR`。其中 `TARGET` 来自固件遥测里的 `BASE` 字段，上位机会自动除以 100 显示成人能读的数值。

## 引脚连接

| 功能 | 引脚 |
| --- | --- |
| 左轮 PWM / PWMA | PA8 |
| 左轮方向 / AIN1, AIN2 | PA25, PA31 |
| 右轮 PWM / PWMB | PB9 |
| 右轮方向 / BIN1, BIN2 | PB16, PB13 |
| TB6612 STBY | PA27 |
| 左轮编码器 A, B | PB2, PB3 |
| 右轮编码器 A, B | PB0, PB1 |
| UART0 TX, RX | PA10, PA11 |
| OpenMV UART2 TX, RX | PA23, PA24 |
| 状态 LED | PB22, PB26, PB27 |

## 文件架构

```text
gpio_toggle_output/
├─ .ccsproject                  CCS 工程描述文件，给 Code Composer Studio 识别工程用
├─ .cproject                    Eclipse/CDT C 工程配置，保存编译器、包含路径等设置
├─ .project                     Eclipse 工程元数据，保存工程名称和构建器信息
├─ .clangd                      clangd 配置，给代码补全/跳转工具使用
├─ gpio_toggle_output.syscfg    TI SysConfig 外设配置源文件，生成 ti_msp_dl_config.*
├─ main.c                       程序入口，初始化外设，调度控制周期、遥测周期和中断入口
├─ README.md                    当前工程说明文档
│
├─ app/                         应用层逻辑，决定小车“做什么”
│  ├─ app_car_control.c         小车主控制：速度 PID、前馈、串口命令、遥测、手动 PWM/MOTOR
│  ├─ app_car_control.h         小车主控制模块接口
│  ├─ app_line_follow.c         巡线控制：解析 OpenMV 的 LINE 数据，生成左右轮转向修正
│  ├─ app_line_follow.h         巡线模块接口
│  ├─ app_motion_control.c      运动命令：把速度/距离/角度命令换算成左右轮目标速度
│  └─ app_motion_control.h      运动控制模块接口
│
├─ bsp/                         板级支持层，封装具体外设芯片
│  ├─ bsp_tb6612.c              TB6612 电机驱动，控制方向脚和 PWM 输出
│  └─ bsp_tb6612.h              TB6612 驱动接口、PWM 最大值等宏定义
│
├─ hw/                          硬件驱动层，封装 MCU 外设
│  ├─ hw_encoder.c              左右轮编码器 GPIO 中断计数，处理正反方向极性
│  ├─ hw_encoder.h              编码器计数结构体和读取/清零接口
│  ├─ hw_uart.c                 UART0 调试串口，负责 PC 上位机命令接收和遥测发送
│  ├─ hw_uart.h                 UART0 调试串口接口
│  ├─ hw_openmv_uart.c          UART2 OpenMV 串口，只接收视觉端 LINE 数据
│  └─ hw_openmv_uart.h          OpenMV 串口接口
│
├─ mid/                         中间层算法，和具体硬件无关
│  ├─ mid_pid.c                 通用 PID 算法，包含积分限幅和输出限幅
│  └─ mid_pid.h                 PID 结构体和函数声明
│
├─ openmv/                      OpenMV 摄像头端脚本
│  └─ xunjitest1.py             图像二值化、线段/弯道识别，并通过 UART 发送 LINE 命令
│
├─ upper_pc/                    PC 上位机工具
│  ├─ serial_tuner.py           串口调参工具：连接 COM、发送 PID/BASE、绘图、保存 CSV、诊断串口速率
│  ├─ requirements.txt          上位机 Python 依赖，主要是 pyserial
│  ├─ README.md                 上位机单独使用说明
│  └─ __pycache__/              Python 运行生成的缓存目录，可以忽略
│
├─ targetConfigs/               CCS 调试/烧录目标配置
│  ├─ MSPM0G3507.ccxml          MSPM0G3507 LaunchPad 调试连接配置
│  └─ readme.txt                CCS 自动生成的 targetConfigs 说明
│
└─ Debug/                       CCS Debug 构建输出目录，可以重新生成
   ├─ gpio_toggle_output.out    CCS 调试/烧录常用固件文件
   ├─ gpio_toggle_output.hex    Intel HEX 固件文件
   ├─ gpio_toggle_output.map    链接 map 文件，用于查看内存占用和符号分布
   ├─ ti_msp_dl_config.c/.h     SysConfig 生成的外设初始化代码
   ├─ *.o / *.d                 编译中间文件和依赖文件
   └─ makefile / *.mk / *.opt   CCS 自动生成的构建脚本和编译选项
```

可以忽略的目录：

- `.git/`：Git 版本库内部数据。
- `.agents/`：Codex/自动化工具的工作数据。
- `.settings/`：Eclipse/CCS 本地设置。
- `.ti_appdata/`：TI 工具运行时缓存。
- `Debug/.clangd/`：clangd 索引缓存。

## 编译和烧录

在 CCS 中打开工程后，直接 Build Project，然后 Debug/Load 到 MSPM0G3507 LaunchPad。

构建产物路径：

```text
Debug/gpio_toggle_output.out
Debug/gpio_toggle_output.hex
```

如果使用当前仓库里的自动生成 makefile，也可以在 `Debug` 目录执行：

```powershell
gmake all
```

## 调参建议

- 平均速度低于目标：优先增大 `SPEED_LOOP_FEED_FORWARD_PERCENT_PER_COUNT`，不要只加 `Kp`。
- 速度到目标但震荡：适当降低 `Kp` 或增加一点 `Kd`。
- 长时间有静态误差：小幅增加 `Ki`。
- 左右轮一边长期偏快：微调 `LEFT_SPEED_FEED_FORWARD_SCALE` 或 `RIGHT_SPEED_FEED_FORWARD_SCALE`。
- `ERR` 是左右轮差速，越接近 0 说明左右轮越一致。
