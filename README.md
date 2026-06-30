# MSPM0 PID Car Controller

This CCS project controls a two-wheel TB6612 car on an MSPM0G3507 LaunchPad.
It provides:

- Quadrature Hall encoder counting for left and right wheels.
- 20 ms speed-loop updates with filtered wheel-speed feedback.
- Independent left/right speed PID control.
- UART0 telemetry and live PID tuning at 115200 8N1.
- A Python serial tuner in `upper_pc/`.

## Source Layout

| Path | Purpose |
| --- | --- |
| `main.c` | Entry point, scheduler, interrupt dispatch |
| `app/` | Car behavior, line following, motion commands |
| `bsp/` | Board-level motor driver |
| `hw/` | Encoder and UART drivers |
| `mid/` | Reusable control logic such as PID |

## Current Wiring

| Function | Pin |
| --- | --- |
| Left PWM / PWMA | PA8 |
| Left direction / AIN1, AIN2 | PA25, PA31 |
| Right PWM / PWMB | PB9 |
| Right direction / BIN1, BIN2 | PB16, PB13 |
| TB6612 STBY | PA27 |
| Left encoder A, B | PB2, PB3 |
| Right encoder A, B | PB0, PB1 |
| UART0 TX, RX | PA10, PA11 |
| Status LED | PB22, PB26, PB27 |

## UART Commands

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

`BASE` sets the shared speed target in encoder counts per 20 ms control period.
The tuned speed-loop defaults live in `app/app_car_control.c`.

The motion commands in `app/app_motion_control.c` wrap the wheel speed PID into
robot units. The defaults assume MG310 motors, 1:20 reduction, 11 PPR motor
encoder, two counted encoder edges per motor pulse, 48 mm wheels, and a 120 mm
wheelbase. Measure the actual wheelbase and encoder counts per wheel revolution
on the car and update the constants there for accurate distance/angle control.

## Upper PC Tool

```powershell
cd upper_pc
python -m pip install -r requirements.txt
python serial_tuner.py
```

The plot uses `TARGET`, `LD`, `RD`, and `ERR`. `L` and `R` are cumulative
encoder counts and are useful for diagnostics, not speed-loop tuning.
