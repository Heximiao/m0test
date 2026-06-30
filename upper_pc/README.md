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

The UI sends newline-terminated ASCII commands:

```text
PID 0.35 0.02 0.08
BASE 8.0
GET
```
