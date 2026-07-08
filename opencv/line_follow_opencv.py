#!/usr/bin/env python3
import argparse
import time

import cv2
import numpy as np


# 摄像头处理分辨率；树莓派算力更高，使用 VGA 提高识别精度。
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
ERROR_OUTPUT_WIDTH = 320

# 摄像头是否旋转 180 度；如果调试画面上下左右反了，运行时加 --no-rotate。
CAMERA_ROTATED_180 = True

# 巡线识别区域：(左上角 x, 左上角 y, 宽, 高)，只在这块区域里找黑线。
ROI = (40, 70, 560, 390)

# 扫描步长；数值越小越细，但 CPU 占用越高。
ROW_STEP = 8
COL_STEP = 8

# 单行里被认为是“赛道黑线”的最小/最大宽度，过滤小噪声和大片阴影。
MIN_TRACK_WIDTH = 24
MAX_TRACK_WIDTH = 300

# 直线判断需要的有效点数量，以及这些点在 y 方向至少跨过多少像素。
MIN_STRAIGHT_POINTS = 5
MIN_STRAIGHT_SPAN = 110

# 取目标点时，如果附近点的 x 跨度太大，说明可能在弯道，就少取一点底部点。
STRAIGHT_MAX_X_SPAN = 140

# 希望小车对准的目标行，越靠下越看近处，越靠上越提前看远处。
TARGET_Y = 380

# 横向黑线最小宽度；用于判断是否出现 T 字、直角弯等横向特征。
TURN_MIN_HORIZONTAL_WIDTH = 140

# 竖向黑线最小高度；用于确认横线旁边确实连着竖线，不是孤立黑块。
TURN_MIN_VERTICAL_HEIGHT = 110

# 横线和竖线允许错开的像素范围，越大越宽松。
TURN_JOIN_MARGIN = 28

# 在横线边缘附近左右搜索竖线的范围。
TURN_EDGE_SEARCH = 92

# 竖线底部达到这个 y 值后，才认为拐弯已经足够近，可以准备发 LTURN。
TURN_READY_Y = 288

# 检测到拐弯但还没真正触发 LTURN 前，临时发送的偏差值。
TURN_ERROR = 150

# 触发 LTURN 后，连续重复发送多少帧，保证下位机能收到。
TURN_REPEAT_FRAMES = 8

# 一次拐弯后冷却多少帧，避免同一个路口被重复识别。
TURN_COOLDOWN_FRAMES = 32

# 拐弯 ready 状态需要连续保持多少帧，防止误触发。
TURN_READY_HOLD_FRAMES = 2

# ready 后再等待多少秒才真正发送 LTURN，用来让车再往前走一点。
# 原来 OpenMV 约 4 FPS 时 13 帧 ~= 3.25 秒；树莓派帧率更高，改用时间更稳定。
TURN_READY_DELAY_SECONDS = 2

# ready 后短暂丢失拐弯特征时，允许继续等待多少秒。
TURN_PENDING_LOST_SECONDS = 2

# LTURN 后稳定等待多少帧，这段时间发送 LINE 0 0。
TURN_SETTLE_FRAMES = 12

# 二值图腐蚀次数，用来去掉小噪点；太大会把细线腐蚀没。
NOISE_ERODE = 1
MORPH_CLOSE_ITER = 1

# 调试窗口里画中心线时的点间隔。
DRAW_STEP = 1

# 每隔多少帧打印一次 TX 日志。
PRINT_EVERY = 6


def clamp(value, low, high):
    return max(low, min(high, value))


def output_error_from_x(x):
    return int(round((x - (FRAME_WIDTH // 2)) * ERROR_OUTPUT_WIDTH / FRAME_WIDTH))


def row_runs(mask, y, x0, w, min_width):
    runs = []
    run_start = -1
    x_end = x0 + w

    for x in range(x0, x_end):
        if mask[y, x] > 0:
            if run_start < 0:
                run_start = x
        elif run_start >= 0:
            width = x - run_start
            if width >= min_width:
                runs.append((run_start, x - 1))
            run_start = -1

    if run_start >= 0:
        width = x_end - run_start
        if width >= min_width:
            runs.append((run_start, x_end - 1))

    return runs


def column_runs(mask, x, y0, h, min_height):
    runs = []
    run_start = -1
    y_end = y0 + h

    for y in range(y0, y_end):
        if mask[y, x] > 0:
            if run_start < 0:
                run_start = y
        elif run_start >= 0:
            height = y - run_start
            if height >= min_height:
                runs.append((run_start, y - 1))
            run_start = -1

    if run_start >= 0:
        height = y_end - run_start
        if height >= min_height:
            runs.append((run_start, y_end - 1))

    return runs


def best_track_run(runs, last_x):
    best = None
    best_score = -1_000_000

    for left, right in runs:
        width = right - left + 1
        if width < MIN_TRACK_WIDTH or width > MAX_TRACK_WIDTH:
            continue
        center = (left + right) // 2
        score = width * 5 - abs(center - last_x)
        if score > best_score:
            best_score = score
            best = (left, right, center, width)

    return best


def collect_centerline(mask, x0, y0, w, h, last_x):
    points = []
    current_x = last_x

    for y in range(y0 + h - 1, y0 - 1, -ROW_STEP):
        run = best_track_run(row_runs(mask, y, x0, w, MIN_TRACK_WIDTH), current_x)
        if run is None:
            continue

        _left, _right, center, _width = run
        current_x = center
        points.append((center, y))

    points.reverse()
    return points


def straight_target(points):
    if len(points) < MIN_STRAIGHT_POINTS:
        return None

    bottom_y = max(p[1] for p in points)
    top_y = min(p[1] for p in points)
    if bottom_y - top_y < MIN_STRAIGHT_SPAN:
        return None

    near = [p for p in points if abs(p[1] - TARGET_Y) <= 36]
    if len(near) < 2:
        near = points[-min(5, len(points)):]

    x_span = max(p[0] for p in near) - min(p[0] for p in near)
    if x_span > STRAIGHT_MAX_X_SPAN:
        near = points[-min(4, len(points)):]

    x_sum = sum(p[0] for p in near)
    y_sum = sum(p[1] for p in near)
    return (x_sum // len(near), y_sum // len(near))


def find_turn_feature(mask, x0, y0, w, h):
    best = None
    best_score = -1_000_000

    for y in range(y0, y0 + h, ROW_STEP):
        for h_left, h_right in row_runs(mask, y, x0, w, TURN_MIN_HORIZONTAL_WIDTH):
            h_width = h_right - h_left + 1
            if h_width < TURN_MIN_HORIZONTAL_WIDTH:
                continue

            for edge_x, direction in ((h_left, 1), (h_right, -1)):
                sx0 = max(x0, edge_x - TURN_EDGE_SEARCH)
                sx1 = min(x0 + w - 1, edge_x + TURN_EDGE_SEARCH)

                for x in range(sx0, sx1 + 1, COL_STEP):
                    for v_top, v_bottom in column_runs(
                        mask, x, y0, h, TURN_MIN_VERTICAL_HEIGHT
                    ):
                        join_ok = (
                            v_top <= y + TURN_JOIN_MARGIN
                            and v_bottom >= y - TURN_JOIN_MARGIN
                        )
                        if not join_ok:
                            continue

                        v_height = v_bottom - v_top + 1
                        score = h_width * 2 + v_height * 3 + v_bottom
                        if score > best_score:
                            best_score = score
                            best = (
                                direction,
                                x,
                                y,
                                h_left,
                                h_right,
                                v_top,
                                v_bottom,
                            )

    if best is None:
        return None

    direction, line_x, join_y, h_left, h_right, v_top, v_bottom = best
    target_x = clamp(line_x + direction * 170, 0, FRAME_WIDTH - 1)
    turn_angle = -direction * 90
    turn_ready = v_bottom >= TURN_READY_Y

    return {
        "err": direction * TURN_ERROR,
        "angle": turn_angle,
        "target": (target_x, join_y),
        "line": (h_left, join_y, h_right, join_y),
        "vertical": (line_x, v_top, line_x, v_bottom),
        "ready": turn_ready,
    }


class Sender:
    def __init__(self, port, baudrate, dry_run):
        self.serial = None
        self.dry_run = dry_run
        if port and not dry_run:
            try:
                import serial

                self.serial = serial.Serial(port, baudrate, timeout=0.05)
                print(f"serial opened: {port} @ {baudrate}")
            except Exception as exc:
                print(f"serial disabled: {exc}")
                print("install pyserial or run with --dry-run while debugging")

    def write(self, message):
        if self.serial is not None:
            self.serial.write((message + "\n").encode("ascii"))


def make_mask(frame, threshold):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)
    if MORPH_CLOSE_ITER > 0 or NOISE_ERODE > 0:
        kernel = np.ones((3, 3), np.uint8)
    if MORPH_CLOSE_ITER > 0:
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, kernel, iterations=MORPH_CLOSE_ITER
        )
    if NOISE_ERODE > 0:
        mask = cv2.erode(mask, kernel, iterations=NOISE_ERODE)
    return mask


def open_camera(device):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    return cap


def draw_debug(frame, mask, points, target, turn, message, mode, fps):
    x0, y0, w, h = ROI
    cv2.rectangle(frame, (x0, y0), (x0 + w, y0 + h), (80, 220, 80), 1)

    if turn is not None:
        cv2.line(
            frame,
            (turn["line"][0], turn["line"][1]),
            (turn["line"][2], turn["line"][3]),
            (0, 255, 0),
            3,
        )
        cv2.line(
            frame,
            (turn["vertical"][0], turn["vertical"][1]),
            (turn["vertical"][2], turn["vertical"][3]),
            (0, 255, 255),
            3,
        )
        cv2.drawMarker(frame, turn["target"], (0, 0, 255), cv2.MARKER_CROSS, 12, 2)
    else:
        for i in range(0, len(points) - DRAW_STEP, DRAW_STEP):
            cv2.line(frame, points[i], points[i + DRAW_STEP], (255, 0, 0), 2)
        if target is not None:
            cv2.drawMarker(frame, target, (0, 0, 255), cv2.MARKER_CROSS, 12, 2)

    cv2.putText(
        frame,
        f"{message} M={mode} P={len(points)} FPS={fps:.1f}",
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.imshow("line-follow", frame)
    cv2.imshow("line-mask", mask)


def parse_args():
    parser = argparse.ArgumentParser(description="OpenCV line follower for Raspberry Pi")
    parser.add_argument("--device", default="/dev/video0", help="camera device")
    parser.add_argument("--serial", default="/dev/ttyAMA0", help="serial port")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--threshold", type=int, default=60, help="black line threshold")
    parser.add_argument("--show", dest="show", action="store_true", default=True, help="show debug windows")
    parser.add_argument("--no-show", dest="show", action="store_false", help="disable debug windows")
    parser.add_argument("--dry-run", action="store_true", help="print only, do not use serial")
    parser.add_argument("--no-rotate", action="store_true", help="disable 180 degree rotation")
    return parser.parse_args()


def main():
    args = parse_args()
    sender = Sender(args.serial, args.baudrate, args.dry_run)
    cap = open_camera(args.device)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open camera: {args.device}")

    frame_count = 0
    last_x = FRAME_WIDTH // 2
    turn_ready_count = 0
    turn_ready_at = None
    turn_cooldown = 0
    turn_repeat_count = 0
    turn_repeat_angle = 0
    turn_settle_count = 0
    turn_pending = False
    turn_pending_angle = 0
    turn_pending_lost_at = None
    last_time = time.monotonic()
    fps = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("camera read failed")
                time.sleep(0.02)
                continue

            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
            if CAMERA_ROTATED_180 and not args.no_rotate:
                frame = cv2.rotate(frame, cv2.ROTATE_180)

            frame_count += 1
            now = time.monotonic()
            dt = now - last_time
            last_time = now
            if dt > 0:
                fps = fps * 0.85 + (1.0 / dt) * 0.15

            if turn_cooldown > 0:
                turn_cooldown -= 1

            if turn_repeat_count > 0:
                turn_repeat_count -= 1
                tx_message = f"LTURN {turn_repeat_angle}"
                sender.write(tx_message)
                if turn_repeat_count == 0:
                    turn_settle_count = TURN_SETTLE_FRAMES
                if (frame_count % PRINT_EVERY) == 0:
                    print(f"TX: {tx_message} M=REPEAT FPS: {fps:.1f}")
                continue

            if turn_settle_count > 0:
                turn_settle_count -= 1
                last_x = FRAME_WIDTH // 2
                turn_ready_count = 0
                turn_ready_at = None
                turn_pending = False
                turn_pending_lost_at = None
                tx_message = "LINE 0 0"
                sender.write(tx_message)
                if (frame_count % PRINT_EVERY) == 0:
                    print(f"TX: {tx_message} M=SETTLE FPS: {fps:.1f}")
                continue

            mask = make_mask(frame, args.threshold)
            x0, y0, w, h = ROI
            turn = find_turn_feature(mask, x0, y0, w, h)
            points = collect_centerline(mask, x0, y0, w, h, last_x)
            target = straight_target(points)

            if turn is not None:
                if turn["ready"]:
                    turn_ready_count += 1
                    turn_pending_lost_at = None
                    if turn_ready_count >= TURN_READY_HOLD_FRAMES:
                        if not turn_pending:
                            turn_ready_at = now
                        turn_pending = True
                        turn_pending_angle = turn["angle"]
                elif not turn_pending:
                    turn_ready_count = 0
                    turn_ready_at = None

                turn_delay_elapsed = (
                    turn_pending
                    and turn_cooldown == 0
                    and turn_ready_at is not None
                    and (now - turn_ready_at) >= TURN_READY_DELAY_SECONDS
                )

                if turn_delay_elapsed:
                    turn_repeat_angle = turn_pending_angle
                    turn_repeat_count = TURN_REPEAT_FRAMES
                    turn_cooldown = TURN_COOLDOWN_FRAMES
                    turn_ready_count = 0
                    turn_ready_at = None
                    turn_pending = False
                    turn_pending_lost_at = None
                    tx_message = f"LTURN {turn_repeat_angle}"
                else:
                    tx_message = "LINE 0 0"

                last_x = turn["target"][0]
                sender.write(tx_message)
                mode = "L"
            elif turn_pending:
                if turn_pending_lost_at is None:
                    turn_pending_lost_at = now

                lost_elapsed = now - turn_pending_lost_at
                if lost_elapsed > TURN_PENDING_LOST_SECONDS or turn_cooldown != 0:
                    turn_pending = False
                    turn_ready_count = 0
                    turn_ready_at = None
                    turn_pending_lost_at = None

                turn_delay_elapsed = (
                    turn_pending
                    and turn_ready_at is not None
                    and (now - turn_ready_at) >= TURN_READY_DELAY_SECONDS
                )

                if turn_delay_elapsed:
                    turn_repeat_angle = turn_pending_angle
                    turn_repeat_count = TURN_REPEAT_FRAMES
                    turn_cooldown = TURN_COOLDOWN_FRAMES
                    turn_pending = False
                    turn_ready_count = 0
                    turn_ready_at = None
                    turn_pending_lost_at = None
                    tx_message = f"LTURN {turn_repeat_angle}"
                elif target is not None:
                    last_x = FRAME_WIDTH // 2
                    tx_message = "LINE 0 0"
                else:
                    last_x = FRAME_WIDTH // 2
                    tx_message = "LINE 0 0"

                sender.write(tx_message)
                mode = "P"
            elif target is not None:
                turn_ready_count = 0
                turn_ready_at = None
                turn_pending_lost_at = None
                last_x = target[0]
                err = output_error_from_x(target[0])
                tx_message = f"LINE 1 {err}"
                sender.write(tx_message)
                mode = "S"
            else:
                turn_ready_count = 0
                turn_ready_at = None
                turn_pending_lost_at = None
                last_x = FRAME_WIDTH // 2
                tx_message = "LINE 0 0"
                sender.write(tx_message)
                mode = "N"

            if (frame_count % PRINT_EVERY) == 0:
                print(f"TX: {tx_message} M={mode} P={len(points)} FPS: {fps:.1f}")

            if args.show:
                draw_debug(frame, mask, points, target, turn, tx_message, mode, fps)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
