import sensor
import time
from pyb import UART

BLACK_THRESHOLD = (0, 40, -20, 20, -20, 20)
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
UART_BAUDRATE = 115200
CAMERA_ROTATED_180 = True

ROI = (20, 35, 280, 195)
ROW_STEP = 6
COL_STEP = 6
MIN_TRACK_WIDTH = 12
MAX_TRACK_WIDTH = 150
MIN_STRAIGHT_POINTS = 5
MIN_STRAIGHT_SPAN = 55
STRAIGHT_MAX_X_SPAN = 70
TARGET_Y = 190

TURN_MIN_HORIZONTAL_WIDTH = 70
TURN_MIN_VERTICAL_HEIGHT = 55
TURN_JOIN_MARGIN = 14
TURN_EDGE_SEARCH = 46
TURN_READY_Y = 194
TURN_ERROR = 150
TURN_REPEAT_FRAMES = 8
TURN_COOLDOWN_FRAMES = 32
TURN_READY_HOLD_FRAMES = 2
TURN_READY_DELAY_FRAMES = 10
TURN_PENDING_LOST_FRAMES = 10
TURN_SETTLE_FRAMES = 12

NOISE_ERODE = 1
DRAW_STEP = 1
PRINT_EVERY = 6


def _is_track_pixel(pixel):
    return pixel[0] > 128 if isinstance(pixel, tuple) else pixel > 128


def _clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def _row_runs(img, y, x0, w, min_width):
    runs = []
    run_start = -1
    x_end = x0 + w

    for x in range(x0, x_end):
        if _is_track_pixel(img.get_pixel(x, y)):
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


def _column_runs(img, x, y0, h, min_height):
    runs = []
    run_start = -1
    y_end = y0 + h

    for y in range(y0, y_end):
        if _is_track_pixel(img.get_pixel(x, y)):
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


def _best_track_run(runs, last_x):
    best = None
    best_score = -1000000

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


def _collect_centerline(img, x0, y0, w, h, last_x):
    points = []
    current_x = last_x

    for y in range(y0 + h - 1, y0 - 1, -ROW_STEP):
        run = _best_track_run(_row_runs(img, y, x0, w, MIN_TRACK_WIDTH),
                              current_x)
        if run is None:
            continue

        left, right, center, width = run
        current_x = center
        points.append((center, y))

    points.reverse()
    return points


def _straight_target(points):
    if len(points) < MIN_STRAIGHT_POINTS:
        return None

    bottom_y = max(p[1] for p in points)
    top_y = min(p[1] for p in points)
    if bottom_y - top_y < MIN_STRAIGHT_SPAN:
        return None

    near = [p for p in points if abs(p[1] - TARGET_Y) <= 18]
    if len(near) < 2:
        near = points[-min(5, len(points)):]

    x_span = max(p[0] for p in near) - min(p[0] for p in near)
    if x_span > STRAIGHT_MAX_X_SPAN:
        near = points[-min(4, len(points)):]

    x_sum = sum(p[0] for p in near)
    y_sum = sum(p[1] for p in near)
    return (x_sum // len(near), y_sum // len(near))


def _find_turn_feature(img, x0, y0, w, h):
    best = None
    best_score = -1000000

    for y in range(y0, y0 + h, ROW_STEP):
        for h_left, h_right in _row_runs(
                img, y, x0, w, TURN_MIN_HORIZONTAL_WIDTH):
            h_width = h_right - h_left + 1
            if h_width < TURN_MIN_HORIZONTAL_WIDTH:
                continue

            for edge_x, direction in ((h_left, 1), (h_right, -1)):
                sx0 = max(x0, edge_x - TURN_EDGE_SEARCH)
                sx1 = min(x0 + w - 1, edge_x + TURN_EDGE_SEARCH)

                for x in range(sx0, sx1 + 1, COL_STEP):
                    for v_top, v_bottom in _column_runs(
                            img, x, y0, h, TURN_MIN_VERTICAL_HEIGHT):
                        join_ok = (
                            v_top <= y + TURN_JOIN_MARGIN and
                            v_bottom >= y - TURN_JOIN_MARGIN)
                        if not join_ok:
                            continue

                        v_height = v_bottom - v_top + 1
                        score = h_width * 2 + v_height * 3 + v_bottom
                        if score > best_score:
                            best_score = score
                            best = (
                                direction, x, y, h_left, h_right,
                                v_top, v_bottom)

    if best is None:
        return None

    direction, line_x, join_y, h_left, h_right, v_top, v_bottom = best
    target_x = _clamp(line_x + direction * 85, 0, FRAME_WIDTH - 1)
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


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
if CAMERA_ROTATED_180:
    sensor.set_hmirror(True)
    sensor.set_vflip(True)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

uart = UART(3, UART_BAUDRATE, timeout_char=1000)
clock = time.clock()

frame_count = 0
last_x = FRAME_WIDTH // 2
turn_ready_count = 0
turn_delay_count = 0
turn_cooldown = 0
turn_repeat_count = 0
turn_repeat_angle = 0
turn_settle_count = 0
turn_pending = False
turn_pending_angle = 0
turn_pending_lost_count = 0

while True:
    clock.tick()
    frame_count += 1

    if turn_cooldown > 0:
        turn_cooldown -= 1

    if turn_repeat_count > 0:
        turn_repeat_count -= 1
        tx_message = "LTURN %d" % turn_repeat_angle
        uart.write(tx_message + "\n")
        if turn_repeat_count == 0:
            turn_settle_count = TURN_SETTLE_FRAMES
        if (frame_count % PRINT_EVERY) == 0:
            print("TX:", tx_message, "M=REPEAT FPS:", clock.fps())
        continue

    if turn_settle_count > 0:
        turn_settle_count -= 1
        last_x = FRAME_WIDTH // 2
        turn_ready_count = 0
        turn_delay_count = 0
        turn_pending = False
        turn_pending_lost_count = 0
        tx_message = "LINE 0 0"
        uart.write(tx_message + "\n")
        if (frame_count % PRINT_EVERY) == 0:
            print("TX:", tx_message, "M=SETTLE FPS:", clock.fps())
        continue

    img = sensor.snapshot()
    img.binary([BLACK_THRESHOLD])
    if NOISE_ERODE > 0:
        img.erode(NOISE_ERODE)

    x0, y0, w, h = ROI
    turn = _find_turn_feature(img, x0, y0, w, h)
    points = _collect_centerline(img, x0, y0, w, h, last_x)
    target = _straight_target(points)

    if turn is not None:
        if turn["ready"]:
            turn_ready_count += 1
            turn_pending_lost_count = 0
            if turn_ready_count >= TURN_READY_HOLD_FRAMES:
                turn_pending = True
                turn_pending_angle = turn["angle"]
        elif not turn_pending:
            turn_ready_count = 0
            turn_delay_count = 0

        if turn_pending and turn_cooldown == 0:
            turn_delay_count += 1
        else:
            turn_delay_count = 0

        if turn_pending and turn_delay_count >= TURN_READY_DELAY_FRAMES:
            turn_repeat_angle = turn_pending_angle
            turn_repeat_count = TURN_REPEAT_FRAMES
            turn_cooldown = TURN_COOLDOWN_FRAMES
            turn_ready_count = 0
            turn_delay_count = 0
            turn_pending = False
            turn_pending_lost_count = 0
            tx_message = "LTURN %d" % turn_repeat_angle
        else:
            tx_message = "LINE 1 %d" % turn["err"]

        last_x = turn["target"][0]
        uart.write(tx_message + "\n")
        img.draw_line(turn["line"], color=(0, 255, 0), thickness=3)
        img.draw_line(turn["vertical"], color=(0, 255, 255), thickness=3)
        img.draw_cross(turn["target"][0], turn["target"][1],
                       color=(255, 0, 0), size=8)
        mode = "L"
    elif turn_pending:
        turn_pending_lost_count += 1
        if (turn_pending_lost_count <= TURN_PENDING_LOST_FRAMES and
                turn_cooldown == 0):
            turn_delay_count += 1
        else:
            turn_pending = False
            turn_ready_count = 0
            turn_delay_count = 0
            turn_pending_lost_count = 0

        if turn_pending and turn_delay_count >= TURN_READY_DELAY_FRAMES:
            turn_repeat_angle = turn_pending_angle
            turn_repeat_count = TURN_REPEAT_FRAMES
            turn_cooldown = TURN_COOLDOWN_FRAMES
            turn_pending = False
            turn_ready_count = 0
            turn_delay_count = 0
            turn_pending_lost_count = 0
            tx_message = "LTURN %d" % turn_repeat_angle
        elif target is not None:
            last_x = target[0]
            err = target[0] - (FRAME_WIDTH // 2)
            tx_message = "LINE 1 %d" % err
        else:
            last_x = FRAME_WIDTH // 2
            tx_message = "LINE 0 0"

        uart.write(tx_message + "\n")
        mode = "P"
    elif target is not None:
        turn_ready_count = 0
        turn_delay_count = 0
        turn_pending_lost_count = 0
        last_x = target[0]
        err = target[0] - (FRAME_WIDTH // 2)
        tx_message = "LINE 1 %d" % err
        uart.write(tx_message + "\n")

        for i in range(0, len(points) - DRAW_STEP, DRAW_STEP):
            img.draw_line(points[i][0], points[i][1],
                          points[i + DRAW_STEP][0],
                          points[i + DRAW_STEP][1],
                          color=(255, 0, 0), thickness=2)
        img.draw_cross(target[0], target[1], color=(255, 0, 0), size=8)
        mode = "S"
    else:
        turn_ready_count = 0
        turn_delay_count = 0
        turn_pending_lost_count = 0
        last_x = FRAME_WIDTH // 2
        tx_message = "LINE 0 0"
        uart.write(tx_message + "\n")
        mode = "N"

    if (frame_count % PRINT_EVERY) == 0:
        print("TX:", tx_message, "M=%s P=%d FPS:" % (mode, len(points)),
              clock.fps())
