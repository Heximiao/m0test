import sensor
import time
from pyb import UART

BLACK_THRESHOLD = (0, 25, -20, 20, -20, 20)
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
ARC_ROI = (40, 55, 250, 165)
SCAN_STEP_X = 5
SCAN_STEP_Y = 5
MIN_RUN_HEIGHT = 4
MAX_RUN_HEIGHT = 34
MIN_RUN_WIDTH = 4
MAX_RUN_WIDTH = 38
MIN_ARC_POINTS = 12
MIN_ARC_WIDTH = 80
MIN_ARC_BULGE = 12
MIN_STEEP_POINTS = 14
MIN_STEEP_HEIGHT = 70
CROSS_MIN_HORIZONTAL_WIDTH = 100
CROSS_MIN_ROWS = 3
CROSS_LINE_OVERLAP_PIXELS = 22
CROSS_VERTICAL_MARGIN = 15
ARC_LOOKAHEAD_PIXELS = 95
RIGHT_ANGLE_MIN_HORIZONTAL_WIDTH = 80
RIGHT_ANGLE_CORNER_MARGIN = 45
RIGHT_ANGLE_LINE_OVERLAP_PIXELS = 30
RIGHT_ANGLE_TURN_ERROR_PIXELS = 150
RIGHT_ANGLE_TURN_TRIGGER_Y = 185
RIGHT_ANGLE_TURN_COOLDOWN_FRAMES = 35
RIGHT_ANGLE_TURN_REPEAT_FRAMES = 8
RIGHT_ANGLE_CONFIRM_FRAMES = 8
RIGHT_ANGLE_MIN_VERTICAL_HEIGHT = 28
RIGHT_ANGLE_ENDPOINT_SEARCH_PIXELS = 35
RIGHT_ANGLE_FEATURE_MIN_VERTICAL_HEIGHT = 55
RIGHT_ANGLE_FEATURE_MIN_JOIN_PIXELS = 12
RIGHT_ANGLE_FEATURE_EDGE_MARGIN = 45
FAST_SAMPLE_X = 4
FAST_SAMPLE_Y = 6
FAST_RIGHT_ANGLE_MIN_HORIZONTAL_WIDTH = 70
FAST_RIGHT_ANGLE_MIN_VERTICAL_HEIGHT = 35
MAX_POINT_DY = 16
MAX_POINT_DX = 18
MAX_GAP_X = 15
MAX_GAP_Y = 15
NOISE_ERODE = 1
DRAW_STEP = 2
PRINT_EVERY = 6
UART_BAUDRATE = 115200
CAMERA_ROTATED_180 = True


def is_white_pixel(pixel):
    return pixel[0] > 128 if isinstance(pixel, tuple) else pixel > 128


def find_column_candidates(img, x, y0, h):
    runs = []
    run_start = -1

    for y in range(y0, y0 + h):
        if is_white_pixel(img.get_pixel(x, y)):
            if run_start < 0:
                run_start = y
        elif run_start >= 0:
            run_height = y - run_start
            if MIN_RUN_HEIGHT <= run_height <= MAX_RUN_HEIGHT:
                runs.append((run_start + y - 1) // 2)
            run_start = -1

    if run_start >= 0:
        run_height = (y0 + h) - run_start
        if MIN_RUN_HEIGHT <= run_height <= MAX_RUN_HEIGHT:
            runs.append((run_start + y0 + h - 1) // 2)

    return runs


def find_row_candidates(img, y, x0, w):
    runs = []
    run_start = -1

    for x in range(x0, x0 + w):
        if is_white_pixel(img.get_pixel(x, y)):
            if run_start < 0:
                run_start = x
        elif run_start >= 0:
            run_width = x - run_start
            if MIN_RUN_WIDTH <= run_width <= MAX_RUN_WIDTH:
                runs.append((run_start + x - 1) // 2)
            run_start = -1

    if run_start >= 0:
        run_width = (x0 + w) - run_start
        if MIN_RUN_WIDTH <= run_width <= MAX_RUN_WIDTH:
            runs.append((run_start + x0 + w - 1) // 2)

    return runs


def find_long_row_runs(img, y, x0, w):
    runs = []
    run_start = -1

    for x in range(x0, x0 + w):
        if is_white_pixel(img.get_pixel(x, y)):
            if run_start < 0:
                run_start = x
        elif run_start >= 0:
            run_width = x - run_start
            if run_width >= CROSS_MIN_HORIZONTAL_WIDTH:
                runs.append((run_start, x - 1))
            run_start = -1

    if run_start >= 0:
        run_width = (x0 + w) - run_start
        if run_width >= CROSS_MIN_HORIZONTAL_WIDTH:
            runs.append((run_start, x0 + w - 1))

    return runs


def find_long_column_runs(img, x, y0, h, min_height):
    runs = []
    run_start = -1

    for y in range(y0, y0 + h):
        if is_white_pixel(img.get_pixel(x, y)):
            if run_start < 0:
                run_start = y
        elif run_start >= 0:
            run_height = y - run_start
            if run_height >= min_height:
                runs.append((run_start, y - 1))
            run_start = -1

    if run_start >= 0:
        run_height = (y0 + h) - run_start
        if run_height >= min_height:
            runs.append((run_start, y0 + h - 1))

    return runs


def find_smooth_path(columns):
    nodes = []
    previous_nodes = []
    best_index = -1
    best_score = -1000000

    for x, ys in columns:
        current_nodes = []

        for y in ys:
            count = 1
            score = 100
            best_prev = -1

            for prev_index in previous_nodes:
                prev = nodes[prev_index]
                dx = x - prev[0]
                dy = abs(y - prev[1])

                if dx <= 0 or dx > MAX_GAP_X or dy > MAX_POINT_DY:
                    continue

                new_score = prev[3] + 100 - (dy * 3) - (dx - SCAN_STEP_X)
                if new_score > score:
                    score = new_score
                    count = prev[2] + 1
                    best_prev = prev_index

            nodes.append((x, y, count, score, best_prev))
            node_index = len(nodes) - 1
            current_nodes.append(node_index)

            if score > best_score:
                best_score = score
                best_index = node_index

        if current_nodes:
            previous_nodes = current_nodes

    points = []
    while best_index >= 0:
        node = nodes[best_index]
        points.append((node[0], node[1]))
        best_index = node[4]

    points.reverse()
    return points


def find_vertical_path(rows):
    nodes = []
    previous_nodes = []
    best_index = -1
    best_score = -1000000

    for y, xs in rows:
        current_nodes = []

        for x in xs:
            count = 1
            score = 100
            best_prev = -1

            for prev_index in previous_nodes:
                prev = nodes[prev_index]
                dy = y - prev[1]
                dx = abs(x - prev[0])

                if dy <= 0 or dy > MAX_GAP_Y or dx > MAX_POINT_DX:
                    continue

                new_score = prev[3] + 100 - (dx * 3) - (dy - SCAN_STEP_Y)
                if new_score > score:
                    score = new_score
                    count = prev[2] + 1
                    best_prev = prev_index

            nodes.append((x, y, count, score, best_prev))
            node_index = len(nodes) - 1
            current_nodes.append(node_index)

            if score > best_score:
                best_score = score
                best_index = node_index

        if current_nodes:
            previous_nodes = current_nodes

    points = []
    while best_index >= 0:
        node = nodes[best_index]
        points.append((node[0], node[1]))
        best_index = node[4]

    points.reverse()
    return points


def is_arc_like(points):
    if len(points) < MIN_ARC_POINTS:
        return False

    x_span = points[-1][0] - points[0][0]
    if x_span < MIN_ARC_WIDTH:
        return False

    left_y = points[0][1]
    right_y = points[-1][1]
    mid_y = min(p[1] for p in points)
    edge_y = (left_y + right_y) // 2

    return (edge_y - mid_y) >= MIN_ARC_BULGE


def is_steep_like(points):
    if len(points) < MIN_STEEP_POINTS:
        return False

    y_span = points[-1][1] - points[0][1]
    if y_span < MIN_STEEP_HEIGHT:
        return False

    x_min = min(p[0] for p in points)
    x_max = max(p[0] for p in points)

    return (x_max - x_min) <= MAX_RUN_WIDTH * 3


def control_point(points):
    target_y = max(p[1] for p in points)
    near_points = [p for p in points if target_y - p[1] <= 18]
    x_sum = sum(p[0] for p in near_points)
    y_sum = sum(p[1] for p in near_points)
    return (x_sum // len(near_points), y_sum // len(near_points))


def arc_control_point(points):
    near_y = max(p[1] for p in points)
    target_y = near_y - ARC_LOOKAHEAD_PIXELS
    lookahead_points = [p for p in points if abs(p[1] - target_y) <= 18]

    if len(lookahead_points) < 3:
        lookahead_points = points[:max(3, len(points) // 3)]

    x_sum = sum(p[0] for p in lookahead_points)
    y_sum = sum(p[1] for p in lookahead_points)
    return (x_sum // len(lookahead_points), y_sum // len(lookahead_points))


def is_cross_like(img, vertical_points, x0, y0, w, h):
    if not is_steep_like(vertical_points):
        return False

    line_x = sum(p[0] for p in vertical_points) // len(vertical_points)
    cross_rows = []

    for y in range(y0, y0 + h, SCAN_STEP_Y):
        for left_x, right_x in find_long_row_runs(img, y, x0, w):
            if (left_x - CROSS_LINE_OVERLAP_PIXELS <= line_x and
                    right_x + CROSS_LINE_OVERLAP_PIXELS >= line_x):
                cross_rows.append(y)
                break

    if len(cross_rows) < CROSS_MIN_ROWS:
        return False

    cross_y = sum(cross_rows) // len(cross_rows)
    has_before_cross = False
    has_after_cross = False

    for point_x, point_y in vertical_points:
        if abs(point_x - line_x) > CROSS_LINE_OVERLAP_PIXELS:
            continue
        if point_y < cross_y - CROSS_VERTICAL_MARGIN:
            has_before_cross = True
        if point_y > cross_y + CROSS_VERTICAL_MARGIN:
            has_after_cross = True

    return has_before_cross and has_after_cross


def fast_right_angle_turn(img, x0, y0, w, h):
    best = None
    best_score = -1000000

    for y in range(y0, y0 + h, FAST_SAMPLE_Y):
        runs = find_long_row_runs(img, y, x0, w)
        for h_left, h_right in runs:
            h_width = h_right - h_left + 1
            if h_width < FAST_RIGHT_ANGLE_MIN_HORIZONTAL_WIDTH:
                continue

            for edge_x, direction in ((h_left, 1), (h_right, -1)):
                search_left = max(x0, edge_x - RIGHT_ANGLE_FEATURE_EDGE_MARGIN)
                search_right = min(x0 + w - 1,
                    edge_x + RIGHT_ANGLE_FEATURE_EDGE_MARGIN)

                for x in range(search_left, search_right + 1, FAST_SAMPLE_X):
                    white_count = 0
                    bottom_y = y
                    for sample_y in range(y, y0 + h, FAST_SAMPLE_Y):
                        if is_white_pixel(img.get_pixel(x, sample_y)):
                            white_count += FAST_SAMPLE_Y
                            bottom_y = sample_y

                    if white_count < FAST_RIGHT_ANGLE_MIN_VERTICAL_HEIGHT:
                        continue

                    score = h_width + white_count + bottom_y
                    if score > best_score:
                        best_score = score
                        best = (direction, x, y, h_left, h_right, bottom_y)

    if best is None:
        return (
            False, 0, 0, (FRAME_WIDTH // 2, y0 + h - 1),
            (0, 0, y0), False)

    direction, line_x, join_y, h_left, h_right, bottom_y = best
    target_x = line_x + (direction * 90)
    if target_x < 0:
        target_x = 0
    elif target_x >= FRAME_WIDTH:
        target_x = FRAME_WIDTH - 1

    return (
        True,
        direction * RIGHT_ANGLE_TURN_ERROR_PIXELS,
        -direction * 90,
        (target_x, join_y),
        (h_left, h_right, join_y),
        bottom_y >= RIGHT_ANGLE_TURN_TRIGGER_Y)


def right_angle_feature(img, x0, y0, w, h):
    best = None
    best_score = -1000000

    for y in range(y0, y0 + h, SCAN_STEP_Y):
        for h_left, h_right in find_long_row_runs(img, y, x0, w):
            h_width = h_right - h_left + 1
            if h_width < RIGHT_ANGLE_MIN_HORIZONTAL_WIDTH:
                continue

            search_ranges = (
                (max(x0, h_left - RIGHT_ANGLE_FEATURE_EDGE_MARGIN),
                    min(x0 + w - 1, h_left + RIGHT_ANGLE_FEATURE_EDGE_MARGIN)),
                (max(x0, h_right - RIGHT_ANGLE_FEATURE_EDGE_MARGIN),
                    min(x0 + w - 1, h_right + RIGHT_ANGLE_FEATURE_EDGE_MARGIN)))

            for range_index, search_range in enumerate(search_ranges):
                search_left, search_right = search_range
                direction = 1 if range_index == 0 else -1

                for x in range(search_left, search_right + 1, SCAN_STEP_X):
                    for v_top, v_bottom in find_long_column_runs(
                            img, x, y0, h,
                            RIGHT_ANGLE_FEATURE_MIN_VERTICAL_HEIGHT):
                        join_ok = (
                            v_top - RIGHT_ANGLE_FEATURE_MIN_JOIN_PIXELS <= y and
                            v_bottom + RIGHT_ANGLE_FEATURE_MIN_JOIN_PIXELS >= y)
                        if not join_ok:
                            continue

                        score = h_width + (v_bottom - v_top + 1) + v_bottom
                        if score > best_score:
                            best_score = score
                            best = (
                                direction, x, y, h_left, h_right, v_top,
                                v_bottom)

    if best is None:
        return (
            False, 0, 0, (FRAME_WIDTH // 2, y0 + h - 1),
            (0, 0, y0), False)

    direction, line_x, join_y, h_left, h_right, v_top, v_bottom = best
    target_x = line_x + (direction * 90)
    if target_x < 0:
        target_x = 0
    elif target_x >= FRAME_WIDTH:
        target_x = FRAME_WIDTH - 1

    turn_angle = -direction * 90
    turn_ready = (v_bottom >= RIGHT_ANGLE_TURN_TRIGGER_Y)

    return (
        True,
        direction * RIGHT_ANGLE_TURN_ERROR_PIXELS,
        turn_angle,
        (target_x, join_y),
        (h_left, h_right, join_y),
        turn_ready)


def right_angle_turn(img, vertical_points, x0, y0, w, h):
    feature_valid, feature_err, feature_angle, feature_target, \
        feature_run, feature_ready = right_angle_feature(img, x0, y0, w, h)

    if feature_valid:
        return (
            feature_valid, feature_err, feature_angle,
            feature_target, feature_run, feature_ready)

    best_run = None
    best_score = -1000000

    for y in range(y0, y0 + h, SCAN_STEP_Y):
        for left_x, right_x in find_long_row_runs(img, y, x0, w):
            run_width = right_x - left_x + 1
            if run_width < RIGHT_ANGLE_MIN_HORIZONTAL_WIDTH:
                continue

            if y < y0 or y > (y0 + h):
                continue

            score = run_width + y
            if score > best_score:
                best_score = score
                best_run = (left_x, right_x, y)

    if best_run is None:
        return (
            False, 0, 0, (FRAME_WIDTH // 2, y0 + h - 1),
            (0, 0, y0), False)

    left_x, right_x, turn_y = best_run
    candidates = []

    for edge_x, direction in ((left_x, -1), (right_x, 1)):
        search_left = max(x0, edge_x - RIGHT_ANGLE_ENDPOINT_SEARCH_PIXELS)
        search_right = min(x0 + w - 1, edge_x + RIGHT_ANGLE_ENDPOINT_SEARCH_PIXELS)

        columns = []
        for x in range(search_left, search_right + 1, SCAN_STEP_X):
            run_start = -1
            run_end = -1

            for y in range(turn_y, y0 + h):
                if is_white_pixel(img.get_pixel(x, y)):
                    if run_start < 0:
                        run_start = y
                    run_end = y
                elif run_start >= 0:
                    break

            if run_start >= 0:
                run_height = run_end - run_start + 1
                if run_height >= RIGHT_ANGLE_MIN_VERTICAL_HEIGHT:
                    columns.append((x, run_start, run_end, run_height))

        if columns:
            x_sum = sum(c[0] for c in columns)
            bottom_y = max(c[2] for c in columns)
            height_sum = sum(c[3] for c in columns)
            line_x = x_sum // len(columns)
            candidates.append((height_sum, direction, line_x, bottom_y))

    if not candidates:
        return (
            False, 0, 0, (FRAME_WIDTH // 2, turn_y),
            best_run, False)

    candidates.sort(reverse=True)
    _, direction, line_x, bottom_y = candidates[0]
    target_x = line_x + (direction * 90)
    turn_angle = -direction * 90
    turn_ready = (bottom_y >= RIGHT_ANGLE_TURN_TRIGGER_Y)

    if target_x < 0:
        target_x = 0
    elif target_x >= FRAME_WIDTH:
        target_x = FRAME_WIDTH - 1

    return (
        True,
        direction * RIGHT_ANGLE_TURN_ERROR_PIXELS,
        turn_angle,
        (target_x, turn_y),
        best_run,
        turn_ready)


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
right_angle_turn_cooldown = 0
right_angle_confirm_count = 0
right_angle_repeat_count = 0
right_angle_repeat_angle = 0

while True:
    clock.tick()
    frame_count += 1
    if right_angle_turn_cooldown > 0:
        right_angle_turn_cooldown -= 1
    if right_angle_repeat_count > 0:
        right_angle_repeat_count -= 1
        tx_message = "LTURN %d" % right_angle_repeat_angle
        print_message = "%s M=L P=0" % tx_message
        uart.write(tx_message + "\n")
        if (frame_count % PRINT_EVERY) == 0:
            print("TX:", print_message, "FPS:", clock.fps())
        continue

    img = sensor.snapshot()

    img.binary([BLACK_THRESHOLD])
    if NOISE_ERODE > 0:
        img.erode(NOISE_ERODE)

    x0, y0, w, h = ARC_ROI

    fast_l_valid, fast_l_err, fast_l_angle, fast_l_target, fast_l_run, \
        fast_l_ready = fast_right_angle_turn(img, x0, y0, w, h)
    if fast_l_valid:
        right_angle_confirm_count += 1
        if (fast_l_ready or right_angle_confirm_count >=
                RIGHT_ANGLE_CONFIRM_FRAMES) and right_angle_turn_cooldown == 0:
            right_angle_repeat_angle = fast_l_angle
            right_angle_repeat_count = RIGHT_ANGLE_TURN_REPEAT_FRAMES
            right_angle_turn_cooldown = RIGHT_ANGLE_TURN_COOLDOWN_FRAMES
            right_angle_confirm_count = 0
            tx_message = "LTURN %d" % right_angle_repeat_angle
        else:
            tx_message = "LINE 1 %d" % fast_l_err

        print_message = "%s M=L P=FAST" % tx_message
        uart.write(tx_message + "\n")
        left_x, right_x, line_y = fast_l_run
        img.draw_line(left_x, line_y, right_x, line_y,
            color=(0, 255, 0), thickness=3)
        img.draw_cross(fast_l_target[0], fast_l_target[1],
            color=(255, 0, 0), size=8)
        if (frame_count % PRINT_EVERY) == 0:
            print("TX:", print_message, "FPS:", clock.fps())
        continue

    columns = []

    for x in range(x0, x0 + w, SCAN_STEP_X):
        candidates = find_column_candidates(img, x, y0, h)
        if candidates:
            columns.append((x, candidates))

    arc_points = find_smooth_path(columns)

    rows = []
    for y in range(y0, y0 + h, SCAN_STEP_Y):
        candidates = find_row_candidates(img, y, x0, w)
        if candidates:
            rows.append((y, candidates))

    steep_points = find_vertical_path(rows)

    right_angle_valid, right_angle_err, right_angle_angle, right_angle_target, \
        right_angle_run, right_angle_ready = right_angle_turn(
            img, steep_points, x0, y0, w, h)

    if is_cross_like(img, steep_points, x0, y0, w, h):
        points = steep_points
        mode = "X"
    elif right_angle_valid:
        points = steep_points
        mode = "L"
    elif is_steep_like(steep_points):
        points = steep_points
        mode = "S"
    elif is_arc_like(arc_points):
        points = arc_points
        mode = "A"
    else:
        points = arc_points if len(arc_points) >= len(steep_points) else steep_points
        mode = "N"

    if mode != "N":
        if mode == "L":
            right_angle_confirm_count += 1
        else:
            right_angle_confirm_count = 0

        if mode == "A":
            target = arc_control_point(points)
        elif mode == "L":
            target = right_angle_target
        else:
            target = control_point(points)
        if mode == "X":
            err = 0
            target = (FRAME_WIDTH // 2, target[1])
        elif mode == "L":
            err = right_angle_err
        else:
            err = target[0] - (FRAME_WIDTH // 2)

        if right_angle_repeat_count > 0:
            tx_message = "LTURN %d" % right_angle_repeat_angle
        elif (mode == "L" and
                (right_angle_ready or
                    right_angle_confirm_count >= RIGHT_ANGLE_CONFIRM_FRAMES) and
                right_angle_turn_cooldown == 0):
            right_angle_repeat_angle = right_angle_angle
            right_angle_repeat_count = RIGHT_ANGLE_TURN_REPEAT_FRAMES
            tx_message = "LTURN %d" % right_angle_repeat_angle
            right_angle_turn_cooldown = RIGHT_ANGLE_TURN_COOLDOWN_FRAMES
            right_angle_confirm_count = 0
        else:
            tx_message = "LINE 1 %d" % err

        print_message = "%s M=%s P=%d" % (tx_message, mode, len(points))
        uart.write(tx_message + "\n")

        for i in range(0, len(points) - DRAW_STEP, DRAW_STEP):
            img.draw_line(points[i][0], points[i][1],
                points[i + DRAW_STEP][0], points[i + DRAW_STEP][1],
                color=(255, 0, 0), thickness=3)

        img.draw_cross(target[0], target[1], color=(255, 0, 0), size=8)
        if mode == "L":
            left_x, right_x, line_y = right_angle_run
            img.draw_line(left_x, line_y, right_x, line_y,
                color=(0, 255, 0), thickness=3)
    else:
        right_angle_confirm_count = 0
        if right_angle_repeat_count > 0:
            tx_message = "LTURN %d" % right_angle_repeat_angle
        else:
            tx_message = "LINE 0 0"
        print_message = "%s M=%s PA=%d PS=%d" % (
            tx_message, mode, len(arc_points), len(steep_points))
        uart.write(tx_message + "\n")

    if (frame_count % PRINT_EVERY) == 0:
        print("TX:", print_message, "FPS:", clock.fps())
