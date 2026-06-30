import sensor
import time
from pyb import UART

BLACK_THRESHOLD = (0, 25, -20, 20, -20, 20)
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
ARC_ROI = (40, 35, 250, 165)
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
MAX_POINT_DY = 16
MAX_POINT_DX = 18
MAX_GAP_X = 15
MAX_GAP_Y = 15
NOISE_ERODE = 1
DRAW_STEP = 2
PRINT_EVERY = 3
UART_BAUDRATE = 115200


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


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

uart = UART(3, UART_BAUDRATE, timeout_char=1000)
clock = time.clock()
frame_count = 0

while True:
    clock.tick()
    frame_count += 1
    img = sensor.snapshot()

    img.binary([BLACK_THRESHOLD])
    if NOISE_ERODE > 0:
        img.erode(NOISE_ERODE)

    x0, y0, w, h = ARC_ROI
    columns = []

    for x in range(x0, x0 + w, SCAN_STEP_X):
        candidates = find_column_candidates(img, x, y0, h)
        if candidates:
            columns.append((x, candidates))

    arc_points = find_smooth_path(columns)
    steep_points = []

    if is_arc_like(arc_points):
        points = arc_points
        mode = "A"
    else:
        rows = []
        for y in range(y0, y0 + h, SCAN_STEP_Y):
            candidates = find_row_candidates(img, y, x0, w)
            if candidates:
                rows.append((y, candidates))

        steep_points = find_vertical_path(rows)
        if is_steep_like(steep_points):
            points = steep_points
            mode = "S"
        else:
            points = arc_points if len(arc_points) >= len(steep_points) else steep_points
            mode = "N"

    if mode != "N":
        target = control_point(points)
        err = target[0] - (FRAME_WIDTH // 2)
        tx_message = "LINE 1 %d" % err
        print_message = "%s M=%s P=%d" % (tx_message, mode, len(points))
        uart.write(tx_message + "\n")

        for i in range(0, len(points) - DRAW_STEP, DRAW_STEP):
            img.draw_line(points[i][0], points[i][1],
                points[i + DRAW_STEP][0], points[i + DRAW_STEP][1],
                color=(255, 0, 0), thickness=3)

        img.draw_cross(target[0], target[1], color=(255, 0, 0), size=8)
    else:
        tx_message = "LINE 0 0"
        print_message = "%s M=%s PA=%d PS=%d" % (
            tx_message, mode, len(arc_points), len(steep_points))
        uart.write(tx_message + "\n")

    if (frame_count % PRINT_EVERY) == 0:
        print("TX:", print_message, "FPS:", clock.fps())
