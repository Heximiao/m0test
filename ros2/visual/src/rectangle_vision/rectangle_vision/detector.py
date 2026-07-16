from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class RectangleDetection:
    corners: np.ndarray
    center: tuple[int, int]
    score: float


def order_corners(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float32).reshape(4, 2)
    ordered = np.empty((4, 2), dtype=np.float32)
    coordinate_sum = points.sum(axis=1)
    coordinate_difference = np.diff(points, axis=1).reshape(-1)
    ordered[0] = points[np.argmin(coordinate_sum)]
    ordered[2] = points[np.argmax(coordinate_sum)]
    ordered[1] = points[np.argmin(coordinate_difference)]
    ordered[3] = points[np.argmax(coordinate_difference)]
    return ordered


def diagonal_intersection(corners: np.ndarray) -> tuple[int, int]:
    top_left, top_right, bottom_right, bottom_left = corners.astype(np.float64)
    first_direction = bottom_right - top_left
    second_direction = bottom_left - top_right
    matrix = np.column_stack((first_direction, -second_direction))
    determinant = np.linalg.det(matrix)
    if abs(determinant) < 1e-6:
        center = corners.mean(axis=0)
    else:
        first_scale = np.linalg.solve(matrix, top_right - top_left)[0]
        center = top_left + first_scale * first_direction
    return tuple(np.rint(center).astype(int))


def _quad_from_contour(contour: np.ndarray, epsilon_ratio: float) -> np.ndarray | None:
    perimeter = cv2.arcLength(contour, True)
    approximation = cv2.approxPolyDP(contour, epsilon_ratio * perimeter, True)
    if len(approximation) != 4 or not cv2.isContourConvex(approximation):
        return None
    return order_corners(approximation.reshape(4, 2))


def _corner_alignment(outer: np.ndarray, inner: np.ndarray) -> float:
    outer_lengths = np.linalg.norm(np.roll(outer, -1, axis=0) - outer, axis=1)
    inner_lengths = np.linalg.norm(np.roll(inner, -1, axis=0) - inner, axis=1)
    if np.any(outer_lengths < 1.0) or np.any(inner_lengths < 1.0):
        return 0.0
    ratios = inner_lengths / outer_lengths
    return float(np.clip(1.0 - np.std(ratios), 0.0, 1.0))


def _ring_pair_is_plausible(outer: np.ndarray, inner: np.ndarray) -> bool:
    corner_gaps = np.linalg.norm(outer - inner, axis=1)
    typical_gap = float(np.median(corner_gaps))
    inner_diagonal = float(np.linalg.norm(inner[2] - inner[0]))
    if typical_gap < 3.0 or inner_diagonal < 20.0:
        return False
    if typical_gap > inner_diagonal * 0.16:
        return False
    if float(np.max(corner_gaps)) > typical_gap * 2.2:
        return False
    return True


def _line_intersection(
    first_start: np.ndarray,
    first_end: np.ndarray,
    second_start: np.ndarray,
    second_end: np.ndarray,
) -> np.ndarray | None:
    first_direction = first_end - first_start
    second_direction = second_end - second_start
    matrix = np.column_stack((first_direction, -second_direction))
    if abs(np.linalg.det(matrix)) < 1e-6:
        return None
    scale = np.linalg.solve(matrix, second_start - first_start)[0]
    return first_start + scale * first_direction


def _centerline_from_inner_quad(
    inner_quad: np.ndarray, black_mask: np.ndarray
) -> np.ndarray | None:
    center = inner_quad.mean(axis=0)
    height, width = black_mask.shape
    maximum_scan = max(8, int(min(height, width) * 0.16))
    shifted_edges = []
    measured_thicknesses = []

    for edge_index in range(4):
        start = inner_quad[edge_index]
        end = inner_quad[(edge_index + 1) % 4]
        direction = end - start
        length = np.linalg.norm(direction)
        if length < 8.0:
            return None
        outward = np.array([direction[1], -direction[0]], dtype=np.float32) / length
        midpoint = (start + end) * 0.5
        if np.dot(outward, midpoint - center) < 0.0:
            outward = -outward

        runs = []
        for fraction in np.linspace(0.18, 0.82, 9):
            edge_point = start + fraction * direction
            run = 0
            started = False
            gap = 0
            for offset in range(1, maximum_scan + 1):
                sample = np.rint(edge_point + outward * offset).astype(int)
                if not (0 <= sample[0] < width and 0 <= sample[1] < height):
                    break
                is_black = black_mask[sample[1], sample[0]] != 0
                if is_black:
                    started = True
                    run = offset
                    gap = 0
                elif started:
                    gap += 1
                    if gap >= 3:
                        break
            if run >= 3:
                runs.append(run)

        thickness = float(np.median(runs)) if runs else 0.0
        measured_thicknesses.append(thickness)
        shifted_edges.append((start, end, outward))

    valid_thicknesses = [value for value in measured_thicknesses if value >= 3.0]
    if len(valid_thicknesses) < 2:
        return None
    typical_thickness = float(np.median(valid_thicknesses))

    center_edges = []
    for (start, end, outward), thickness in zip(shifted_edges, measured_thicknesses):
        if thickness < 3.0 or thickness > typical_thickness * 2.2:
            thickness = typical_thickness
        offset = outward * thickness * 0.5
        center_edges.append((start + offset, end + offset))

    corners = []
    for corner_index in range(4):
        previous_edge = center_edges[(corner_index - 1) % 4]
        current_edge = center_edges[corner_index]
        intersection = _line_intersection(
            previous_edge[0], previous_edge[1], current_edge[0], current_edge[1]
        )
        if intersection is None:
            return None
        corners.append(intersection)
    return order_corners(np.asarray(corners, dtype=np.float32))


def _detect_from_inner_region(
    blurred: np.ndarray,
    black_mask: np.ndarray,
    minimum_area_ratio: float,
    maximum_area_ratio: float,
) -> RectangleDetection | None:
    white_mask = cv2.bitwise_not(black_mask)
    contours, _ = cv2.findContours(white_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    image_area = blurred.shape[0] * blurred.shape[1]
    best_detection = None

    for contour in contours:
        area = abs(cv2.contourArea(contour))
        area_ratio = area / image_area
        if not minimum_area_ratio <= area_ratio <= min(maximum_area_ratio, 0.82):
            continue

        inner_quad = None
        perimeter = cv2.arcLength(contour, True)
        for epsilon_ratio in (0.02, 0.03, 0.04, 0.055, 0.07, 0.09):
            approximation = cv2.approxPolyDP(contour, epsilon_ratio * perimeter, True)
            if len(approximation) == 4 and cv2.isContourConvex(approximation):
                inner_quad = order_corners(approximation.reshape(4, 2))
                break
        if inner_quad is None:
            continue

        centerline = _centerline_from_inner_quad(inner_quad, black_mask)
        if centerline is None:
            continue

        polygon_mask = np.zeros_like(blurred)
        cv2.fillConvexPoly(polygon_mask, np.rint(inner_quad).astype(np.int32), 255)
        inside_brightness = cv2.mean(blurred, mask=polygon_mask)[0]
        if inside_brightness < 105.0:
            continue

        score = area_ratio * (inside_brightness / 255.0)
        if best_detection is None or score > best_detection.score:
            best_detection = RectangleDetection(
                corners=centerline,
                center=diagonal_intersection(centerline),
                score=score,
            )
    return best_detection


def detect_black_rectangle(
    frame: np.ndarray,
    minimum_area_ratio: float = 0.08,
    maximum_area_ratio: float = 0.95,
    approximation_epsilon: float = 0.025,
) -> RectangleDetection | None:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, black_mask = cv2.threshold(
        blurred, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU
    )
    contours, hierarchy = cv2.findContours(
        black_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )
    if hierarchy is None:
        return None

    image_area = frame.shape[0] * frame.shape[1]
    best_detection = None
    hierarchy = hierarchy[0]

    for outer_index, outer_contour in enumerate(contours):
        outer_area = abs(cv2.contourArea(outer_contour))
        area_ratio = outer_area / image_area
        if not minimum_area_ratio <= area_ratio <= maximum_area_ratio:
            continue

        outer_quad = _quad_from_contour(outer_contour, approximation_epsilon)
        if outer_quad is None:
            continue

        child_index = hierarchy[outer_index][2]
        while child_index != -1:
            inner_contour = contours[child_index]
            inner_area = abs(cv2.contourArea(inner_contour))
            inner_quad = _quad_from_contour(inner_contour, approximation_epsilon)
            hole_ratio = inner_area / outer_area if outer_area else 0.0

            if (
                inner_quad is not None
                and 0.25 <= hole_ratio <= 0.90
                and _ring_pair_is_plausible(outer_quad, inner_quad)
            ):
                centerline = (outer_quad + inner_quad) * 0.5
                alignment = _corner_alignment(outer_quad, inner_quad)
                score = area_ratio * hole_ratio * alignment
                if best_detection is None or score > best_detection.score:
                    best_detection = RectangleDetection(
                        corners=centerline,
                        center=diagonal_intersection(centerline),
                        score=score,
                    )
            child_index = hierarchy[child_index][0]

    fallback = _detect_from_inner_region(
        blurred,
        black_mask,
        minimum_area_ratio,
        maximum_area_ratio,
    )
    if fallback is not None and (
        best_detection is None or fallback.score > best_detection.score
    ):
        return fallback
    return best_detection


class RectangleTracker:
    def __init__(self, maximum_missed_frames: int = 10, hold_frames: int = 3):
        self.maximum_missed_frames = maximum_missed_frames
        self.hold_frames = hold_frames
        self.previous_gray = None
        self.previous_corners = None
        self.velocity = np.zeros((4, 2), dtype=np.float32)
        self.missed_frames = 0

    def _appearance_plausible(
        self, frame: np.ndarray, corners: np.ndarray
    ) -> bool:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        threshold, _ = cv2.threshold(
            blurred, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU
        )
        center = corners.mean(axis=0)
        height, width = gray.shape
        border_values = []
        interior_values = []

        for edge_index in range(4):
            start = corners[edge_index]
            end = corners[(edge_index + 1) % 4]
            for fraction in np.linspace(0.12, 0.88, 13):
                border_point = start + fraction * (end - start)
                interior_point = border_point * 0.76 + center * 0.24
                for point, values in (
                    (border_point, border_values),
                    (interior_point, interior_values),
                ):
                    sample = np.rint(point).astype(int)
                    if 0 <= sample[0] < width and 0 <= sample[1] < height:
                        values.append(int(blurred[sample[1], sample[0]]))

        if len(border_values) < 30 or len(interior_values) < 30:
            return False
        black_border_fraction = np.mean(np.asarray(border_values) < threshold)
        bright_interior_fraction = np.mean(
            np.asarray(interior_values) > max(95.0, threshold)
        )
        return bool(
            black_border_fraction >= 0.43
            and bright_interior_fraction >= 0.48
        )

    def _valid_corners(self, corners: np.ndarray, frame_shape) -> bool:
        height, width = frame_shape[:2]
        integer_corners = np.rint(corners).astype(np.int32)
        area = abs(cv2.contourArea(integer_corners))
        if area < height * width * 0.025:
            return False
        if area > height * width * 0.92:
            return False
        if not cv2.isContourConvex(integer_corners):
            return False
        margin = max(height, width) * 0.06
        return bool(
            np.all(corners[:, 0] >= -margin)
            and np.all(corners[:, 0] <= width + margin)
            and np.all(corners[:, 1] >= -margin)
            and np.all(corners[:, 1] <= height + margin)
        )

    def _track(self, gray: np.ndarray) -> np.ndarray | None:
        if self.previous_gray is None or self.previous_corners is None:
            return None

        feature_mask = np.zeros_like(self.previous_gray)
        cv2.fillConvexPoly(
            feature_mask, np.rint(self.previous_corners).astype(np.int32), 255
        )
        features = cv2.goodFeaturesToTrack(
            self.previous_gray,
            maxCorners=120,
            qualityLevel=0.008,
            minDistance=5,
            mask=feature_mask,
            blockSize=5,
        )
        corner_features = self.previous_corners.reshape(-1, 1, 2).astype(np.float32)
        if features is None:
            features = corner_features
        else:
            features = np.concatenate((features, corner_features), axis=0)

        tracked, status, errors = cv2.calcOpticalFlowPyrLK(
            self.previous_gray,
            gray,
            features,
            None,
            winSize=(31, 31),
            maxLevel=4,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        if tracked is None or status is None:
            return None
        status = status.reshape(-1).astype(bool)
        if errors is not None:
            status &= errors.reshape(-1) < 45.0
        previous_points = features.reshape(-1, 2)[status]
        current_points = tracked.reshape(-1, 2)[status]
        if len(current_points) < 4:
            return None

        homography, inliers = cv2.findHomography(
            previous_points, current_points, cv2.RANSAC, 4.0
        )
        minimum_inliers = max(6, int(len(current_points) * 0.45))
        if (
            homography is None
            or inliers is None
            or int(inliers.sum()) < minimum_inliers
        ):
            return None
        tracked_corners = cv2.perspectiveTransform(
            self.previous_corners.reshape(1, 4, 2), homography
        )[0]
        if not self._valid_corners(tracked_corners, gray.shape):
            return None
        previous_area = abs(
            cv2.contourArea(np.rint(self.previous_corners).astype(np.int32))
        )
        tracked_area = abs(
            cv2.contourArea(np.rint(tracked_corners).astype(np.int32))
        )
        area_ratio = tracked_area / previous_area if previous_area else 1.0
        if not 0.58 <= area_ratio <= 1.72:
            return None

        previous_diagonal = np.linalg.norm(
            self.previous_corners[2] - self.previous_corners[0]
        )
        displacements = np.linalg.norm(
            tracked_corners - self.previous_corners, axis=1
        )
        median_displacement = float(np.median(displacements))
        if float(np.max(displacements)) > max(
            previous_diagonal * 0.42, median_displacement * 3.0 + 18.0
        ):
            return None
        return order_corners(tracked_corners)

    def update(
        self,
        frame: np.ndarray,
        minimum_area_ratio: float = 0.08,
        maximum_area_ratio: float = 0.95,
        approximation_epsilon: float = 0.025,
        run_detection: bool = True,
    ) -> RectangleDetection | None:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        direct = None
        if run_detection or self.previous_corners is None:
            direct = detect_black_rectangle(
                frame,
                minimum_area_ratio=minimum_area_ratio,
                maximum_area_ratio=maximum_area_ratio,
                approximation_epsilon=approximation_epsilon,
            )

        if direct is not None and self.previous_corners is not None:
            predicted_corners = self.previous_corners + self.velocity
            predicted_center = predicted_corners.mean(axis=0)
            direct_center = direct.corners.mean(axis=0)
            diagonal = np.linalg.norm(
                self.previous_corners[2] - self.previous_corners[0]
            )
            expected_speed = np.linalg.norm(self.velocity.mean(axis=0))
            maximum_jump = max(35.0, diagonal * 0.10 + expected_speed * 1.8)
            previous_area = abs(
                cv2.contourArea(np.rint(self.previous_corners).astype(np.int32))
            )
            direct_area = abs(
                cv2.contourArea(np.rint(direct.corners).astype(np.int32))
            )
            area_ratio = direct_area / previous_area if previous_area else 1.0
            if (
                np.linalg.norm(direct_center - predicted_center) > maximum_jump
                or not 0.55 <= area_ratio <= 1.75
            ):
                direct = None

        if direct is not None:
            if not self._appearance_plausible(frame, direct.corners):
                direct = None

        if direct is not None:
            if self.previous_corners is not None:
                measured_velocity = direct.corners - self.previous_corners
                self.velocity = 0.65 * measured_velocity + 0.35 * self.velocity
            else:
                self.velocity.fill(0.0)
            self.previous_corners = direct.corners.copy()
            self.previous_gray = gray
            self.missed_frames = 0
            return direct

        tracked_corners = self._track(gray)
        if tracked_corners is not None and self._appearance_plausible(
            frame, tracked_corners
        ):
            self.velocity = 0.65 * (tracked_corners - self.previous_corners) + 0.35 * self.velocity
            self.previous_corners = tracked_corners
            self.previous_gray = gray
            self.missed_frames += 1
            return RectangleDetection(
                corners=tracked_corners,
                center=diagonal_intersection(tracked_corners),
                score=max(0.05, 0.5 - self.missed_frames * 0.04),
            )

        self.missed_frames += 1
        if self.previous_corners is not None and self.missed_frames <= self.hold_frames:
            predicted = self.previous_corners + self.velocity
            self.velocity *= 0.75
            if self._valid_corners(predicted, gray.shape) and self._appearance_plausible(
                frame, predicted
            ):
                self.previous_corners = predicted
                self.previous_gray = gray
                return RectangleDetection(
                    corners=predicted,
                    center=diagonal_intersection(predicted),
                    score=0.03,
                )

        self.previous_gray = gray
        if tracked_corners is not None or self.missed_frames > self.maximum_missed_frames:
            self.previous_corners = None
            self.velocity.fill(0.0)
        return None


def draw_detection(
    frame: np.ndarray,
    detection: RectangleDetection | None,
    line_thickness: int = 3,
    center_radius: int = 7,
) -> np.ndarray:
    annotated = frame.copy()
    if detection is None:
        cv2.putText(
            annotated,
            "rectangle not detected",
            (16, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        return annotated

    corners = np.rint(detection.corners).astype(np.int32)
    cv2.polylines(annotated, [corners], True, (0, 255, 0), line_thickness, cv2.LINE_AA)
    cv2.circle(annotated, detection.center, center_radius, (0, 0, 255), -1, cv2.LINE_AA)
    return annotated
