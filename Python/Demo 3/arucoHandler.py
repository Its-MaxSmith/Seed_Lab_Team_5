import cv2 as cv
from cv2 import aruco
import numpy as np

# === CONSTANTS ===
MARKER_SIZE_IN = 2.0            # Marker width in inches
FOCAL_LENGTH_PIXELS = 630
FOV_DEG = 50.5                  # Horizontal FOV of camera
PIXEL_THRESHOLD = 50            # For color detection
SIDE_WIDTH = 200                # Side ROI width for color

# Color ranges (HSV)
RED_LOWER = np.array([150, 122, 170])
RED_UPPER = np.array([180, 230, 255])

GREEN_LOWER = np.array([70, 70, 30])
GREEN_UPPER = np.array([100, 255, 255])

# Aruco Setup
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
ARUCO_PARAMS = aruco.DetectorParameters()
DETECTOR = aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)


def arucoHandler(frame: np.ndarray):
    """
    Returns:
        distance_str:  'f#'  (1–∞)
        angle_str:     'l#' or 'r#' (+ steering angle)
        color:         'L', 'R', or 'N'
    """

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    corners, ids, _ = DETECTOR.detectMarkers(gray)

    if ids is None or len(corners) == 0:
        return "f", "S", "N"

    # ----------------------------------------
    # 1) FIND THE CLOSEST MARKER
    # ----------------------------------------
    closest_dist = None
    closest_corner = None

    for corner in corners:
        pts = corner[0]
        diffs = np.diff(np.vstack([pts, pts[0]]), axis=0)
        side_lengths = np.linalg.norm(diffs, axis=1)
        P = np.mean(side_lengths)
        dist = (MARKER_SIZE_IN * FOCAL_LENGTH_PIXELS) / P

        if closest_dist is None or dist < closest_dist:
            closest_dist = dist
            closest_corner = pts

    if closest_dist > 50:
        return "f", "S", "N"
        
    # Final rounded distance
    D = round(closest_dist)
    distance_str = "f" + str(D)

    # ----------------------------------------
    # 2) ANGLE DETECTION (using closest marker)
    # ----------------------------------------
    c = closest_corner
    center_x = c[:, 0].mean()
    image_center_x = frame.shape[1] / 2

    ratio = (image_center_x - center_x) / image_center_x
    angle_val = round(ratio * (FOV_DEG / 2))

    if angle_val < 0:
        angle_str = "r" + str(abs(angle_val))
    elif angle_val > 0:
        angle_str = "l" + str(abs(angle_val))
    else:
        angle_str = "S"

    # ----------------------------------------
    # 3) COLOR DETECTION (based on closest marker)
    # ----------------------------------------
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    x_min, y_min = np.min(c, axis=0).astype(int)
    x_max, y_max = np.max(c, axis=0).astype(int)

    x_min = max(0, x_min)
    y_min = max(0, y_min)
    x_max = min(frame.shape[1], x_max)
    y_max = min(frame.shape[0], y_max)

    # Left ROI → green
    left_x_min = max(0, x_min - SIDE_WIDTH)
    left_x_max = x_min
    left_roi = hsv[y_min:y_max, left_x_min:left_x_max]
    green_pixels = cv.countNonZero(cv.inRange(left_roi, GREEN_LOWER, GREEN_UPPER))

    # Right ROI → red
    right_x_min = x_max
    right_x_max = min(frame.shape[1], x_max + SIDE_WIDTH)
    right_roi = hsv[y_min:y_max, right_x_min:right_x_max]
    red_pixels = cv.countNonZero(cv.inRange(right_roi, RED_LOWER, RED_UPPER))

    if green_pixels > PIXEL_THRESHOLD and red_pixels > PIXEL_THRESHOLD:
        if green_pixels > red_pixels:
            color = 'L'
        else:
            color = 'R'
    elif green_pixels > PIXEL_THRESHOLD:
        color = "L"
    elif red_pixels > PIXEL_THRESHOLD:
        color = "R"
    else:
        color = "E"

    # ----------------------------------------
    # Return all 3 results
    # ----------------------------------------
    return distance_str, angle_str, color
