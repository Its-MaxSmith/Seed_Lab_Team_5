# RGcolorDetector
#
# Max Smith & Garret Reader
# 11/5/2025
# 
# Description

# Imports for OpenCV
import cv2 as cv
from cv2 import aruco
import numpy as np

# Color constants
# Red color range 
RED_LOWER = np.array([150, 122, 170])
RED_UPPER = np.array([180, 230, 255])

# Green color range
GREEN_LOWER = np.array([55, 110, 110])
GREEN_UPPER = np.array([100, 215, 150])

# Pixel count needed
PIXEL_THRESHOLD = 50

# Aruco Library
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
ARUCO_PARAMS = aruco.DetectorParameters()
DETECTOR = aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

def RGcolorDetector(frame: np.ndarray, side_width=200):

    # Gray scale frame
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Detect marker
    corners, ids, rejected = DETECTOR.detectMarkers(gray_frame)

    if ids is None or len(corners) == 0:
        return 'N', frame

    corner = corners[0][0]
    
    # Marker bounding box
    x_min, y_min = np.min(corner, axis=0).astype(int)
    x_max, y_max = np.max(corner, axis=0).astype(int)

    # Bounds in frame
    x_min = max(0, x_min)
    y_min = max(0, y_min)
    x_max = min(frame.shape[1], x_max)
    y_max = min(frame.shape[0], y_max)

    # Convert whole frame to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # --- Left ROI for green detection ---
    left_x_min = max(0, x_min - side_width)
    left_x_max = x_min
    left_roi = hsv[y_min:y_max, left_x_min:left_x_max]
    mask_green = cv.inRange(left_roi, GREEN_LOWER, GREEN_UPPER)
    green_pixels = cv.countNonZero(mask_green)

    # --- Right ROI for red detection ---
    right_x_min = x_max
    right_x_max = min(frame.shape[1], x_max + side_width)
    right_roi = hsv[y_min:y_max, right_x_min:right_x_max]
    mask_red = cv.inRange(right_roi, RED_LOWER, RED_UPPER)
    red_pixels = cv.countNonZero(mask_red)
    print(str(red_pixels))
    print(str(green_pixels))

    # Determine color based on pixel counts
    if green_pixels > PIXEL_THRESHOLD:
        color = 'L'  # Green on left
    elif red_pixels > PIXEL_THRESHOLD:
        color = 'R'  # Red on right
    else:
        color = 'N'  # None detected

    return color

