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
# Red color range - upper and lower due to 0deg and 180deg
RED_LOWER1 = np.array([0, 100, 100])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([160, 100, 100])
RED_UPPER2 = np.array([180, 255, 255])

# Green color range
GREEN_LOWER = np.array([40, 50, 50])
GREEN_UPPER = np.array([90, 255, 255])

# Pixel count needed
PIXEL_THRESHOLD = 1000

# Aruco Library
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
ARUCO_PARAMS = aruco.DetectorParameters()
DETECTOR = aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

def RGcolorDetector(frame: np.ndarray):

    # Gray scale frame
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Decect marker
    corners, ids, rejected = DETECTOR.detectMarkers(gray_frame)

    if ids is None or len(corners) == 0:
        return 'N'
    
    corner = corners[0][0]

    # Bounding box
    x_min, y_min = np.min(corner, axis = 0).astype(int)
    x_max, y_maz = np.max(corner, axis = 0).astype(int)

    # Bounds in frame
    x_min = max(0, x_min)
    y_min = max(0, y_min)
    x_max = min(frame.shape[1], x_max)
    y_max = min(frame.shape[0], y_max)

    # Crop around marker
    roi = frame[y_min:y_max, x_min:x_max]

    # Convert to HSV
    hsv = cv.cvtColor(roi, cv.COLOR_BGR2HSV)

    # Combine red ranges
    mask_red = cv.bitwise_or(
        cv.inRange(hsv, RED_LOWER1, RED_UPPER1), 
        cv.inrange(hsv, RED_LOWER2, RED_UPPER2)
    )
    mask_green = cv.inrange(hsv, GREEN_LOWER, GREEN_UPPER)

    red_pixels = cv.countNonZero(mask_red)
    green_pixels = cv.countNonZero(mask_green)

    # Determine color based on pixel counts
    if red_pixels > PIXEL_THRESHOLD and green_pixels > PIXEL_THRESHOLD:
        color = 'R' if red_pixels > green_pixels else 'L'
    elif red_pixels > PIXEL_THRESHOLD:
        color = 'R'
    elif green_pixels > PIXEL_THRESHOLD:
        color = 'L'
    else:
        color = 'N'

    return color