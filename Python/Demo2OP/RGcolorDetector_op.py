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
RED_LOWER = np.array([130, 100, 100])
RED_UPPER = np.array([180, 255, 255])

# Green color range
GREEN_LOWER = np.array([55, 76, 16])
GREEN_UPPER = np.array([85, 175, 255])

# Pixel count needed
PIXEL_THRESHOLD = 200

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
    x_max, y_max = np.max(corner, axis = 0).astype(int)

    # Bounds in frame
    x_min = max(0, x_min)
    y_min = max(0, y_min)
    x_max = min(frame.shape[1], x_max)
    y_max = min(frame.shape[0], y_max)

    '''# Crop around marker
    roi = frame[y_min:y_max, x_min:x_max]'''

    # Convert to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    mask_red = cv.inRange(hsv, RED_LOWER, RED_UPPER)
    mask_green = cv.inRange(hsv, GREEN_LOWER, GREEN_UPPER)

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
