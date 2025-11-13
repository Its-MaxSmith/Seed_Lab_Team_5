# distanceDetector.py
#
# Max Smith & Garret Reader
# 11/7/2025
# 
# Description

import cv2 as cv
from cv2 import aruco
import numpy as np

# Constants
MARKER_SIZE_IN = 2.0  # inches
FOCAL_LENGTH_PIXELS = 2000  

# Aruco dict
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
ARUCO_PARAMS = aruco.DetectorParameter()
DETECTOR = aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

def distanceDetector(frame: np.ndarray):

    # Convert to grayscale
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Detect marker
    corners, ids, _ = DETECTOR.detectMarkers(gray_frame)

    if ids is None or len(corners) == 0:
        return "No marker found"
    
    corner = corners[0][0]

    # Vectorize
    diffs = np.diff(np.vstack([corner, corner[0]]), axis = 0)
    side_lengths = np.linalg.norm(diffs, axis = 1)
    P = np.mean(side_lengths)

    # Find distance
    D = (MARKER_SIZE_IN * FOCAL_LENGTH_PIXELS) / P

    return D
