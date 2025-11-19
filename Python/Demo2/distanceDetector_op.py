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
FOCAL_LENGTH_PIXELS = 630  

# Aruco dict
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
ARUCO_PARAMS = aruco.DetectorParameters()
DETECTOR = aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

def distanceDetector(frame: np.ndarray):

    # Convert to grayscale
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Detect marker
    corners, ids, _ = DETECTOR.detectMarkers(gray_frame)

    if ids is None or len(corners) == 0:
        return "f"
    
    corner = corners[0][0]

    best_D = None

    for corner in corners:
        pts = corner[0]
        diffs = np.diff(np.vstack([pts, pts[0]]), axis=0)
        side_lengths = np.linalg.norm(diffs, axis=1)
        P = np.mean(side_lengths)
        dist = (MARKER_SIZE_IN * FOCAL_LENGTH_PIXELS) / P

        if best_D is None or dist < best_D:
            best_D = dist


    D = round(best_D)
    
    if D < 12:
        D = 1

    return 'f' + str(D)
