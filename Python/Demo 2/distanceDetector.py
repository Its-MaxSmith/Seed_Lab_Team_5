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
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

def distanceDetector(frame: np.ndarray):

    distance = 'No marker found'
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    corners, ids, _ = aruco.detectMarkers(gray_frame, aruco_dict)
    aruco.drawDetectedMarkers(frame, corners, borderColor=1)

    if ids is not None:
        corner = corners[0][0]

        # Compute average side length (in pixels)
        sides = [
            np.linalg.norm(corner[0] - corner[1]),
            np.linalg.norm(corner[1] - corner[2]),
            np.linalg.norm(corner[2] - corner[3]),
            np.linalg.norm(corner[3] - corner[0])
        ]
        P = np.mean(sides)
        print(P)
        # Estimate distance using pinhole model
        D = (MARKER_SIZE_IN * FOCAL_LENGTH_PIXELS) / P

        distance = f'Estimated Distance: {D:.2f} inches'

    return distance
