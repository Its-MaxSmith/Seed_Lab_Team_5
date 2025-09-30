# Python function that looks for an Aruco marker using images in grayscale 
# taken using an atached camera

#returns location of marker in one of four quadrants of the frame and the corners of the marker
# Imports for CV
import cv2 as cv
from cv2 import aruco
import numpy as np


def arucoDetector(frame: np.ndarray):
    # Add the aruco dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) # Convert frame to grayscale

    # Decect marker
    corners, ids, rejected = aruco.detectMarkers(gray_frame, aruco_dict)
    aruco.drawDetectedMarkers(frame, corners, borderColor = 1)

    #Establish frame quadrants
    height, width, _ = frame.shape
    
    # Coordinates of the center
    center_x = width // 2
    center_y = height // 2

    # Draw vertical line
    cv.line(frame, (center_x, 0), (center_x, height), color=(0, 0, 255), thickness=2)

    # Draw horizontal line
    cv.line(frame, (0, center_y), (width, center_y), color=(0, 0, 255), thickness=2)

    # Determine location of marker
    location = 0
    if not ids is None:
        ids = ids.flatten()
            # Find center of marker
        all_corners = np.array([c.reshape(4,2) for c in corners])
        centers = all_corners.mean(axis=1)
        for (outline, id, center) in zip(corners, ids, centers):
            if center[0] < width/2 and center[1] >= height/2:
                location = 1
            elif center[0] >= width/2 and center[1] >= height/2:
                location = 2
            elif center[0] < width/2 and center[1] < height/2:
                location = 3
            elif center[0] >= width/2 and center[1] < height/2:
                location = 4
    return location, corners
