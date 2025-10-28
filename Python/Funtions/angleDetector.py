import cv2 as cv
from cv2 import aruco
import numpy as np

def angleDetector(frame: np.ndarray):
    # Add the aruco dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) # Convert frame to grayscale

    # Decect marker
    corners, ids, rejected = aruco.detectMarkers(gray_frame, aruco_dict)
    aruco.drawDetectedMarkers(frame, corners, borderColor = 1)

    angle = None
    if not ids is None:
        ids = ids.flatten()
        # Find center of marker
        all_corners = np.array([c.reshape(4,2) for c in corners])
        centers = all_corners.mean(axis=1)

        height, width, _ = frame.shape
    
        fov = 61.0  # Field of view in degrees
        center_x = width // 2
        distance = center_x - centers[0][0]
        ratio = center_x / distance
        angle = ratio * fov / 2
        
    return angle