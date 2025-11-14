import cv2 as cv
from cv2 import aruco
import numpy as np

# Pre-load the dictionary
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
ARUCO_PARAMS = aruco.DetectorParameters()
DETECTOR = aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

def angleDetector(frame: np.ndarray):
    # Convert to grayscale
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) # Convert frame to grayscale

    # Decect marker
    corners, ids, rejected = DETECTOR.detectMarkers(gray_frame)

    if ids is None or len(corners) == 0:
        return 'S'
    
    c = corners[0].reshape((4,2))
    center_x = c[:, 0].mean()

    # Get frame center
    width = frame.shape[1]
    image_center_x = width / 2
    fov = 50.5 # fied of view in deg

    # Calculate the horixontal offset ratio
    ratio = (image_center_x - center_x) / image_center_x
    angle = round(ratio * (fov / 2), 2)

    if angle < 0:
        angle = 'r' + str(abs(angle))
    elif angle > 0:
        angle = 'l' + str(abs(angle))

    return angle
