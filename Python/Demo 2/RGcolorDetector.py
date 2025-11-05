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

# Aruco Library
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

def RGcolorDetector(frame: np.ndarray):

    # Variable to be returned
    color = 'No marker found'

    # Red color range - upper and lower due to 0deg and 180deg
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # Green color range
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([90, 255, 255])

    # Gray scale frame
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Decect marker
    corners, ids, rejected = aruco.detectMarkers(gray_frame, aruco_dict)
    aruco.drawDetectedMarkers(frame, corners, borderColor = 1)

    if not ids is None:

        # Convert to Hue Saturation Value
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Red masks
        mask_red1 = cv.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv.bitwise_or(mask_red1, mask_red2)

        # Green mask
        mask_green = cv.inRange(hsv, lower_green, upper_green)

        red_pixels = cv.countNonZero(mask_red)
        green_pixels = cv.countNonZero(mask_green)

        # Check how many pixels are seen - change pixel count as needed
        # Test if both are seen what is higher
        if red_pixels > 1000 and green_pixels > 1000:
            
            if red_pixels > green_pixels:
                print("More red than green")
                color = 'red'
            else:
                print("More green than red")
                color = 'green'

        # Test if red is seen
        elif red_pixels > 1000:
            print("Red detected")
            color = 'red'

        # Test if green is seen
        elif green_pixels > 1000:
            print("Green detected")
            color = 'green'

        else:
            print("No red or green detected")
            color = 'none found'

        return(color)
    
    return('No maker found')



