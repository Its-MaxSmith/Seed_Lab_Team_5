
# Demo 1 Main
#
# Max Smith & Garret Reader
# 10/13/25
#
# Description

# Imports for CV
import cv2 as cv
from cv2 import aruco
import numpy as np
import time

# Imports for function files
import LCD_print

# Main program
def main():

    # Load calibration data
    data = np.load('camera_calibration_data.npz')
    mtx = data['mtx']
    dist = data['dist']

    # Load a new image to undistort
    capture = cv.VideoCapture(0)

    # Get image dimensions
    h, w = capture.shape[:2]

    # Compute the new camera matrix and ROI
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    # Undistort the image
    dst = cv.undistort(capture, mtx, dist, None, newcameramtx)

    # Crop the image to the valid ROI
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]

    i = 0 # For print delay
    msg = 'No message' # For degree message

    while True:
        isTrue, frame = capture.read()

        #----------------
        # Degree Finding Function
        #----------------

        if (i >= 50):
            LCD_print.LCD_print(msg)
            i = 0

        i += 1

        if cv.waitKey(20) & 0xFF == ord('d'):
            # Clear pointers and windows
            capture.release()
            cv.destroyAllWindows()
            break

# End main

# Run main
if __name__ == "__main__":
    main() # Run main program