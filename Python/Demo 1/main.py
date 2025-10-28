# Demo 1 Main
#
# Max Smith & Garret Reader
# 10/13/25
#
# Description: This code reads the angle between the camera and the marker. This angle is then sent to
# the LCD through i2c.

# Imports for CV
import cv2 as cv
from cv2 import aruco
import numpy as np
import time

# Imports for function files
import LCD_print
import angleDetector

# Main program
def main():

    # Load calibration data
    data = np.load('camera_calibration_data.npz')
    mtx = data['mtx']
    dist = data['dist']

    # Start video capture
    capture = cv.VideoCapture(0)
    if not capture.isOpened():
        print("Could not open camera.")
        return

    i = 0  # For LCD print delay
    msg = 'No message'  # Placeholder for degree message

    print("Live camera feed started.")
    print("Press 'd' to exit.\n")

    while True:
        isTrue, frame = capture.read()
        cv.imshow("Live Feed", frame)
        if not isTrue:
            print("Failed to grab frame.")
            break

        # Get image dimensions
        h, w = frame.shape[:2]

        # Compute the new camera matrix and ROI
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

        # Undistort the frame
        undistorted = cv.undistort(frame, mtx, dist, None, newcameramtx)

        # Crop the frame to valid ROI
        x, y, w, h = roi
        undistorted = cv.undistort(frame, mtx, dist, None, newcameramtx)
        undistorted = cv.resize(undistorted, (600, 400))

        # Run angle detection on the undistorted frame
        angle = angleDetector.angleDetector(undistorted)

        # Show live undistorted video feed
        '''cv.imshow("Live Feed", undistorted)'''

        msg = "x angle: " + str(angle)
        # Update LCD occasionally
        if i >= 50:
            try:
                LCD_print.LCD_print(msg)
            except Exception as e:
                print(f"LCD error: {e}")
            i = 0

        i += 1

        # Exit if 'd' is pressed
        if cv.waitKey(20) & 0xFF == ord('d'):
            print("Exiting live feed...")
            break

    # Cleanup
    capture.release()
    cv.destroyAllWindows()

# End main

# Run main
if __name__ == "__main__":
    main()  # Run main program

