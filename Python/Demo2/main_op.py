# Demo 2 Main
# 
# Max Smith & Garret Reader
# 11/5/2025
#
# Description

# Imports for CV
import cv2 as cv
from cv2 import aruco
import numpy as np
import time

# Imports for I2C
from smbus2 import SMBus
from time import sleep

# Imports for function files
import angleDetector_op
import RGcolorDetector_op
import sendToArd_op
import distanceDetector_op
from lcd_updater import LCDupdater

# Main program
def main():

    # Load calibration data
    data = np.load('camera_calibration_data.npz')
    mtx, dist = data['mtx'], data['dist']

    # Start video capture
    capture = cv.VideoCapture(0)
    if not capture.isOpened():
        print("Could not open camera.")
        return
    
    ret, frame = capture.read()
    if not ret:
        print("Failed to read first frame.")
        return

    h, w = frame.shape[:2]

    # Create undistortion map
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), 5)

    # Start LCD updater
    lcd_updater = LCDupdater(update_interval=1.0)
    lcd_updater.start()

    while True:
        ret, frame = capture.read()
        if not ret:
            print("Failed to grab frame.")
            break

        # Undistort frame
        undistorted = cv.remap(frame, mapx, mapy, cv.INTER_LINEAR)

        # Run angle detection on the undistorted frame
        angle = angleDetector_op.angleDetector(undistorted)

        # Run color detection
        color = RGcolorDetector_op.RGcolorDetector(undistorted)

        # Run distance detector
        forward = distanceDetector_op.distanceDetector(undistorted)

        msg = f"x angle: {angle}\n{color} {forward}"

        if color == 'N' & angle != 'S':
            color = 'E' #End condition if no color detected

        # Send angle and color to arduino
        print(forward + "," + angle + "," + color)
        sendToArd_op.sendToArd(forward + "," + angle + "," + color)

        # Update LCD
        lcd_updater.update_message(msg)
        
        # Exit if 'd' is pressed
        if cv.waitKey(1) & 0xFF == ord('d'):
            print("Exiting")
            break

    # Cleanup
    lcd_updater.stop()
    capture.release()
    cv.destroyAllWindows()
    sendToArd_op.closeBus()

# End main

# Run main
if __name__ == "__main__":
    main() 
