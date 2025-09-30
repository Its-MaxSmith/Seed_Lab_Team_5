# Python program that will take a video and output to an arduino the location of an Aruco marker
# in one of four quadrants of the video frame

# Imports for CV
import cv2 as cv
from cv2 import aruco
import numpy as np
import arucoDetector
import LCDprint
import time

capture = cv.VideoCapture(0)

while True:
    isTrue, frame = capture.read()
    location = arucoDetector.arucoDetector(frame)
    cv.imshow('Video', frame) # Show original video
    match location:
        case 0: 
            out = 0 
            msg = "No Marker", # No marker detected                
        case 1: 
            out = 00, # Marker in bottom left
            msg = "Desired Location: 0, 0", 
        case 2: 
            out = 01, # Marker in bottom right
            msg = "Desired Location: 0, 1",
        case 3: 
            out = 10, # Marker in top left
            msg = "Desired Location: 1, 0",
        case 4: 
            out = 11  # Marker in top right
            msg = "Desired Location: 1, 1",
    if (out != 0 ):
        # Send signal to arduino)

        LCDprint.LCD_print(msg)
    else:
        LCDprint.LCD_print("No Marker")
    
    # Clear pointers and windows
    capture.release()
    cv.destroyAllWindows()