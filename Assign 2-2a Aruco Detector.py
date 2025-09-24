# Assignment 2a
# Max Smith
# 9/9/25
#
# Python program that looks for an Aruco marker using images in grayscale 
# taken using an atached camera

# Imports for CV
import cv2 as cv
from cv2 import aruco
import numpy as np
import time
from time import sleep

# Imports for LCD
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# LCD print function
def LCD_print(write):

    lcd_colomns = 16
    lcd_rows = 2

    i2c = board.I2C()

    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_colomns, lcd_rows)
    lcd.clear()
    lcd.color = [0, 100, 0]
    lcd.text_direction = lcd.LEFT_TO_RIGHT
    lcd.message = write

# Add the aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

# Set capture as video from the camera
capture = cv.VideoCapture(0)

i = 0

# Display video
while True:
    isTrue, frame = capture.read()

    id = -1

    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) # Convert frame to grayscale

    # Decect marker
    corners, ids, rejected = aruco.detectMarkers(gray_frame, aruco_dict)
    frame = aruco.drawDetectedMarkers(frame, corners, borderColor = 1)


    if not ids is None:
        ids = ids.flatten()
        for (outline, id) in zip(corners, ids):
            markerCorners = outline.reshape((4,2))
            overlay = cv.putText(frame, str(id), (int(markerCorners[0,0]), int(markerCorners[0,1]) - 15), cv.FONT_HERSHEY_TRIPLEX, 0.5, (0,255,0), 2)

    cv.imshow('Video', frame) # Show original video

    if (i >= 50):
        
        if (id == -1):
            LCD_print('Marker not found')

        else:
            LCD_print('id ' + str(id) + ' found')

        i = 0
    i+=1

    if cv.waitKey(20) & 0xFF == ord('d'):
        break


# Clear pointers and windows
capture.release()
cv.destroyAllWindows()

# LCD print function
def LCD_print(write):

    lcd_colomns = 16
    lcd_rows = 2

    i2c = board.I2C()

    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_colomns, lcd_rows)
    lcd.clear()