# Mini Project Python Code
# Max Smith & Garret Reader
# 10/3/25
# 
# Python program that will take a video and output to an arduino the location of an Aruco marker
# in one of four quadrants of the video frame

# Imports for CV
import cv2 as cv
from cv2 import aruco
import numpy as np
import time

# Imports for I2C
from smbus2 import SMBus
from time import sleep

# Imports for LCD
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# I2C Set up
# I2C address for the Arduino = 8
ARD_ADDR = 8
# Initialize the SMBis library with I2C bus 1
i2c = SMBus(1)


def main(): 
    capture = cv.VideoCapture(0)
    
    i = 0
    
    while True:
        isTrue, frame = capture.read()
        location = arucoDetector(frame)
        cv.imshow('Video', frame) # Show original video
        match location:
            case 0: 
                out = "0" 
                msg = "No Marker" # No marker detected                
            case 1: 
                out = "11" # Marker in bottom left
                msg = "Desired Location\n[1, 1]"
            case 2: 
                out = "01" # Marker in bottom right
                msg = "Desired Location\n[0, 1]"
            case 3: 
                out = "10" # Marker in top left
                msg = "Desired Location\n[1, 0]"
            case 4: 
                out = "00"  # Marker in top right
                msg = "Desired Location\n[0, 0]"
        if (i >= 50):
            if (out != "0" ):
                # Send signal to arduino)
                LCD_print(msg)
                sendToArd(out)
            else:
                LCD_print("No Marker")

            i = 0
        i+=1
        if cv.waitKey(20) & 0xFF == ord('d'):
            # Clear pointers and windows
            capture.release()
            cv.destroyAllWindows()
            break

# End Main

# Functions
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
    return location



# LCD print function
def LCD_print(msg):

    lcd_coloms = 16
    lcd_rows = 2

    i2c = board.I2C()

    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_coloms, lcd_rows)
    lcd.clear()
    lcd.color = [0, 100, 0]
    lcd.text_direction = lcd.LEFT_TO_RIGHT
    lcd.message = msg
# End LCD_print

# Send packet to arduino
def sendToArd(out):

    offset = 0 # Pre set to 0

    command = [ord(character) for character in out]
    i2c.write_i2c_block_data(ARD_ADDR, offset, command)
# End sentToArd


if __name__ == "__main__":
    main()  # Run main program