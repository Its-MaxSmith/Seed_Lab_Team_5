# Python program that will take a video and output to an arduino the location of an Aruco marker
# in one of four quadrants of the video frame

# Imports for CV
import cv2 as cv
from cv2 import aruco
import numpy as np
import arucoDetector
import LCDprint
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

    while True:
        isTrue, frame = capture.read()
        location = arucoDetector.arucoDetector(frame)
        cv.imshow('Video', frame) # Show original video
        match location:
            case 0: 
                out = "0" 
                msg = "No Marker", # No marker detected                
            case 1: 
                out = "00", # Marker in bottom left
                msg = "Desired Location: 0, 0", 
            case 2: 
                out = "01", # Marker in bottom right
                msg = "Desired Location: 0, 1",
            case 3: 
                out = "10", # Marker in top left
                msg = "Desired Location: 1, 0",
            case 4: 
                out = "11",  # Marker in top right
                msg = "Desired Location: 1, 1",
        if (out != "0" ):
            # Send signal to arduino)

            LCDprint.LCD_print(msg)
            sendToArd(out)
        else:
            LCDprint.LCD_print("No Marker")
        
        # Clear pointers and windows
        capture.release()
        cv.destroyAllWindows()
# End Main

# Functions
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