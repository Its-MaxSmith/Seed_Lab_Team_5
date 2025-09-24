# Assignment 2 1a Python
# Max Smith
# 9/8/25
#
# Python program that links to an Arduino to send a string from the Pi to
# the Arduino and displays the string character and ASCII codes

# Imports
from smbus2 import SMBus
from time import sleep

# I2C address for the Arduino = 8, also set in Arduino code
ARD_ADDR = 8

# Initialize the SMBis library with I2C bus 1
i2c = SMBus(1)

# Body code - infinite while
while(True):
    
    # Read in offset
    offset = int(input("Enter an offset (7 to quit): "))

    # Exit key
    if(offset == 7):
        break

    # Read in string
    string = input("Enter in string:")

    # Write to i2c bus
    command = [ord(character) for character in string]
    i2c.write_i2c_block_data(ARD_ADDR, offset, command)
