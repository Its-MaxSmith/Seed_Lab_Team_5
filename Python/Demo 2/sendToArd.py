# sendToArd
#
# Max Smith & Garret Reader
# 11/5/2025
#
# Description

# I2C Set up
# I2C address for the Arduino = 8
ARD_ADDR = 8
# Initialize the SMBis library with I2C bus 1
i2c = SMBus(1)

def sendToArd(out):

    offset = 0 # Pre set to 0

    command = [ord(character) for character in out]
    i2c.write_i2c_block_data(ARD_ADDR, offset, command)
# End sentToArd