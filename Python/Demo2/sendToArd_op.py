# sendToArd
#
# Max Smith & Garret Reader
# 11/5/2025
#
# Description

# Imports
from smbus2 import SMBus, i2c_msg
import time

# I2C Set up
# I2C address for the Arduino = 8
ARD_ADDR = 8

MAX_I2C_LENGTH = 32

# Initialize the SMBis library with I2C bus 1
i2c = SMBus(1)

def sendToArd(out: str):

    try:
        data = [ord(c) for c in out[:MAX_I2C_LENGTH - 1]]
        if not data:
            return False
        
        i2c.write_i2c_block_data(ARD_ADDR, 0, data)
        return True
    
    except OSError as e:
        print(f"[I2C Error] {e}")
        time.sleep(0.1)
        return False
    
    except Exception as e:
        print(f"[senToARd] Unexpected error: {e}")
        return False
    
def closeBus():
    try:
        i2c.close()
    except Exception:
        pass

# End sentToArd
