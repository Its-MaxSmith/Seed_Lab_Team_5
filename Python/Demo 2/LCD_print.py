# LCD_print
#
# Max Smith & Garret Reader
# 10/13/25
#
# Prints a messege to the LCD screen

# Imports for LCD
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

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