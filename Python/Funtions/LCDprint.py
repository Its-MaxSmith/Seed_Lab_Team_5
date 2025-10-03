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