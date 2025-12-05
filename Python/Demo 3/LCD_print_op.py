# LCD_print
#
# Max Smith & Garret Reader
# 10/13/25
#
# Prints a messege to the LCD screen

# Imports for LCD
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# Initialize LCD once
LCD_COLUMNS = 16
LCD_ROWS = 2

# Global variables for LCD
_i2c = None
_lcd = None
_last_message = ""

def _init_lcd():

    global _i2c, _lcd
    if _lcd is None:
        _i2c = board.I2C()
        _lcd = character_lcd.Character_LCD_RGB_I2C(_i2c, LCD_COLUMNS, LCD_ROWS)
        _lcd.color = [0, 100, 0]
        _lcd.text_direction = _lcd.LEFT_TO_RIGHT
        _lcd.clear()
    return _lcd

def LCD_print(msg: str):
    global _last_message
    lcd = _init_lcd()

    # Avoid redundant updates
    if msg == _last_message:
        return

    _last_message = msg

    # Split lines if needed
    lines = msg.split("\n")
    if len(lines) > 2:
        lines = lines[:2]  # only 2 rows available
    formatted = "\n".join(line[:LCD_COLUMNS] for line in lines)

    try:
        lcd.clear()
        lcd.message = formatted
    except Exception as e:
        print(f"LCD write error: {e}")
