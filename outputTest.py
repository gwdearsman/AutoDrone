import I2C_LCD_driver
from time import *

mylcd = I2C_LCD_driver.lcd()

mylcd.lcd_display_string("Choose Mode", 1)
mylcd.lcd_display_string(" 1   1   1   1  ", 2)

