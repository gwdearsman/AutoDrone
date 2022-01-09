import I2C_LCD_driver
from time import *
import time
import Adafruit_ADS1x15

mylcd = I2C_LCD_driver.lcd()

mylcd.lcd_display_string("Choose Mode", 1)
mylcd.lcd_display_string(" 1   2   3   4  ", 2)


adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1

string = " 1   2   3   4  "
mode1 = "[1]  2   3   4  "
mode2 = " 1  [2]  3   4  "
mode3 = " 1   2  [3]  4  "
mode4 = " 1   2   3  [4] "
# Main loop.
while True:
    # Read all the ADC channel values in a list.
    y_axis = adc.read_adc(0, gain=GAIN)
    x_axis = adc.read_adc(1, gain=GAIN)

    if x_axis > 30000:
      mode = mode+1
    if x_axis < 10000:
      mode = mode-1
    if mode == 0:
      mode = 4
    if mode == 5:
      mode = 1
    mylcd.lcd_display_string("Choose Mode", 1)
    if mode == 1:
      mylcd.lcd_display_string(mode1, 2)
    if mode == 2:
      mylcd.lcd_display_string(mode2, 2)
    if mode == 3:
      mylcd.lcd_display_string(mode3, 2)
    if mode == 4:
      mylcd.lcd_display_string(mode4, 2)

    # Pause for half a second.
    time.sleep(0.5)
