import subprocess
import I2C_LCD_driver
from time import *
import time
import Adafruit_ADS1x15

mylcd = I2C_LCD_driver.lcd()

mylcd.lcd_display_string("Choose Mode", 1)
mylcd.lcd_display_string(" 1   2   3   4  ", 2)


adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1

mode = 1
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
    if y_axis <12000:
      break
    # Pause for half a second.
    time.sleep(0.25)
mylcd.lcd_display_string("Starting program:", 1)
if mode == 1:
  mylcd.lcd_display_string("Mode 1         ", 2)
  process = subprocess.Popen(["python3","ServoTest.py","--connect","/dev/serial0"], stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE, text = True)
  print(process.stdout.read())
if mode == 2:
  mylcd.lcd_display_string("Mode 2         ", 2)
  process = subprocess.Popen(["python3","deliver1.py","--connect","/dev/serial0"], stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE, text = True)
  print(process.stdout.read())
if mode == 3:
  mylcd.lcd_display_string("Mode 3         ", 2)
  process = subprocess.Popen(["python3","deliver2.py","--connect","/dev/serial0"], stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE, text = True)
  print(process.stdout.read())
if mode == 4:
  mylcd.lcd_display_string("Mode 4         ", 2)
  process = subprocess.Popen(["python3","guideLand.py","--connect","/dev/serial0"], stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE, text = True)
  print(process.stdout.read())
process.wait()
mylcd.lcd_display_string("Program Finished", 1)
print("Completed Script!")
