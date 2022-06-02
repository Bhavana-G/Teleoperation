#  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
# / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
#| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
# \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
#                  (____/ 
# Osoyoo MG995 Raspberry Pi programming example
# tutorial url: https://osoyoo.com/?p=39526
from __future__ import division
import time
import RPi.GPIO as GPIO
import Adafruit_PCA9685
pwm = Adafruit_PCA9685.PCA9685()

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

ob_range=30
 
pwm.set_pwm(0, 0, 100)
time.sleep(5)
pwm.set_pwm(0, 0, 0)
 
 
