
from __future__ import division
import time
import sys

# Import the PCA9685 module.
import Adafruit_PCA9685


# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Configure min and max servo pulse lengths (per degree it is 2.36111) for motor 1
servo_min1 = 150  # Min pulse length out of 4096
servo_max1 = 575  # Max pulse length out of 4096

# Configure min and max servo pulse lengths (per degree it is 1.69444) for motor 2
servo_min2 = 100
servo_max2 = 440

# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

def deg_to_pwm(deg, motor):
    if motor == 1:
    	res_pwm = (int(deg) - 0)/(180 - 0) * (servo_max1 - servo_min1) + servo_min1
    elif motor == 2:
    	res_pwm = (int(deg) - 0)/(180 - 0) * (servo_max2 - servo_min2) + servo_min2
    print(res_pwm)
    return res_pwm

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

degrees1 = sys.argv[1]
degrees2 = sys.argv[2]
pwm_value1 = int(deg_to_pwm(degrees1, 1))
pwm_value2 = int(deg_to_pwm(degrees2, 2))
#pwm_value1 = int(degrees1)
#pwm_value2 = int(degrees2)
print('Moving servo on channel 0, press Ctrl-C to quit...')
#while True:
    # Move servo on channel O between extremes.
#    pwm.set_pwm(0, 0, servo_min)
#    time.sleep(1)
#    pwm.set_pwm(0, 0, servo_max)
#    time.sleep(1)
pwm.set_pwm(0, 0, pwm_value1)
time.sleep(1)
print('Moving servo on channel 15, press Ctrl-C to quit...')
pwm.set_pwm(15, 0, pwm_value2)
time.sleep(1)
