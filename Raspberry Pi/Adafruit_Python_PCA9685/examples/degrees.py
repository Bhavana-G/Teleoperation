
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

# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096

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

def deg_to_pwm(deg):
    res_pwm = (int(deg) - 1)/(180 - 1) * (600-150) + 150
    print(res_pwm)
    return res_pwm

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

degrees = sys.argv[1]
pwm_value = int(deg_to_pwm(degrees))

print('Moving servo on channel 0, press Ctrl-C to quit...')
while True:
    # Move servo on channel O between extremes.
    pwm.set_pwm(15, 0, servo_min)
    time.sleep(1)
#    pwm.set_pwm(0, 0, servo_max)
#    time.sleep(1)
    pwm.set_pwm(15, 0, pwm_value)
    time.sleep(1)
