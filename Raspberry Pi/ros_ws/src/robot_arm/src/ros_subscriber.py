#!/usr/bin/env python

from __future__ import division
import rospy
from sensor_msgs.msg import JointState
import time
import sys
import Adafruit_PCA9685
import math

class SubscriberClass:
	def __init__(self):
		self.subscriber = rospy.Subscriber("/joint_states", JointState, self.subscriber_callback)
		self.data = JointState()
		
	def subscriber_callback(self, msg):
		self.data = msg
		#print(self.data.position[0])
		degrees1 = math.degrees(self.data.position[0]) # base_dummy
		degrees2 = math.degrees(self.data.position[1]) # dummy_link
		if(degrees1 == 0 and degrees2 == 0):
			return
		print(degrees1,degrees2)
		pwm_value1 = int(deg_to_pwm(rviz_to_robot(degrees1, 1), 1))
		pwm_value2 = int(deg_to_pwm(rviz_to_robot(degrees2, 2), 2))
		pwm.set_pwm(15, 0, pwm_value1)
		pwm.set_pwm(0, 0, pwm_value2)
		
pwm = Adafruit_PCA9685.PCA9685()

# robot angles
min_angle = 0
max_angle = 180
# base_dummy PWM
servo_min1 = 100
servo_max1 = 440
# dummy_link PWM
servo_min2 = 150
servo_max2 = 575

# rviz angles
min_rviz = 0
max_rviz = 360
# base_dummy
rviz_min1 = 90
rviz_max1 = 270
# dummy_link
rviz_min2 = 0 
rviz_max2 = 180

pwm.set_pwm_freq(60)

def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    #print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    #print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)
    
def rviz_to_robot(deg, motor):
    if motor == 1: # base_dummy
    	deg = deg * -1
    	deg = deg + 90
    	#rviz_deg = (deg - min_rviz)/(max_rviz - min_rviz) * (rviz_max1 - rviz_min1) + rviz_min1
    	#res_deg = (rviz_deg - rviz_min1)/(rviz_max1 - rviz_min1) * (max_angle - min_angle) + min_angle
    elif motor == 2: # dummy_link
    	deg = deg + 180
    	#res_deg = (deg - min_rviz)/(max_rviz - min_rviz) * (rviz_max2 - rviz_min2) + rviz_min2
    print(deg)
    return deg

def deg_to_pwm(deg, motor):
    if motor == 1: # base_dummy
    	res_pwm = (int(deg) - min_angle)/(max_angle - min_angle) * (servo_max1 - servo_min1) + servo_min1
    elif motor == 2: # dummy_link
    	res_pwm = (int(deg) - min_angle)/(max_angle - min_angle) * (servo_max2 - servo_min2) + servo_min2
    #print(res_pwm)
    return res_pwm

if __name__ == "__main__":
	rospy.init_node("raspberry", anonymous=True) # Create a node
	sub = SubscriberClass()
	rospy.spin() # Ensures to read the topic forever
