#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

class SubscriberClass:
	def __init__(self):
		self.subscriber = rospy.Subscriber("/joint_states", JointState, self.subscriber_callback)
		self.data = JointState()
		
	def subscriber_callback(self, msg):
		self.data = msg
		print(self.data)
		
if __name__ == "__main__":
	rospy.init_node("raspberry", anonymous=True) # Create a node
	sub = SubscriberClass()
	rospy.spin() # Ensures to read the topic forever
