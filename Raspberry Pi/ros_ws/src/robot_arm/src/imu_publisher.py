#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Quaternion
import pandas as pd

def publish_topic():
	pub = rospy.Publisher('/imu_csv', Quaternion, queue_size=10)
	rospy.init_node('raspberry', anonymous=True)
	
	rate = rospy.Rate(10) # 10hz
	orig_df = pd.read_csv("/home/ubuntu/ros_ws/src/robot_arm/data/quat.csv", skiprows=5)
	df = orig_df[['Quaternion_w', 'Quaternion_x', 'Quaternion_y', 'Quaternion_z']]
	print(df)
	for index, quat in df.iterrows():
		if(index-1 > 0):
			prev = df.iloc[[index-1]]
			diff = float(prev['Quaternion_w'] - quat['Quaternion_w'])
			if(abs(diff) > 0.001):
				msg = Quaternion()
				msg.x = quat['Quaternion_x']
				msg.y = quat['Quaternion_y']
				msg.z = quat['Quaternion_z']
				msg.w = quat['Quaternion_w']
		
				pub.publish(msg)
				logging_str = "Quaternion(%f, %f, %f, %f) published at: %s" % (msg.x, msg.y, msg.z, msg.w, rospy.get_time())
				rospy.loginfo(logging_str)
		
				rate.sleep()

if __name__ == '__main__':
	try:
		publish_topic()
	except rospy.ROSInterruptException:
		pass
		
