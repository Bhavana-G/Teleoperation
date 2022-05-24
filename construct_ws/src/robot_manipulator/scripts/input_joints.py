#!/usr/bin/env python

import sys
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import numpy as np
from scipy.spatial.transform import Rotation
import warnings

# Initialization
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('input_pose', anonymous=True)

# Initialize robot
robot = moveit_commander.RobotCommander() # info about robot
group_name = 'arm'
group = moveit_commander.MoveGroupCommander(group_name)

# Publish trajectory - to be used by Moveit
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

print('Enter roll(x) between -180 to 0 degrees:')
x = float(input())
print('Enter pitch(y) between -90 to 90 degrees:')
y = float(input())

joint_goal = group.get_current_joint_values()
print("Joints" , joint_goal)
joint_goal[0] = math.radians(y) #-0.5
joint_goal[1] = math.radians(x) #-pi/4
group.go(joint_goal, wait=True)
group.stop()
group.clear_pose_targets()
