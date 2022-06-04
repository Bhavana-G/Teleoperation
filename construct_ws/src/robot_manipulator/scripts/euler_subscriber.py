#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Quaternion
import math
import moveit_commander
import moveit_msgs.msg
import sys

def initializeROS():
    global cnt
    cnt = 0
    # Initialization
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('master', anonymous=True)

    # Initialize robot
    group_name = 'arm'
    global group
    group = moveit_commander.MoveGroupCommander(group_name)

    # Publish trajectory - to be used by Moveit
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)

def setJointGoal(x, y):
    global group

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = math.radians(y) # base_dummy
    joint_goal[1] = math.radians(x) # dummy_link
    group.go(joint_goal, wait=True)

def imu_to_rviz(x, y, z):
    # X of ROS is y of IMU
    y = y * -1
    X = y - 90

    # Y of ROS is z of IMU
    Y = z + 90

    return X, Y
	
def subscriber_callback(msg):
    global qinv

    euler = Quaternion()
    euler = msg
    print(euler)
    x, y = imu_to_rviz(euler.x, euler.y, euler.z)
    print('x: ' + str(x) + ', y: ' + str(y) + '\n')
    if(y >= -90 and y <= 90): # base_dummy -> -90 to 90
        if(x >= -180 and x <= 0): # dummy_link -> -180 to 0
            print('setting goal')
            setJointGoal(x, y)

if __name__ == "__main__":
    initializeROS()
    rospy.Subscriber("/euler_csv", Quaternion, subscriber_callback)
    rospy.spin()
        