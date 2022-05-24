#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation
import warnings
import math
import moveit_commander
import moveit_msgs.msg
import sys

def quaternion_to_euler(w, x, y, z):
    q = [x, y, z, w]
    rot = Rotation.from_quat(q)
    try:
        ang = rot.as_euler('zyx', degrees=True) #zyx yzx yxz returns-> z, y, x
        print(ang)
        X = round(ang[2], 3)
        Y = round(ang[1], 3)
    except:
        print('Gimbal lock caught during euler representation')
        warnings.filterwarnings("ignore")
        ang = rot.as_euler('zyx', degrees=True)
        X = round(ang[0], 3)
        Y = round(ang[1], 3)
    return X, Y

def initializeROS():
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
    # print("Joints" , joint_goal)
    joint_goal[0] = math.radians(y) # base_dummy
    joint_goal[1] = math.radians(-x) # dummy_link (-x is for mapping angle received from IMU sensor to rviz)
    group.go(joint_goal, wait=True)

def subscriber_callback(msg):
    quat = Quaternion()
    quat = msg
    print(quat)
    x, y = quaternion_to_euler(quat.w, quat.x, quat.y, quat.z)
    print('x: ' + str(x) + ', y: ' + str(y) + '\n')
    setJointGoal(x, y)

if __name__ == "__main__":
    initializeROS()
    rospy.Subscriber("/imu_csv", Quaternion, subscriber_callback)
    rospy.spin()
        