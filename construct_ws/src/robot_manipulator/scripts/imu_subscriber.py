#!/usr/bin/env python

from itertools import count
import rospy
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation
import warnings
import math
import moveit_commander
import moveit_msgs.msg
import sys
# from pyrr import quaternion
from pyquaternion import Quaternion as qt

def quaternion_to_euler(w, x, y, z):
    q = [x, y, z, w]
    rot = Rotation.from_quat(q)
    #print('Inverse: ', rot.inv().as_quat())
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
    global cnt
    cnt += 1

    joint_goal = group.get_current_joint_values()
    # print("Joints" , joint_goal)
    joint_goal[0] = math.radians(y) # base_dummy
    joint_goal[1] = math.radians(-x) # dummy_link (-x is for mapping angle received from IMU sensor to rviz)
    group.go(joint_goal, wait=True)
	
# def q_mult(q1, q2):
#     w1, x1, y1, z1 = q1
#     w2, x2, y2, z2 = q2
#     print(w1, x1, y1, z1)
#     print(w2, x2, y2, z2)
#     w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
#     x = (w1 * x2 + x1 * w2 + y1 * z2) - z1 * y2
#     y = (w1 * y2 + y1 * w2 + z1 * x2) - x1 * z2
#     z = (w1 * z2 + z1 * w2 + x1 * y2) - y1 * x2
#     return w, x, y, z

def subscriber_callback(msg):
    global cnt
    global qinv

    if(cnt == 0):
        qi = Quaternion()
        qi = msg
        print('Initial quaternion: ', qi)
        # qinv = quaternion.inverse([qi.x, qi.y, qi.z, qi.w])
        pyq = qt([qi.w, qi.x, qi.y, qi.z])
        qinv = pyq.inverse
        print('Inverse: ', qinv)
        x, y = quaternion_to_euler(qi.w, qi.x, qi.y, qi.z)
        print('Initial x: ' + str(x) + ', y: ' + str(y) + '\n')
        setJointGoal(x, y)
    else:
        quat = Quaternion()
        quat = msg
        print(quat)
        # q = quaternion.cross(quat1=[quat.x, quat.y, quat.z, quat.w], quat2=[qinv.x, qinv.y, qinv.z, qinv.w])
        quat1 = qt([quat.w, quat.x, quat.y, quat.z])
        print('Inverse: ', qinv)
        #q = q_mult([quat.w, quat.x, quat.y, quat.z], [qinv.w, qinv.x, qinv.y, qinv.z])
        q = quat1*qinv
        print('Cross: ', q)
        #   q= qt([0.999484275247902, -0.014307749371976, -0.019471021737904, 0.012880901318044])
        x, y = quaternion_to_euler(q.w, q.x, q.y, q.z)#q[0], q[1], q[2], q[3])
        print('x: ' + str(x) + ', y: ' + str(y) + '\n')
        setJointGoal(x, y)
        

if __name__ == "__main__":
    initializeROS()
    rospy.Subscriber("/imu_csv", Quaternion, subscriber_callback)
    rospy.spin()
        