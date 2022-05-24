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

print('Enter roll(x) between -90 to 90 degrees:')
x = float(input())
print('Enter pitch(y) between -180 to 0 degrees:')
y = float(input())

def quaternion_to_euler_angle_vectorized1(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))
    return X, Y, Z

def quaternion_to_euler_angle_vectorized2(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)

    t2 = np.clip(t2, a_min=-1.0, a_max=1.0)
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z

quat = [0.25,  # x
        0.75,   # y
        0.433012701892219,  # z
        0.433012701892219]  # w
print(quaternion_to_euler_angle_vectorized1(quat[0], quat[1], quat[2], quat[3]))
warnings.filterwarnings("error")

#rot = Rotation.from_euler('xyz', [90, 45, 30], degrees=True)
try:
    rot = Rotation.from_quat(quat)
except:
    print('caught during conversion')
# print(rot.as_euler('xyz', degrees=True))

# Create a rotation object from Euler angles specifying axes of rotation
# rot = Rotation.from_euler('xyz', [50, 40, 0], degrees=True)

# Convert to quaternions and print
rot_quat = rot.as_quat()
print(rot_quat)

# rot = Rotation.from_quat(rot_quat)

# Convert the rotation to Euler angles given the axes of rotation
try:
    ang = rot.as_euler('zyx', degrees=True) #zyx yzx yxz returns-> z, y, x
    print(ang)
    print('x: ' + str(round(ang[2], 3)))
    print('y: ' + str(round(ang[1], 3)))
except:
    print('Gimbal lock caught during euler representation')
    warnings.filterwarnings("ignore")
    ang = rot.as_euler('zyx', degrees=True)
    print('x: ' + str(round(ang[0], 3)))
    print('y: ' + str(round(ang[1], 3)))

joint_goal = group.get_current_joint_values()
print("Joints" , joint_goal)
joint_goal[0] = math.radians(x) #-0.5
joint_goal[1] = math.radians(y) #-pi/4
group.go(joint_goal, wait=True)
group.stop()
group.clear_pose_targets()

# pose_goal = geometry_msgs.msg.Pose()
# pose_goal.orientation.w = q[3]#1.0
# pose_goal.position.x = q[0]#0.4
# pose_goal.position.y = q[1]#0.1
# pose_goal.position.z = q[2]#0.4   
# group.set_pose_target(pose_goal)
# group.setPlanningTime(10)
# plan = group.plan()
# print(plan)
#if not plan.joint_trajectory.points:
#    print("Error")

#plan = group.go(wait=True)
