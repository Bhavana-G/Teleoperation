import pandas as pd
import moveit_commander
import rospy
import moveit_msgs.msg
import numpy as np
import sys
from scipy.spatial.transform import Rotation
import warnings
import math

def initializeROS():
    # Initialization
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('input_pose', anonymous=True)

    # Initialize robot
    group_name = 'arm'
    global group
    group = moveit_commander.MoveGroupCommander(group_name)

    # Publish trajectory - to be used by Moveit
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)

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

def setJointGoal(x, y):
    global group
    joint_goal = group.get_current_joint_values()
    # print("Joints" , joint_goal)
    joint_goal[0] = math.radians(y) # base_dummy
    joint_goal[1] = math.radians(-x) # dummy_link (-x is for mapping angle received from IMU sensor to rviz)
    group.go(joint_goal, wait=True)

if __name__ == '__main__':
    global group 

    initializeROS()
    
    # Reading IMU data from excel
    orig_df = pd.read_excel('/home/student/construct_ws/src/robot_manipulator/data/Imp1 copy.xlsx', skiprows=7)
    df = orig_df[['Quat_W', 'Quat_X', 'Quat_Y', 'Quat_Z']]
    
    for index, quat in df.iterrows():
        x, y = quaternion_to_euler(quat['Quat_W'], quat['Quat_X'], quat['Quat_Y'], quat['Quat_Z'])
        print(quat['Quat_W'], quat['Quat_X'], quat['Quat_Y'], quat['Quat_Z'])
        print('x: ' + str(x) + ', y: ' + str(y))
        setJointGoal(x, y)

    # Stop the simulation
    group.stop()
    group.clear_pose_targets()
