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

if __name__ == '__main__':
    global group 

    initializeROS()
    
    # Reading IMU data from excel
    orig_df = pd.read_csv('/home/student/construct_ws/src/robot_manipulator/data/new/home_euler.csv', skiprows=10) # 5 for server 10 for phone
    print(orig_df)
    df = orig_df[['Euler_X', 'Euler_Y', 'Euler_Z']] # Euler_x for server Euler_X for phone
    
    for index, euler in df.iterrows():
        print(euler)
        x, y = imu_to_rviz(euler['Euler_X'], euler['Euler_Y'], euler['Euler_Z'])
        print('x: ' + str(x) + ', y: ' + str(y))
        if(y >= -90 and y <= 90): # base_dummy -> -90 to 90
            if(x >= -180 and x <= 0): # dummy_link -> -180 to 0
                print('setting goal')
                setJointGoal(x, y)

    # Stop the simulation
    group.stop()
    group.clear_pose_targets()
