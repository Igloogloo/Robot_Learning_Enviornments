#!/usr/bin/env python

import rospy
import numpy as np
from sqlalchemy import Float
from sphero_interface.msg import *
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Point, Quaternion
from kinova_msgs.msg import ArmJointAnglesGoal, ArmJointAnglesAction
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from actionlib_msgs.msg import GoalStatusArray

current_observation = np.zeros(26)

def joint_update(data):
    arm_joints = list(data.position)[5:12]
    current_observation[:7] = arm_joints

def eef_update(data):
    eef_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
    eef_orientation = [data.pose.orientation.x, data.pose.orientation.z, \
         data.pose.orientation.y, data.pose.orientation.w]
    current_observation[8:11] = eef_pose

def sphero_pose_update(data):
    sphero_pose = [data.x, data.y]
    current_observation[11:13] = sphero_pose

def sphero_light_update(data):
    current_observation[13] = data.data

# def rate_testing(data):
#     status = data.header.seq
#     current_observation[12] = status

def observationPublisher():
    pub = rospy.Publisher('rl_observation', ObsMessage, queue_size=1)
    rospy.Subscriber("/joint_states", JointState, callback=joint_update)
    rospy.Subscriber("/j2s7s300_driver/out/tool_pose", PoseStamped, callback=eef_update)
    #rospy.Subscriber("/move_group/status", GoalStatusArray, callback=rate_testing)
    rospy.Subscriber("/red_sphero_cord", Point, callback=sphero_pose_update)
    rospy.Subscriber("/sphero_light", Float32, callback=sphero_light_update)
    rospy.init_node('observation_pub', anonymous=True)
    rate = rospy.Rate(1000) # 10hz
    while not rospy.is_shutdown():
        pub.publish(ObsMessage(current_observation.tolist()))
        rate.sleep()

if __name__ == '__main__':
    try:
        observationPublisher()
    except rospy.ROSInterruptException:
        pass