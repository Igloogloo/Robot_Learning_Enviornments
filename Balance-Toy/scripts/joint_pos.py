#!/usr/bin/env python

import cv2, os, time 
import rospy
from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import argparse
import imutils
from continuous_cartesian import go_to_relative, go_to_absalute
from measure_width_utils import get_width_image, get_width, get_max_width, get_total_width
from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from hlpr_manipulation_utils.manipulator import Gripper
import hlpr_manipulation_utils.transformations as Transform
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

rospy.init_node("joint_pos", disable_signals=True)
collison_service = rospy.ServiceProxy('/check_state_validity', GetStateValidity)

arm = ArmMoveIt("j2s7s300_link_base")
grip = Gripper()

joints = {"j2s7s300_joint_1":0.3037605866, 
        "j2s7s300_joint_2":0.9946144349, 
        "j2s7s300_joint_3":3.29652311589, 
        "j2s7s300_joint_4":5.267673841, 
        "j2s7s300_joint_5":4.14327215301, 
        "j2s7s300_joint_6":1.69886481408, 
        "j2s7s300_joint_7":2.72438278}

#print(joints)
#print(arm.get_current_pose())
#print(arm.get_FK(), "APPLE")
robot_state = arm.state_from_joints(arm.get_current_pose())
# robot_state = arm.state_from_joints(joints)
#robot_state.joint_state.name=['j2s7s300_joint_1', 'j2s7s300_joint_2', 'j2s7s300_joint_3', 'j2s7s300_joint_4', 'j2s7s300_joint_5', 'j2s7s300_joint_6', 'j2s7s300_joint_7']
validityRequest = GetStateValidityRequest()
validityRequest.robot_state=robot_state
#print(robot_state)
print(collison_service(validityRequest))
print(collison_service(validityRequest).contacts[0].contact_body_1)
print(collison_service(validityRequest).contacts[0].contact_body_2)
"if contact body 1 or 2 contains j2s7s300 or gripper"
#arm.move_to_joint_pose([1.320531, 2.054955, -0.168523, 4.035109, 3.233114, 4.227190, 4.636301])