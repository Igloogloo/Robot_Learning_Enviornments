#!/usr/bin/env python

import cv2, os, time, math 
import rospy, tf

from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import argparse
import imutils

#from continuous_cartesian import go_to_relative, go_to_absalute#
#from measure_width_utils import get_width_image, get_width, get_max_width, get_total_width

from continuous_cartesian import go_to_relative, go_to_absalute
from measure_width_utils import get_width_image, get_width, get_max_width, get_total_width

from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from hlpr_manipulation_utils.manipulator import Gripper
import hlpr_manipulation_utils.transformations as Transform
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

import moveit_msgs

def get_IK(pose, arm):
        rospy.wait_for_service('compute_ik')
        compute_ik = rospy.ServiceProxy('compute_ik', moveit_msgs.srv.GetPositionIK)
        
        wkPose = pose
        #print(pose)
        """if root is None:
            wkPose.header.frame_id = self.group[group_id].get_planning_frame() # name:odom
        else:
            wkPose.header.frame_id = root"""
        
        wkPose.header.stamp=rospy.Time.now()

        msgs_request = moveit_msgs.msg.PositionIKRequest()
        msgs_request.group_name = "arm" # name: arm
        msgs_request.robot_state = arm.robot.get_current_state()
        msgs_request.robot_state.joint_state.header.frame_id='j2s7s300_link_base'
        msgs_request.robot_state.multi_dof_joint_state.header.frame_id='j2s7s300_link_base'
        msgs_request.pose_stamped = wkPose
        #msgs_request.ik_link_names = ['j2s7s300_link_1','j2s7s300_link_2',
        #                                'j2s7s300_link_3','j2s7s300_link_4','j2s7s300_link_5','j2s7s300_link_6','j2s7s300_link_7']
        msgs_request.timeout.secs = 2
        msgs_request.avoid_collisions = False
        msgs_request.robot_state.is_diff = True
        #print(msgs_request)

        try:
            jointAngle=compute_ik(msgs_request)
            #print jointAngle
            ### Made change here from: ans=list(jointAngle.solution.joint_state.position[1:7])
            ans=list(jointAngle.solution.joint_state.position[2:9])
            if jointAngle.error_code.val == -31:
                rospy.logerr('No IK solution')
                return None
            return ans
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {}".format(e))



rospy.init_node("joint_pos", disable_signals=True)
collison_service = rospy.ServiceProxy('/check_state_validity', GetStateValidity)


arm = ArmMoveIt(planning_frame="j2s7s300_link_base")
#arm = ArmMoveIt(planning_frame="base_link")
grip = Gripper()

#joints = arm.get_current_pose()
#joints['j2s7s300_joint_1'] = 1.57
#3.349011450902082, 0.8204698379017197, 5.716069487949769, 2.7672169514278173, 
#6.728805145730037, 5.132135380416333, -0.4256149432976181
joints = {"j2s7s300_joint_1":3.349,
        "j2s7s300_joint_2":0.82, 
        "j2s7s300_joint_3":5.716, 
        "j2s7s300_joint_4":2.767, 
        "j2s7s300_joint_5":6.7288, 
        "j2s7s300_joint_6":5.132, 
        "j2s7s300_joint_7":-0.456}
joints = {"j2s7s300_joint_1":1.57,
        "j2s7s300_joint_2":1.57, 
        "j2s7s300_joint_3":5.716, 
        "j2s7s300_joint_4":2.767, 
        "j2s7s300_joint_5":6.7288, 
        "j2s7s300_joint_6":5.132, 
        "j2s7s300_joint_7":-0.456}
#print(joints)
#arm.move_to_joint_pose(joints)
#print(arm.robot.get_planning_frame())
sorted_joints = [arm.get_current_pose()[x] for x in sorted(arm.get_current_pose())]
#print(sorted_joints)
#print(arm.get_current_pose()[sorted(arm.get_current_pose())])
#print(arm.get_FK()[0], "APPLE")
#robot_state = arm.state_from_joints(arm.get_current_pose())

pose = arm.get_FK()[0]
print pose

ik_joints = get_IK(pose, arm)
#print(ik_joints)

#exit()
ik_joints = {"j2s7s300_joint_1":ik_joints[0], 
        "j2s7s300_joint_2":ik_joints[1], 
        "j2s7s300_joint_3":ik_joints[2], 
        "j2s7s300_joint_4":ik_joints[3], 
        "j2s7s300_joint_5":ik_joints[4], 
        "j2s7s300_joint_6":ik_joints[5], 
        "j2s7s300_joint_7":ik_joints[6]}

#print(arm.get_IK(arm.get_FK()[0].pose))
#print(moveit_msgs.srv.GetPositionIK)
#print(robot_state)
robot_state = arm.state_from_joints(ik_joints)
print(arm.get_FK(state=robot_state)[0])

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