#!/usr/bin/env python

import math, time
import rospy
import numpy as np
from continuous_cartesian import go_to_relative
#from hlpr_manipulation.hlpr_manipulation_utils.src.hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
#import hlpr_manipulation.hlpr_manipulation_utils.src.hlpr_manipulation_utils.transformations as Transform
from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from hlpr_manipulation_utils.manipulator import Gripper
import hlpr_manipulation_utils.transformations as Transform
from geometry_msgs.msg import Pose

rospy.init_node("cartesian_testing", disable_signals=True)
arm = ArmMoveIt("j2s7s300_link_base")
grip = Gripper()
print(arm.get_FK())
target = Pose()
target.position.x = .52067
target.position.y = .3734
target.position.z = .09771
target.orientation.x = .7127
target.orientation.y = -.0259
target.orientation.z = .7009
target.orientation.w = .0019
#target.position.x = .2
#target.position.y = .2
#target.position.z = .1
#x: 1.66533453694e-16
#y: -4.4408920985e-16
#z: 2.22044604925e-16

pos =  {"joint_1":5.267042344822184, 
        "joint_2":1.8907727349229357, 
        "joint_3":3.2263690605702853, 
        "joint_4":1.4864016839566672,
        "joint_5":5.2265820036450465, 
        "joint_6":1.7769064861974533, 
        "joint_7":-4.581260787263586}
print(arm.get_FK())
#for i in range(5):
#    target.position.y -= .2
#    print(target.position.y)
#    arm.move_to_ee_pose(target)
#target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = \
#    Transform.quaternion_from_euler(0.0, math.pi/2, math.pi/2) 
target.orientation.x, target.orientation.y, target.orientation.z = Transform.euler_from_quaternion([.7127, -.0259,.7009, .0019])
#target.orientation.x = -2
print(target.orientation)
#print(Transform.quaternion_from_euler(0.0, math.pi/2, math.pi/2))
#arm.move_to_ee_pose(target)
#go_to_relative([0,0,0,0,0,90])
#time.sleep(2)
grip.open()
#grip.close(block=False)
#print(arm.move_to_joint_pose({'j2s7s300_joint_4': 1.422962656787944, 'j2s7s300_joint_5': 1.9241043264941498, 'j2s7s300_joint_6': 4.186045550966956, 'j2s7s300_joint_7': 4.5712958534560615, 'j2s7s300_joint_2': 1.8077725232314719, 'j2s7s300_joint_3': -2.6089913282425905}
#))
#grip.set_pos(.9)
#arm.move_to_joint_pose(arm.get_IK(target))
#x: -0.174339383732
#y: -0.962369088467
#z: 0.44244129236

#rospy.init_node("cartesian_testing", disable_signals=True)
#print(arm.robot.get_current_state())
#print(arm.get_current_pose())

#print(target.position.x)