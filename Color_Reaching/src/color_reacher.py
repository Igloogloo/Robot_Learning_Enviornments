#!/usr/bin/env python

import os, time 
import rospy
from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import argparse
import imutils
from continuous_cartesian import go_to_relative
from measure_width_utils import get_width_image, get_width, get_max_width, get_total_width
from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from hlpr_manipulation_utils.manipulator import Gripper
import hlpr_manipulation_utils.transformations as Transform
from hlpr_manipulation_utils.msg import RLTransitionPlusReward
from hlpr_manipulation_utils.srv import GetActionFromObs
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Pose

current_object_pos = None

def update_object_pos(object_pose):
    pos = object_pose.pose.position
    global current_object_pos
    current_object_pos = [pos.x, pos.y, pos.z]

def go_to_start(arm, pose=None, start=False):
    if start is True:
        os.system('rosservice call /j2s7s300_driver/in/home_arm')
    if pose is None:
        target = Pose()
        target.position.x = .52067
        target.position.y = .3734
        target.position.z = .09771
        target.orientation.x = .7127
        target.orientation.y = -.0259
        target.orientation.z = .7009
        target.orientation.w = .0019
        target.orientation.x, target.orientation.y, target.orientation.z = Transform.euler_from_quaternion([.7127, -.0259,.7009, .0019])
    else:
        target = Pose()

    arm.move_to_ee_pose(target)
    orig = arm.get_FK()
    #print(arm.get_FK())
    time.sleep(3)

class ColorReacher():
    def __init__(self, with_pixels=False, max_action=1, n_actions=2, reset_pose=None, episode_time=60, stack_size=4, max_action_true=10):
        """
        with_pixels = True to learn from overhead camera
        max_action: the maximum degree the robot can rotate in xyz cartesian space
        n_actions: xyz cartesian rotations (not advised to change)
        reset_pose: cartesian pose and orientation for the robot to start in
        episode_time: maximum time alloted for each episode
        """
        rospy.init_node("color_reacher", disable_signals=True)
        rospy.Subscriber("/aabl/poi", PoseStamped, update_object_pos, queue_size=1)

        self.arm = ArmMoveIt("j2s7s300_link_base")
        self.grip = Gripper()
        self.with_pixels = with_pixels
        self.max_action = max_action  
        self.max_action_true = max_action_true   
        self.reset_pose = reset_pose   
        self.stack_size = stack_size
        self.episode_time = episode_time
        #self.camera.start()
        self.n_actions = n_actions
        self.action_space = n_actions
        if with_pixels:
            # TODO
            # image = next(self.image_gen)
            # image_list = np.array([image for _ in range(self.stack_size)])
            # self.observation_space = image_list.shape
            self.observation_space = [15]
        else:
            self.observation_space = [15]

        #self.arm.move_to_joint_pose([1.320531, 2.054955, -0.168523, 4.035109, 3.233114, 4.227190, 4.636301])
        time.sleep(1)
        self.grip.open()
        self.grip.close()
        go_to_start(self.arm, self.reset_pose, start=True)
        
        #self.grip.open()
        time.sleep(5)
        self.grip.close(block=False)

        self.cur_time = time.time()
        self.total_time = time.time()
    
    def get_obs(self):
        # Return dicrete observation:[arm joints, eef pose, position of object]

        curr_joints = self.arm.get_current_pose()
        curr_joints = curr_joints.values()
        curr_ee_pose = self.arm.get_FK()

        global current_object_pos
        while current_object_pos is None:
            print("Object has not been detected")
            object_pos = rospy.wait_for_message('/aabl/poi', PoseStamped, timeout=.1)
            pos = object_pos.pose.postion
            current_object_pos = [pos.x, pos.y, pos.z]

        return np.concatenate((current_object_pos,np.concatenate(curr_joints, curr_ee_pose)))

    def step(self, action, action_duration=0.05):
        # Robot will execute given action. New Observation is returned.
        # action_duration specifies 

        if not np.shape(action)[0] == self.n_actions:
            raise ValueError("Action shpae dimensionality mismatch: recieved %x, need %s" % (np.shape(action)[0], self.n_actions))
        if np.isnan(action).any():
            print("NaN DETECTEED")
            action = np.zeros_like(action)
        


