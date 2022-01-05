#!/usr/bin/env python

import os, time

from numpy.lib.shape_base import column_stack 
import rospy, tf
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
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Point, Quaternion
import actionlib
from kinova_msgs.msg import ArmJointAnglesGoal, ArmJointAnglesAction

current_object_pos = None
arm_frame = "j2s7s300_link_base"
camera_frame = "camera_color_optical_frame"
client = actionlib.SimpleActionClient('/j2s7s300_driver/joints_action/joint_angles', ArmJointAnglesAction)

def update_object_pos(object_pose):
    pos = object_pose.pose.position
    global current_object_pos
    current_object_pos = [pos.x, pos.y, pos.z]

def go_to_start(arm, pose=None, start=False):
    if start is True:
        os.system('rosservice call /j2s7s300_driver/in/home_arm')
    if pose is None:
        target = Pose()
        target.position.x = 0.06439
        target.position.y = -0.4431
        target.position.z = 0.02324
        target.orientation.x = 0.7092844
        target.orientation.y = -0.70425379
        target.orientation.z = 0.029602
        target.orientation.w = -0.008121
        target.orientation.x, target.orientation.y, target.orientation.z = Transform.euler_from_quaternion([target.orientation.x, 
                                                                                    target.orientation.y,target.orientation.z, target.orientation.w])
        # joints: 1.088973027899044, 2.266060868363049, 3.729168814371174, 1.2807426640427695, 5.403782350992303, 3.7993063574717194, 2.244924690371635
    else:
        target = Pose()
    joints = {"j2s7s300_joint_1":1.088, "j2s7s300_joint_2":2.268163, "j2s7s300_joint_3":3.7316721, "j2s7s300_joint_4":1.280286,
                 "j2s7s300_joint_5":5.4074, "j2s7s300_joint_6":3.80056, "j2s7s300_joint_7":2.246956}

    # client.wait_for_server()
    # goal = ArmJointAnglesGoal()

    # goal.angles.joint1 = 1.088
    # goal.angles.joint2 = 2.266
    # goal.angles.joint3 = 3.729
    # goal.angles.joint4 = 1.28
    # goal.angles.joint5 = 5.40
    # goal.angles.joint6 = 3.799
    # goal.angles.joint7 = 2.244
    # print(goal.angles)
    # #print(goal.angles)
    # client.send_goal(goal)
    # client.wait_for_result()

    #arm.move_to_ee_pose(target)
    arm.move_to_joint_pose(joints)
    # actionlib.(joints)
    orig = arm.get_FK()
    #print(arm.get_FK())
    time.sleep(3)

class ColorReacher():
    def __init__(self, with_pixels=False, max_action=1, n_actions=2, reset_pose=None, episode_time=60, stack_size=4, max_action_true=10,
                    sparse_rewards=False, success_threshold=.01):
        """
            with_pixels = True to learn from overhead camera
            max_action: the maximum degree the robot can rotate in xyz cartesian space
            n_actions: xyz cartesian rotations (not advised to change)
            reset_pose: cartesian pose and orientation for the robot to start in
            episode_time: maximum time alloted for each episode
            sparse_rewards: if rewards are set to sparse, the robot will recieve -1 while not touching
                the object and 0 for touching it. Else, the reward will be the current euclidiean 
                distance between the eef and the object.
            success_threshold: the minimum distance for the object to have been considered reached
            (TODO: test a good success_threshold)
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
        self.sparse_rewards = sparse_rewards
        self.success_threshold = success_threshold
        self.tf_listener = tf.TransformListener()
        if with_pixels:
            # TODO
            # image = next(self.image_gen)
            # image_list = np.array([image for _ in range(self.stack_size)])
            # self.observation_space = image_list.shape
            self.observation_space = [16]
        else:
            self.observation_space = [16]

        #self.arm.move_to_joint_pose([1.320531, 2.054955, -0.168523, 4.035109, 3.233114, 4.227190, 4.636301])
        time.sleep(1)
        #self.grip.open()
        self.grip.close()
        go_to_start(self.arm, self.reset_pose, start=False)
        
        #self.grip.open()
        #time.sleep(5)
        #self.grip.close(block=False)

        self.cur_time = time.time()
        self.total_time = time.time()

    def render(self):
        # TODO
        pass
        
    def get_obs(self):
        # Return dicrete observation:[arm joints, eef pose, position of object]

        curr_joints = self.arm.get_current_pose()
        curr_joints = curr_joints.values()
        curr_ee_pose = self.arm.get_FK()[0]
        curr_ee_pose = [curr_ee_pose.pose.position.x, curr_ee_pose.pose.position.y, curr_ee_pose.pose.position.z,curr_ee_pose.pose.orientation.x,  
                                                            curr_ee_pose.pose.orientation.y, curr_ee_pose.pose.orientation.z]

        global current_object_pos
        while current_object_pos is None:
            print("Object has not been detected")
            object_pos = rospy.wait_for_message('/aabl/poi', PoseStamped, timeout=.1)
            object_cameraFrame = PoseStamped()
            # p = point
            q = Quaternion()
            q.x = 0 #rot[0]
            q.y = 0 #rot[1]
            q.z = 0 #rot[2]
            q.w = 1. #rot[3]
            object_cameraFrame.pose.position = object_pos.pose.position
            object_cameraFrame.pose.orientation = q
            object_cameraFrame.header.frame_id = camera_frame
            goal_armFrame = self.tf_listener.transformPose(arm_frame, object_cameraFrame)
            pos = goal_armFrame.pose.postion
            # NOT SURE IF TF IS RIGHT
            current_object_pos = [pos.x, pos.y, pos.z]
        obs = np.concatenate((curr_joints, curr_ee_pose))
        return np.concatenate((obs, current_object_pos,))

    def step(self, action, complete_action=False, action_duration=0.05, check_collisions=True):
        """
            Robot will execute given action. returns: new observation, reward, done, and episode elapsed time
            
            Parameters:
                - if "complete_action" is true, the robot will preform the entire action specified,
                  stop, and then return the obs. Else, it will store the action in a buffer of actions
                  to execute and return an observation after "action_duration" seconds (allows for 
                  smoother movement but more noise in observations).
                - if "check_collisions" is true, the robot will only take actions that dont result in 
                  a collison.
        """

        if not np.shape(action)[0] == self.n_actions:
            raise ValueError("Action shpae dimensionality mismatch: recieved %x, need %s" % (np.shape(action)[0], self.n_actions))
        if np.isnan(action).any():
            print("NaN DETECTEED")
            action = np.zeros_like(action)

        #TODO: It is not exactly clear if the way the max action is being used here is correct
        action = np.minimum(np.array(action), self.max_action_true)
        action = [action[0], action[1],0,0,0,0]

        if complete_action:
            go_to_relative(action, collision_check=check_collisions, complete_action=True)
        else:
            go_to_relative(action, collision_check=check_collisions, complete_action=False)
            rospy.sleep(action_duration)
        
        obs = self.get_obs()
        eep_xyz_pose = obs[7:10]
        obj_pose = obs[13:]
        print(eep_xyz_pose, obj_pose)
        obj_distance = np.linalg.norm(eep_xyz_pose-obj_pose)
        done = (obj_distance < self.success_threshold) or (time.time() - self.cur_time > self.episode_time)

        if self.sparse_rewards:
            if obj_distance < self.success_threshold:
                reward = 0
            else: 
                reward = -1
        else:
            reward = obj_distance
        
        return obs, reward, done, (time.time()-self.cur_time)

    def reset(self):
        go_to_start(self.arm, self.reset_pose, start=False)
        self.cur_time = time.time()
        obs = self.get_obs()
        if self.with_pixels:
            # TODO
            return obs
        else:
            return obs