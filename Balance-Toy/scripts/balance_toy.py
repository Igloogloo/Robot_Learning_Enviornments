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
from measure_width_utils import get_width_image, get_width
from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from hlpr_manipulation_utils.manipulator import Gripper
import hlpr_manipulation_utils.transformations as Transform
from geometry_msgs.msg import Pose

PIXELS_PER_METRIC = 100
cur_pos = np.array([0,0,0])

def midpoint(ptA, ptB):
	return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)

def go_to_start(arm, pose=None):
    os.system('rosservice call /j2s7s300_driver/in/home_arm ')
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
    go_to_relative([0,0,0,0,0,90])

class Camera():
    def __init__(self, stack_size=4):
        self.vid = cv2.VideoCapture(0)
        self.stack_size = stack_size
        self.img_queue = []
    def start(self):
        #vid = cv2.VideoCapture(0)
        #while True:
        _, image = self.vid.read()
        if len(self.img_queue) < self.stack_size:
            self.img_queue.append(image)
        else:
            self.img_queue = self.img_queue[1:]
            self.img_queue.append(image)
        yield image 
    
    def stop(self):
        self.vid.release()
        # Destroy all the windows
        cv2.destroyAllWindows()
    
    def get_image(self):
        _, image = self.vid.read()
        return image
    
    def get_image_stack(self):
        images = []
        for i in range(self.stack_size):
            _, image = self.vid.read()
            images.append(image)
        self.img_queue = images
        vert_stack = np.array(images)
        return vert_stack
    
    def get_observation(self):
        if not np.shape(self.img_queue)[0] == self.stack_size:
            return self.get_image_stack()
        else:
            _, image = self.vid.read()
            self.img_queue = self.img_queue[1:]
            self.img_queue.append(image)
            return np.array(self.img_queue)

class BalanceToy():
    def __init__(self, with_pixels=True, max_action=10, n_actions=3, reset_pose=None, episode_time=60, stack_size=4):
        """
        with_pixels = True to learn from overhead camera
        max_action: the maximum degree the robot can rotate in xyz cartesian space
        n_actions: xyz cartesian rotations (not advised to change)
        reset_pose: cartesian pose and orientation for the robot to start in
        episode_time: maximum time alloted for each episode
        """
        rospy.init_node("ball_toy", disable_signals=True)
        self.arm = ArmMoveIt("j2s7s300_link_base")
        self.grip = Gripper()
        self.with_pixels = with_pixels
        self.max_action = max_action     
        self.reset_pose = reset_pose   
        self.stack_size = stack_size
        self.episode_time = episode_time
        self.camera = Camera(stack_size=self.stack_size)
        self.image_gen = self.camera.start()
        #self.camera.start()
        self.n_actions = n_actions
        if with_pixels:
            image = next(self.image_gen)
            self.observation_size = image.shape
        else:
            # TODO: impliment non-pixel based version
            self.observation_size = 8

        self.cur_time = time.time()
        self.total_time = time.time()

        go_to_start(self.arm, self.reset_pose)

        self.grip.open()
        print("Please place toy in upright position in gripper. Gripper will close in 5 seconds. WATCH FINGERS")
        time.sleep(5)
        self.grip.close(block=False)

    def render(self, pixels_only=False, show_width=True):
        """
        Renders enviornment.

        pixels_only=True if do not want image to be show.
        show_width=True to show width boundries and values on balance toy.
        """
        frame = self.camera.get_image()
        if show_width:
            image = get_width_image(frame, PIXELS_PER_METRIC)
        else: 
            image = frame
        if image is None:
            cv2.imshow("env render", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyWindow("env render")
            return

        if pixels_only:
            return image
        
        cv2.imshow("env render", image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyWindow("env render")

    def step(self, action):
        """
        Take step in enviornment.

        Returns:
            observation, reward, done, total time in enviornment
        """
        if not np.shape(action) == self.n_actions:
            raise ValueError("Action shpae dimensionality mismatch: recieved %x, need %s" % (action.shape, self.n_actions))
        
        action = np.array(action)
        global cur_pos
        cur_pos += action
        for i in range(len(cur_pos)):
            if cur_pos[i] < -90:
                cur_pos[i] = -90
                action[i] = -90
            if cur_pos[i] > 90:
                cur_pos[i] = 90
                action[i] = 90

        action = [0,0,0, action[0], action[1], action[2]]

        if time.time() - self.cur_time <= self.episode_time:
            go_to_relative(action)
            observation = self.camera.get_image_stack()
            return observation, get_width(self.camera.get_image(), PIXELS_PER_METRIC), False, self.total_time.duration
        else:
            go_to_relative(action)
            observation = self.camera.get_image_stack()
            self.reset()
            return observation, get_width(self.camera.get_image(), PIXELS_PER_METRIC), True, self.total_time.duration

    def reset(self):
        go_to_start(self.arm, self.reset_pose)
        self.cur_time = time.time()
        return self.camera.get_image_stack()
