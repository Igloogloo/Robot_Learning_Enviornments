#!/usr/bin/env python
#Author: James Staley

import roslib
import rospy
import sys
import time
import tf
import math
import numpy as np
import hlpr_manipulation_utils.transformations as tft

from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from hlpr_manipulation_utils.manipulator import Gripper
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Point, Quaternion

world_frame = "base_link"
depth_frame = "camera_depth_frame"
camera_frame = "camera_color_optical_frame"
arm_frame = "j2s7s300_link_base"

class PokePoint():
    def __init__(self):
        self.blob_pub = rospy.Subscriber("/aabl/poi", PoseStamped, self.pointCB, queue_size=1)
        self.moveit = ArmMoveIt("j2s7s300_link_base")
        self.gripper = Gripper()

        self.tf_listener = tf.TransformListener()
        self.object_tf_pub_src = tf.TransformBroadcaster()
        self.object_tf_pub_target = tf.TransformBroadcaster()
        self.vision_fudge_factor=(0, -.05, 0) # TODO: this should be based on the percieved depth of the object
        self.approach_height=0.07
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.lift_height = 0.15
        self.place_height = 0.01

    def pointCB(self, msg): # TODO: we know this is in camera frame but it should come in as PointStamped
        print("got point")
        p = msg.pose.position
        goal_camera = PoseStamped()
        # p = point
        q = Quaternion()
        q.x = 0 #rot[0]
        q.y = 0 #rot[1]
        q.z = 0 #rot[2]
        q.w = 1. #rot[3]
        goal_camera.pose.position = p
        goal_camera.pose.orientation = q
        goal_camera.header.frame_id = camera_frame
        
        goal_arm = self.tf_listener.transformPose(arm_frame, goal_camera)
        # plan = self.moveit.plan_ee_pos(goal_arm)
        # self.moveit.move_through_waypoints(plan)

        goal_arm.pose.orientation = q

        print("GETTING IK")
        
        print("FOUND IK", self.moveit.get_IK(new_pose=goal_arm))
        #print("Goal in base-frame ", goal_arm)
        self.pick([goal_arm.pose.position.x, goal_arm.pose.position.y, goal_arm.pose.position.z])
        #self.moveit.move_to_ee_pose(goal_arm.pose)

        pt_arm = [goal_arm.pose.position.x, goal_arm.pose.position.y, goal_arm.pose.position.z]

        self.object_tf_pub_src.sendTransform(
            pt_arm,
            (0,0,0,1),
            rospy.Time.now(), "obj_world", world_frame)

        
        self.object_tf_pub_src.sendTransform(
            (p.x, p.y, p.z),
            (0,0,0,1),
            rospy.Time.now(), "obj_camera", camera_frame)

    def pick(self, target_position):

        target_position = np.array(target_position) + np.array(self.vision_fudge_factor)

        # open gripper
        self.gripper.open(block=True)

        # move over frame
        print("approaching frame")
        target = Pose()
        target.position.x = target_position[0]
        target.position.y = target_position[1]
        target.position.z = target_position[2] + self.approach_height
        target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = \
            tft.quaternion_from_euler(np.pi/2, 0.0, 0.0)

        self.publish_pose_as_tf(target)

        # Go to position above object
        if not self.moveit.move_to_ee_pose(target):
            print("Failed to move to position above object. Exiting..")
            return

        # Drop down on to object
        print("moving to frame")
        target.position.z = target_position[2]
        self.publish_pose_as_tf(target)

        if not self.moveit.move_to_ee_pose(target):
            print("Failed to drop into position around object. Exiting..")
            return

        # close gripper
        print("closing gripper")
        self.gripper.close(block=True)

        # move up
        print("moving up")
        target.position.z = target_position[2] + self.lift_height
        self.publish_pose_as_tf(target)
        if not self.moveit.move_to_ee_pose(target):
            print("Failed to lift into position with object. Exiting..")
            return

        self.is_holding = True
        self.next_place_height = target_position[2] + self.place_height
        print("done picking")

    def publish_pose_as_tf(self, target):
            self.broadcaster.sendTransform((target.position.x, target.position.y, target.position.z),
                (target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w),
                rospy.Time(0),
                "target",
                arm_frame)
   


if __name__== '__main__':
    rospy.init_node("poke_point", anonymous=False)
    rospy.loginfo("Starting the aabl poke point node")
    lookat = PokePoint()
    rospy.spin()