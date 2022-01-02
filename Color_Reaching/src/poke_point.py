#!/usr/bin/env python
#Author: James Staley

import roslib
import rospy
import sys
import time
import tf
import math
import numpy as np

from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Point, Quaternion

world_frame = "base_link"
depth_frame = "camera_depth_frame"
camera_frame = "camera_color_optical_frame"
arm_frame = "j2s7s300_link_base"

class PokePoint():
    def __init__(self):
        self.blob_pub = rospy.Subscriber("/aabl/poi", PoseStamped, self.pointCB, queue_size=1)
        self.moveit = ArmMoveIt("j2s7s300_link_base")

        self.tf_listener = tf.TransformListener()
        self.object_tf_pub_src = tf.TransformBroadcaster()
        self.object_tf_pub_target = tf.TransformBroadcaster()
    
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
        
        goal_world = self.tf_listener.transformPose(arm_frame, goal_camera)
        # plan = self.moveit.plan_ee_pos(goal_arm)
        # self.moveit.move_through_waypoints(plan)

        goal_world.pose.orientation = q
        
        print("IKKKKK", self.moveit.get_IK(new_pose=goal_world))
        print("Goal in base-frame ", goal_world)
        self.moveit.move_to_ee_pose(goal_world.pose)

        pt_arm = [goal_world.pose.position.x, goal_world.pose.position.y, goal_world.pose.position.z]

        self.object_tf_pub_src.sendTransform(
            pt_arm,
            (0,0,0,1),
            rospy.Time.now(), "obj_world", world_frame)

        
        self.object_tf_pub_src.sendTransform(
            (p.x, p.y, p.z),
            (0,0,0,1),
            rospy.Time.now(), "obj_camera", camera_frame)


if __name__== '__main__':
    rospy.init_node("poke_point", anonymous=False)
    rospy.loginfo("Starting the aabl poke point node")
    lookat = PokePoint()
    rospy.spin()