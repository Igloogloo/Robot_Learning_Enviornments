#!/usr/bin/env python
#Author: James Staley

from logging import exception
import roslib
import rospy
import sys
import time
import tf
import math
import cv2
import numpy as np

import pyrealsense2 as rs2
from cv_bridge import CvBridge
from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from hlpr_lookat.srv import LookAt, LookAtT, LookAtTS
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Point, Quaternion
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

from hlpr_lookat.pantilt import PanTilt

lower = (130,10,10)
upper = (255,50,50)

world_frame = "base_link"
depth_frame = "camera_depth_frame"
camera_frame = "camera_color_optical_frame"
arm_frame = "j2s7s300_link_base"

depth_image_topic = "/camera/depth/image_rect_raw"
depth_info_topic = "/camera/depth/camera_info"


hz = 1 / 10.
p_inc = 0.4
t_inc = p_inc / 2.

class LookAtColor():
    def __init__(self):
        rospy.Subscriber("/camera/color/image_raw", Image, self.colorImgCB, queue_size=1)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.alignedDImgCB, queue_size=1)
        rospy.Subscriber(depth_info_topic, CameraInfo, self.depthInfoCB, queue_size=1)

        self.object_tf_pub_src = tf.TransformBroadcaster()
        self.blob_pub = rospy.Publisher("/camera/color/blue_blobs", Image, queue_size=10)
        self.poi_pub = rospy.Publisher("/aabl/poi", PoseStamped, queue_size=10)

        self.tf_listener = tf.TransformListener()

        self.toggle = False
        self.prevX = -1.
        self.prevY = -1.
        self.prevZ = -1.

        self.t_img = time.time()
        self.t_Dimg = time.time()

        self.cvbridge = CvBridge()
        self.center = None

        self.intrinsics = None # depth camera intrinsics

        pan_limits = [-0.75, 0.75]
        tilt_limits = [-0.75, 0.01]
        self.ptu = PanTilt(pan_limits=pan_limits, tilt_limits=tilt_limits)

        self.t0 = time.time()

        self.moveit = ArmMoveIt()
        
    def colorImgCB(self, msg):
        if (time.time() - self.t_img < hz): return

        img = self.cvbridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        mask = cv2.inRange(img, lower, upper)
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if len(contours) == 0:
            rospy.loginfo("No colored contours found.")
            self.center = None
            self.blob_pub.publish(self.cvbridge.cv2_to_imgmsg(img, encoding="rgb8"))
        else:
            blob = max(contours, key=lambda el: cv2.contourArea(el))
            M = cv2.moments(blob)
            try:
                self.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            except Exception as e:
                rospy.logerr(e)
                rospy.loginfo("Couldn't calculate center from moment: {}".format(M))
                self.center = None
                self.t_img = time.time()
                return

            canvas = img.copy()
            cv2.circle(canvas, self.center, 50, (255,255,255), -1)

            self.blob_pub.publish(self.cvbridge.cv2_to_imgmsg(canvas, encoding="rgb8"))

        self.t_img = time.time()

    def alignedDImgCB(self, msg):
        if (time.time() - self.t_Dimg < hz): return
        if (self.center is None): return

        img = self.cvbridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        depth = img[self.center[1], self.center[0]] # swapped 
        # rospy.loginfo("aligned depth image received. Looking for center {}. Center: {}.".format(self.center, depth))

        # we now have (1) the center of the blob in pixel coords and (2) the depth of the blob. User the camera intrinsics to grab the 3D point associated with it.
        if (self.intrinsics):
            point = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [self.center[0], self.center[1]], depth)
            point = [entry / 1000. for entry in point] # get in meters

            pan_increment = 0.0
            tilt_increment = 0.0
            pan_centered = False
            tilt_centered = False

            w = 0.
            h = 0.
            thresh_w = 0.1
            thresh_h = 0.1 

            # rospy.loginfo("{}".format(point))
            if (point[0] > (w + thresh_w)):
                pan_increment = -p_inc
            elif (point[0] < (w - thresh_w)):
                pan_increment = p_inc
            else:
                pan_centered = True

            if (point[1] > (h + thresh_h)):
                tilt_increment = t_inc
            elif (point[1] < (h - thresh_h)):
                tilt_increment = -t_inc
            else:
                tilt_centered = True

            self.ptu.increment_pantilt([pan_increment, tilt_increment])

            # ISS: changed from if to while
            while (pan_centered and tilt_centered):
                depth_array = np.array(img, dtype=np.float32)
                center_idx = np.array(depth_array.shape) / 2
                center_depth = depth_array[center_idx[0], center_idx[1]] / 1000.
                if (center_depth < 0.1): # skip, we got an empty frame
                    pass
                else:
                    # p = Point(0., 0., center_depth)
                    p = Point(point[0], point[1], point[2])
                    poi = PoseStamped()
                    poi.pose.position = p
                    poi.pose.orientation = Quaternion(0.,0.,0.,1.)
                    poi.header.frame_id = camera_frame

                    self.object_tf_pub_src.sendTransform(
                        (p.x, p.y, p.z),
                        (0,0,0,1),
                        rospy.Time.now(), "lookat_camera", camera_frame)
                    self.poi_pub.publish(poi)
                    rospy.sleep(.05)

        self.t_Dimg = time.time()

    def depthInfoCB(self, camerainfo):
        if self.intrinsics:
            pass
        else:
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = camerainfo.width
            self.intrinsics.height = camerainfo.height
            self.intrinsics.ppx = camerainfo.K[2]
            self.intrinsics.ppy = camerainfo.K[5]
            self.intrinsics.fx = camerainfo.K[0]
            self.intrinsics.fy = camerainfo.K[4]
            if (camerainfo.distortion_model == "plumb_bob"):
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif (camerainfo.distortion_model == "equidistant"):
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in camerainfo.D]

if __name__== '__main__':
    rospy.init_node("aabl_lookat_color", anonymous=False)
    rospy.loginfo("Starting the aabl lookatcolor node")
    lookat = LookAtColor()
    rospy.spin()