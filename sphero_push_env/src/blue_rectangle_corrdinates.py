#!/usr/bin/env python

from cgitb import grey
import imp
from random import random
import numpy as np
import cv2
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError 
from sphero_interface.srv import GetRandomPoint, GetRandomPointResponse

def get_random_points(required_data):

    bridge = CvBridge()

 
    image_message = rospy.wait_for_message("/logi_camera/frame", Image)
    frame = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
     
    # Define an initial bounding box
    bbox = (287, 183, 46, 42)
 
    # Uncomment the line below to select a different bounding box
    #bbox = cv2.selectROI(frame, False)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([60,110,100])
    upper_blue = np.array([130,255,255])

    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    #cv2.imshow('mask',mask)
    res = cv2.bitwise_and(frame,frame, mask= mask)
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    #gray = cv2.GaussianBlur(gray, (21, 21), 5)
    gray = cv2.blur(gray, (49, 49))
    #res=cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

    shape = "unidentified"
    """peri = cv2.arcLength(4, True)
    approx = cv2.approxPolyDP(4, 0.04 * peri, True)
    if len(approx) == 4:
        # compute the bounding box of the contour and use the
        # bounding box to compute the aspect ratio
        (x, y, w, h) = cv2.boundingRect(approx)
        ar = w / float(h)
        # a square will have an aspect ratio that is approximately
        # equal to one, otherwise, the shape is a rectangle
        shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"""
    #ret,thresh = cv2.threshold(gray,0,0,0)
    try:
        im2,contours = cv2.findContours(gray, 1, 2)
        bbox = cv2.boundingRect(im2[1])
        p1 = (int(bbox[0])+70, int(bbox[1])+70)
        p2 = (int(bbox[0] + bbox[2])-70, int(bbox[1] + bbox[3])-70)
        #cv2.rectangle(gray, p1, p2, (25,155,155), 2, 1)
        #cv2.circle(gray, (x_randCorr, y_randCorr), 20, (255,255, 255), thickness=-1)
        
        #cv2.imshow("RandomPoint", gray)
        #cv2.waitKey()
        
        x_randCorr = np.random.randint(low=p1[0], high=p2[0])
        y_randCorr = np.random.randint(low=p1[1], high=p2[1])
    except:
        x_randCorr = 1000
        y_randCorr = 650
        
    random_point = GetRandomPointResponse()
    random_point.x = int(x_randCorr)
    random_point.y = int(y_randCorr)


    

    return random_point



def random_point_server():
    rospy.init_node('random_point_server')
    s = rospy.Service('get_random_point', GetRandomPoint, get_random_points)
    print("Ready to give random point in blue rectangle.")
    rospy.spin()

if __name__ == "__main__":
    random_point_server()
