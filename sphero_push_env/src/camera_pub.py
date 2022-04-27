#!/usr/bin/env python
import rospy 
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError 

def main():
    rospy.init_node("logi_camera")
    pub = rospy.Publisher('logi_camera/frame', Image, queue_size=10)
    video = cv2.VideoCapture(-1) # for using CAM
    # Exit if video not opened.
    if not video.isOpened():
        print("Could not open video")
        return 

    bridge = CvBridge()

    ok, frame = video.read()

    height = len(frame)
    width = len(frame[0])

    image_message = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
    print(image_message.header)
    while not rospy.is_shutdown(): # do work
        pub.publish(image_message)
        ok, frame = video.read()
        if not ok:
            print('Couldnt read frame')
            continue
        image_message = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        #print(image_message.height, image_message.width)
        #if cv2.waitKey(10) & 0xFF == ord("q"): 
        #    break
        rospy.sleep(.01)
        #key = cv2.waitKey(1000)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass