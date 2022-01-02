from realsensecv import RealsenseCapture

import cv2
import numpy as np
from realsensecv import RealsenseCapture
from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import argparse
import imutils

cap = RealsenseCapture()
 # Property setting
cap.WIDTH = 640
cap.HEIGHT = 480
cap.FPS = 30
# Unlike cv2.VideoCapture (), do not forget cap.start ()
cap.start()
  
def midpoint(ptA, ptB):
	return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)

while True:

    ret, frames = cap.read()  # RGB to frames [0], frames [ 1] in the image that contains the ndarray of Depth
    color_frame = frames[0]
  
    # Display the resulting frame
    #cv2.imshow('frame', frame)
    image = color_frame
    hsv = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([60,160,100])
    upper_blue = np.array([130,255,255])
 
    # Here we are defining range of bluecolor in HSV
    # This creates a mask of blue coloured
    # objects found in the frame.
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    #cv2.imshow('mask',mask)
    res = cv2.bitwise_and(color_frame,color_frame, mask= mask)
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)
    #res = cv2.bitwise_and(frame,frame, mask= mask)
    #gray = res
    #cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)

    edged = cv2.Canny(gray, 50, 100)
    edged = cv2.dilate(edged, None, iterations=1)
    edged = cv2.erode(edged, None, iterations=1)
    # find contours in the edge map
    cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    # sort the contours from left-to-right and initialize the
    # 'pixels per metric' calibration variable
    while len(cnts) == 0:
  
        # Display the resulting frame
        #cv2.imshow('frame', frame)
        #image = frame
        #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([60,110,100])
        upper_blue = np.array([130,255,255])
    
        # Here we are defining range of bluecolor in HSV
        # This creates a mask of blue coloured
        # objects found in the frame.
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        #cv2.imshow('mask',mask)
        res = cv2.bitwise_and(color_frame,color_frame, mask= mask)
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)
        #res = cv2.bitwise_and(frame,frame, mask= mask)
        #gray = res
        #cv2.imshow('frame',frame)
        #cv2.imshow('mask',mask)
        #cv2.imshow('res',res)

        # perform edge detection, then perform a dilation + erosion to
        # close gaps in between object edges
        edged = cv2.Canny(gray, 50, 100)
        edged = cv2.dilate(edged, None, iterations=1)
        edged = cv2.erode(edged, None, iterations=1)
        # find contours in the edge map
        cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
    (cnts, _) = contours.sort_contours(cnts)

    # loop over the contours individually
    for c in cnts:
        # if the contour is not sufficiently large, ignore it
        if cv2.contourArea(c) < 1650:
            continue
        # compute the rotated bounding box of the contour
        orig = image.copy()
        box = cv2.minAreaRect(c)
        box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
        box = np.array(box, dtype="int")
        # order the points in the contour such that they appear
        # in top-left, top-right, bottom-right, and bottom-left
        # order, then draw the outline of the rotated bounding
        # box
        box = perspective.order_points(box)
        cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 2)
        # loop over the original points and draw them
        for (x, y) in box:
            cv2.circle(orig, (int(x), int(y)), 5, (0, 0, 255), -1)
      
    	# unpack the ordered bounding box, then compute the midpoint
        # between the top-left and top-right coordinates, followed by
        # the midpoint between bottom-left and bottom-right coordinates
        (tl, tr, br, bl) = box
        (tltrX, tltrY) = midpoint(tl, tr)
        (blbrX, blbrY) = midpoint(bl, br)
        # compute the midpoint between the top-left and top-right points,
        # followed by the midpoint between the top-righ and bottom-right
        (tlblX, tlblY) = midpoint(tl, bl)
        (trbrX, trbrY) = midpoint(tr, br)
        # draw the midpoints on the image
        cv2.circle(orig, (int(tltrX), int(tltrY)), 5, (255, 0, 0), -1)
        cv2.circle(orig, (int(blbrX), int(blbrY)), 5, (255, 0, 0), -1)
        cv2.circle(orig, (int(tlblX), int(tlblY)), 5, (255, 0, 0), -1)
        cv2.circle(orig, (int(trbrX), int(trbrY)), 5, (255, 0, 0), -1)
        # draw lines between the midpoints
        cv2.line(orig, (int(tltrX), int(tltrY)), (int(blbrX), int(blbrY)),
            (255, 0, 255), 2)
        cv2.line(orig, (int(tlblX), int(tlblY)), (int(trbrX), int(trbrY)),
            (255, 0, 255), 2)
        # the 'q' button is set as the
        # quitting button you may use any
        # desired button of your choice
        	# compute the Euclidean distance between the midpoints
        dA = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
        dB = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
        #print(dA, dB)
        # if the pixels per metric has not been initialized, then
        # compute it as the ratio of pixels to supplied metric
        # (in this case, inches)
        #if pixelsPerMetric is None:
            #pixelsPerMetric = dB / args["width"]
        	# compute the size of the object
        dimA = dA / 100
        dimB = dB / 100
        # draw the object sizes on the image
        cv2.putText(orig, "{:.1f}in".format(dimA),
            (int(tltrX - 15), int(tltrY - 10)), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, (255, 255, 255), 2)
        cv2.putText(orig, "{:.1f}in".format(dimB),
            (int(trbrX + 10), int(trbrY)), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, (255, 255, 255), 2)
        # show the output image
        cv2.imshow("Image", orig)

    depth_frame = frames[1]

    # in the heat map conversion
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(

    depth_frame, alpha=0.08), cv2.COLORMAP_JET)

    # rendering
    images = np.hstack((color_frame, depth_colormap))  # display side by side RGB and Depth next to
    cv2.imshow('RealSense', images)


    if cv2.waitKey(1) & 0xFF == ord('q'):

        break


# Streaming Stop
cap.release()
cv2.destroyAllWindows()
 