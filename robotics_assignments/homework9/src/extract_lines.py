#!/usr/bin/env python

import sys
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ExtractLines:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("cropped", Image, self.Lines_in_bw_cb)
        self.pubw = rospy.Publisher("white_line", Image, queue_size=10)
        self.puby = rospy.Publisher("yellow_line", Image, queue_size=10)
    
    def Lines_in_bw_cb(self, msg):
        
        # convert to a ROS image using the bridge        
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        #convert to HSV from bgr
        img_hsv = cv.cvtColor(cv_img, cv.COLOR_BGR2HSV)
        
        # extract whites in the image
        lower = np.array([0, 0, 220], dtype="uint8")
        upper = np.array([255, 30, 255], dtype="uint8")
        white_mask = cv.inRange(img_hsv, lower, upper)

        lower = np.array([30, 100, 120], dtype="uint8")
        upper = np.array([100, 255, 255], dtype="uint8")
        yellow_mask = cv.inRange(img_hsv, lower, upper)

        # Converting them to white
        white_edges = cv.bitwise_and(img_hsv, img_hsv, mask=white_mask)
        white_edges[white_mask>0]=(0,0,255)       
        yellow_edges = cv.bitwise_and(img_hsv, img_hsv, mask=yellow_mask)
        yellow_edges[yellow_mask>0]=(0,0,255)       
        
        # Convert HSV to BGR
        rgbimg_yellow = cv.cvtColor(img_hsv, cv.COLOR_HSV2BGR)
        rgbimg_white = cv.cvtColor(img_hsv, cv.COLOR_HSV2BGR)

        rgbimg_yellow = cv.cvtColor(yellow_edges, cv.COLOR_HSV2BGR)
        rgbimg_white = cv.cvtColor(white_edges, cv.COLOR_HSV2BGR)

        # convert new image to ROS to send 
        whiteLines = self.bridge.cv2_to_imgmsg(rgbimg_white, "bgr8")
        yellowLines = self.bridge.cv2_to_imgmsg(rgbimg_yellow, "bgr8")
        
        # publish the new image
        self.pubw.publish(whiteLines)
        self.puby.publish(yellowLines)
      
if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("line_extract", anonymous=True)
    img_flip = ExtractLines()
    rospy.spin()
