#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ExtractLines:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("cropped", Image, self.Lines_in_bw_cb)
        self.pubw = rospy.Publisher("image_linesw", Image, queue_size=10)

    def output_lines(self, original_image, lines):

        r_res = 1
        theta_res = np.pi/180
        thresh = 1
        min_length = 0
        max_gap = 20
        lines = cv2.HoughLinesP(lines, r_res, theta_res, thresh, np.empty(1), min_length, max_gap)
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output


    def Lines_in_bw_cb(self, msg):
        
        # convert to a ROS image using the bridge        
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        #convert to HSV from bgr
        img_hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        
        # extract whites in the image
        lower = np.array([0, 0, 220], dtype="uint8")
        upper = np.array([255, 30, 255], dtype="uint8")
        white_mask = cv2.inRange(img_hsv, lower, upper)

        lower = np.array([30, 100, 120], dtype="uint8")
        upper = np.array([100, 255, 255], dtype="uint8")
        yellow_mask = cv2.inRange(img_hsv, lower, upper)


        # Converting them to white
        white_edges = cv2.bitwise_and(img_hsv, img_hsv, mask=white_mask)
        white_edges[white_mask>0]=(0,0,255)       

        yellow_edges = cv2.bitwise_and(img_hsv, img_hsv, mask=yellow_mask)
        yellow_edges[yellow_mask>0]=(0,0,255) 

        white_yellow = cv2.bitwise_or(yellow_edges, white_edges, mask=None)
        edges = cv2.Canny(white_yellow, 0, 300, apertureSize=3)
        final_image = self.output_lines(img_hsv, edges) 
        
        # Convert HSV to BGR
        final = cv2.cvtColor(final_image, cv2.COLOR_HSV2BGR)

        # convert new image to ROS to send 
        Lines = self.bridge.cv2_to_imgmsg(final, "bgr8")
        
        # publish the new image
        self.pubw.publish(Lines)


      
if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("lines", anonymous=True)
    img_flip = ExtractLines()
    rospy.spin()
