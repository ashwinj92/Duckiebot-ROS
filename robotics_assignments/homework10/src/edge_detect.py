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
        #rospy.Subscriber("white_line", Image, self.Lines_in_bw_cb)
        #rospy.Subscriber("yellow_line", Image, self.Lines_in_bw_cb)
        #rospy.Subscriber("cropped", Image, self.Combine_and_publish)
        rospy.Subscriber("white_line", Image, self.Lines_white)
        rospy.Subscriber("yellow_line", Image, self.Lines_yellow)
        self.pubw = rospy.Publisher("Line_white", Image, queue_size=10)
        self.puby = rospy.Publisher("Line_yellow", Image, queue_size=10)
   
    def Combine_and_publish(self,msg):
        
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")



    def get_lines(self,original_image, filtered_image):
        r_res = 1
        theta_res = np.pi/180
        thresh = 1
        min_length = 0
        max_gap = 20
        lines = cv2.HoughLinesP(filtered_image, r_res, theta_res, thresh, np.empty(1), min_length, max_gap)

        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                #print(lines[i])
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]),(255,0,0), 3, cv2.LINE_AA)
        return output

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

    def Lines_yellow(self,msg):

        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        hsv_yellow = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        
        edges = cv2.Canny(hsv_yellow, 0, 500, apertureSize=3)

        yellow_output = self.output_lines(hsv_yellow, edges)

        cv_image = self.bridge.cv2_to_imgmsg(yellow_output, "bgr8")
        
        self.puby.publish(cv_image)

    def Lines_white(self,msg):

        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        hsv_white = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
       
        edges = cv2.Canny(hsv_white, 0, 300, apertureSize=3)

        white_output = self.output_lines(cv_img, edges)

        cv_image = self.bridge.cv2_to_imgmsg(white_output, "bgr8")
        
        self.pubw.publish(cv_image)


      
if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("edge_detect", anonymous=True)
    img_flip = ExtractLines()
    rospy.spin()
