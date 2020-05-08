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
        rospy.Subscriber("cropped", Image, self.Combine_and_publish)
        rospy.Subscriber("white_line", Image, self.Lines_white)
        rospy.Subscriber("yellow_line", Image, self.Lines_yellow)
        self.pubw = rospy.Publisher("Line_white", Image, queue_size=10)
        self.puby = rospy.Publisher("Line_yellow", Image, queue_size=10)
        
        self.img1 = None
        self.img2 = None
        self.img3 = None
        self.cb1 = 0 
        self.cb2 = 0
   
    def Combine_and_publish(self,msg):
        
        self.img3 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        return self.img3

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
        
        edges = cv2.Canny(hsv_yellow, 0, 300, apertureSize=3)

        self.img1 = self.output_lines(cv_img, edges)
      
        self.cb1 = 1


    def Lines_white(self,msg):

        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        hsv_white = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
       
        edges = cv2.Canny(hsv_white, 0, 300, apertureSize=3)

        self.img2 = self.output_lines(cv_img, edges)

        self.cb2 = 1
        
      
if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("edge_detect", anonymous=True)
    img_flip = ExtractLines()
    
    while img_flip.cb1 == 1 and img_flip.cb2 == 1: 

        img = cv2.bitwise_and(img_flip.img1, img_flip.img2, mask=None)
        image_final = img_flip.bridge.cv2_to_imgmsg(img, "bgr8")
        img_flip.puby.publish(image_final)
        self.cb1 = 0
        self.cb2 = 0
        break

        #self.pubw.publish(img_flip.cv_image2)
    rospy.spin()
