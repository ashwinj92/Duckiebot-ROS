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
        rospy.Subscriber("cropped", Image, self.Draw_Lines)
        self.pubw = rospy.Publisher("white_line", Image, queue_size=10)
        self.puby = rospy.Publisher("yellow_line", Image, queue_size=10)
        self.pubwy = rospy.Publisher("yellow_white_overlap", Image, queue_size=10)
    
    def Lines_in_bw_cb(self, msg):
        
        # convert to a ROS image using the bridge        
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        #convert to HSV from bgr
        img_hsv = cv.cvtColor(cv_img, cv.COLOR_BGR2HSV)
        
        # extract whites in the image
        lower = np.array([0, 0, 220], dtype="uint8")
        upper = np.array([255, 30, 255], dtype="uint8")
        white_mask = cv.inRange(img_hsv, lower, upper)
 
        # extract yellows in the image
        lower = np.array([30, 100, 120], dtype="uint8")
        upper = np.array([100, 255, 255], dtype="uint8")
        yellow_mask = cv.inRange(img_hsv, lower, upper)

        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (3,3))
        white_dilate = cv.dilate(white_mask, kernel)
        yellow_dilate = cv.dilate(yellow_mask, kernel)

        edges = cv.Canny(img_hsv, 0, 300, apertureSize=3)

        white_edges = cv.bitwise_and(img_hsv, img_hsv, mask=white_dilate)
        yellow_edges = cv.bitwise_and(img_hsv, img_hsv, mask=yellow_dilate)


        #white_edges1 = cv.bitwise_and(white_edges, white_edges, mask=edges)
        #yellow_edges1 = cv.bitwise_and(yellow_edges, yellow_edges, mask=edges)
        
        # Overlap both masks into a single mask
        mask = cv.bitwise_or(white_mask, yellow_mask)
        
        # And it to the original image
        result = cv.bitwise_and(img_hsv, img_hsv, mask=mask)
        
        rgbimg_yellow = cv.cvtColor(white_edges, cv.COLOR_HSV2BGR)
        rgbimg_white = cv.cvtColor(yellow_edges, cv.COLOR_HSV2BGR)
        rgbimg = cv.cvtColor(result, cv.COLOR_HSV2BGR)

        # convert new image to ROS to send 
        whiteLines = self.bridge.cv2_to_imgmsg(rgbimg_yellow, "bgr8")
        yellowLines = self.bridge.cv2_to_imgmsg(rgbimg_white, "bgr8")
        ros_cropped = self.bridge.cv2_to_imgmsg(rgbimg, "bgr8")
        
        # publish the new image
        self.pubw.publish(whiteLines)
        self.puby.publish(yellowLines)
        self.pubwy.publish(ros_cropped)


    def Draw_Lines(self, msg):
        
        # convert to a ROS image using the bridge        
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        #convert to HSV from bgr
        img_hsv = cv.cvtColor(cv_img, cv.COLOR_BGR2HSV)
        
        # extract whites in the image
        lower = np.array([0, 0, 220], dtype="uint8")
        upper = np.array([255, 30, 255], dtype="uint8")
        white_mask = cv.inRange(img_hsv, lower, upper)
 
        # extract yellows in the image
        lower = np.array([30, 100, 120], dtype="uint8")
        upper = np.array([100, 255, 255], dtype="uint8")
        yellow_mask = cv.inRange(img_hsv, lower, upper)

        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (3,3))
        white_dilate = cv.dilate(white_mask, kernel)
        yellow_dilate = cv.dilate(yellow_mask, kernel)

        edges = cv.Canny(img_hsv, 0, 300, apertureSize=3)

        white_edges = cv.bitwise_and(white_dilate, edges)
        yellow_edges = cv.bitwise_and(yellow_dilate, edges)

        white_output = self.get_lines(img_hsv, white_edges)
        yellow_output = self.get_lines(img_hsv, yellow_edges)
        #white_edges1 = cv.bitwise_and(white_edges, white_edges, mask=edges)
        #yellow_edges1 = cv.bitwise_and(yellow_edges, yellow_edges, mask=edges)
        
        # Overlap both masks into a single mask
        mask = cv.bitwise_or(white_mask, yellow_mask)
        
        # And it to the original image
        result = cv.bitwise_and(img_hsv, img_hsv, mask=mask)
        
        #rgbimg_yellow = cv.cvtColor(white_edges, cv.COLOR_HSV2BGR)
        #rgbimg_white = cv.cvtColor(yellow_edges, cv.COLOR_HSV2BGR)
        rgbimg = cv.cvtColor(result, cv.COLOR_HSV2BGR)

        # convert new image to ROS to send 
        whiteLines = self.bridge.cv2_to_imgmsg(white_output, "bgr8")
        yellowLines = self.bridge.cv2_to_imgmsg(yellow_output, "bgr8")
        ros_cropped = self.bridge.cv2_to_imgmsg(rgbimg, "bgr8")
        
        # publish the new image
        self.pubw.publish(whiteLines)
        self.puby.publish(yellowLines)
        self.pubwy.publish(ros_cropped)

        


    def get_lines(self,original_image, filtered_image):
        r_res = 1
        theta_res = np.pi/180
        thresh = 1
        min_length = 0
        max_gap = 20
        lines = cv.HoughLinesP(filtered_image, r_res, theta_res, thresh, np.empty(1), min_length, max_gap)

        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                #print(lines[i])
                l = lines[i][0]
                cv.line(output, (l[0],l[1]), (l[2],l[3]), (0,0,255), 3, cv.LINE_AA)
        return output


if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("line_extract", anonymous=True)
    img_flip = ExtractLines()
    rospy.spin()
