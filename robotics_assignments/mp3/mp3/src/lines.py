#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class ExtractLines:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
       # rospy.Subscriber("cropped", Image, self.Lines_in_bw_cb)
        self.pubw = rospy.Publisher("image_linesw", Image, queue_size=1)
        rospy.Subscriber("ashwinsduckie/anti_instagram_node/corrected_image/compressed",CompressedImage, self.Lines_in_bw_cb, queue_size=1, buff_size=2**24)

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
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        #convert to HSV from bgr
        crop = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        
        height = crop.shape[0]
        width = crop.shape[1]
        img_hsv = crop[height/2:height,0:width]

        # extract whites in the image
        lower = np.array([0, 0, 180], dtype="uint8")
        upper = np.array([115, 60, 255], dtype="uint8")
        white_mask = cv2.inRange(img_hsv, lower, upper)

        lower = np.array([20, 100,110], dtype="uint8")
        upper = np.array([30, 255, 255], dtype="uint8")
        yellow_mask = cv2.inRange(img_hsv, lower, upper)


        # Converting them to white
        white_edges = cv2.bitwise_and(img_hsv, img_hsv, mask=white_mask)
        white_edges[white_mask>0]=(0,0,255)       
        edgesw = cv2.Canny(white_edges,0,300,apertureSize=3) 

  
        yellow_edges = cv2.bitwise_and(img_hsv, img_hsv, mask=yellow_mask)
        yellow_edges[yellow_mask>0]=(0,0,255) 
        edgesy = cv2.Canny(yellow_edges,0,300, apertureSize=3)
         
        white_yellow = cv2.bitwise_or(edgesy, edgesw, mask=None)
       
        #edges = cv2.Canny(white_yellow, 0, 300, apertureSize=3)
        final_image = self.output_lines(img_hsv, white_yellow) 
        
        # Convert HSV to BGR
        final = cv2.cvtColor(final_image, cv2.COLOR_HSV2BGR)

          # convert new image to ROS to send 
        Lines = self.bridge.cv2_to_imgmsg(final, "bgr8")
  
        rospy.loginfo(rospy.get_caller_id() + "entered callback")      
        # publish the new image
        self.pubw.publish(Lines)

      
if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("lines", anonymous=True)
    img_flip = ExtractLines()
    rospy.spin()
