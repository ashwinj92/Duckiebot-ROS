#!/usr/bin/env python

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageCropper:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("image", Image, self.cropper_cb)
        self.pub = rospy.Publisher("cropped", Image, queue_size=10)
    
    def cropper_cb(self, msg):
        # convert to a ROS image using the bridge
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # crop the top half
        height = cv_img.shape[0]
        width = cv_img.shape[1]
        crop_img = cv_img[height/2:height,0:width]
        
        # convert new image to ROS to send
        ros_cropped = self.bridge.cv2_to_imgmsg(crop_img, "bgr8")
        
        # publish flipped image
        self.pub.publish(ros_cropped)
        

if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("image_cropper", anonymous=True)
    img_flip = ImageCropper()
    rospy.spin()
