#!/usr/bin/env python
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class ImageProcess:
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_cropped=0
        self.original=0
        self.cropped_edge=0
        self.final_white=0
        rospy.Subscriber("/ashwinsduckie/anti_instagram_node/corrected_image/compressed", CompressedImage, self.lanefilter_cb, queue_size=1, buff_size=2**24)
        self.pub = rospy.Publisher("screen_reading", Image, queue_size=10)
    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output
    def lanefilter_cb(self, msg):
        # convert to a ROS image using the bridge
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        length=cv_img.shape[0]
        cv_cropped=cv_img[(length/2):length]
        self.cv_cropped = self.bridge.cv2_to_imgmsg(cv_cropped, "bgr8")
        #color filtering
        image_hsv=cv2.cvtColor(cv_cropped,cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(image_hsv,(0,0,0),(180,25,255))
        img_white1=cv2.bitwise_or(image_hsv,image_hsv,mask=mask)
        #mask_white=cv2.bitwise_or(image_hsv,
        mask2=cv2.inRange(image_hsv,(12,120,0),(36,255,255))
        img_yellow1=cv2.bitwise_or(image_hsv,image_hsv,mask=mask2) 
        #cv_cropped=cv_img[(length/2):length]
        cv_white2=cv2.cvtColor(img_white1,cv2.COLOR_HSV2BGR)
        cv_yellow2=cv2.cvtColor(img_yellow1,cv2.COLOR_HSV2BGR)
        cv_white=cv2.cvtColor(cv_white2,cv2.COLOR_BGR2GRAY)
        cv_yellow=cv2.cvtColor(cv_yellow2,cv2.COLOR_BGR2GRAY)
        kernel_d=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
        cv_yellow=cv2.erode(cv_yellow,kernel_d)
        cv_white=cv2.erode(cv_white,kernel_d)
        kernel_e=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7))
        cv_yellow=cv2.dilate(cv_yellow,kernel_e)
        cv_white=cv2.dilate(cv_white,kernel_e)
        #ros_white = self.bridge.cv2_to_imgmsg(cv_white, "mono8")
        #ros_yellow = self.bridge.cv2_to_imgmsg(cv_yellow, "mono8")
        # convert to a ROS image using the bridge
        cv_img = self.bridge.imgmsg_to_cv2(self.cv_cropped,"mono8")
        cv_img1 = self.bridge.imgmsg_to_cv2(self.cv_cropped,"bgr8")
        self.original=cv_img1
        blur_img1=cv2.GaussianBlur(cv_img1,(5,5),0)
        edge_imgh=cv2.Sobel(blur_img1,cv2.CV_8U,1,0)
        edge_imgv=cv2.Sobel(blur_img1,cv2.CV_8U,0,1)
        edge_imghv=cv2.Sobel(blur_img1,cv2.CV_8U,1,1)
        edge_imgf1=cv2.bitwise_or(edge_imgv,edge_imgh)  
        edge_img=cv2.bitwise_or(edge_imghv,edge_imgf1)
        edge_imgf=cv2.Canny(edge_img,200,20)
        self.cropped_edge=edge_imgf
        white_combine(cv_white)
        yellow_combine(cv_yellow)
    def white_combine(msg):
        #cv_img = self.bridge.imgmsg_to_cv2(msg, "mono8")
        cv_img=msg
        edge_img=cv2.bitwise_and(self.cropped_edge,cv_img)
        ros_white = self.bridge.cv2_to_imgmsg(edge_img,"mono8")
        self.pub3.publish(ros_white)
        kernel_e=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
        self.white_edge=cv2.dilate(edge_img,kernel_e)
        lines = cv2.HoughLinesP(self.white_edge,1,np.pi/180,1,100,10)
        self.final_white=self.output_lines(self.original,lines)
        #ros_white = self.bridge.cv2_to_imgmsg(final_white,"bgr8")
        #self.pub1.publish(ros_white)
    def yellow_combine(msg):
        #cv_img = self.bridge.imgmsg_to_cv2(msg, "mono8")
        cv_img=msg
        edge_img=cv2.bitwise_and(self.cropped_edge,cv_img)
        ros_yellow = self.bridge.cv2_to_imgmsg(edge_img,"mono8")
        self.pub4.publish(ros_yellow)
        kernel_e=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
        self.yellow_edge=cv2.dilate(edge_img,kernel_e)
        lines = cv2.HoughLinesP(self.yellow_edge,1,np.pi/180,1,100,10)
        final_yellow=self.output_lines(self.original,lines)
        final_img=cv2.bitwise_or(self.final_white,final_yellow)
        ros_final = self.bridge.cv2_to_compressed_imgmsg(final_img,"bgr8")
        self.pub.publish(ros_final)
if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("ImageProcess", anonymous=True)
    img_filter = ImageProcess()
    rospy.spin()        
