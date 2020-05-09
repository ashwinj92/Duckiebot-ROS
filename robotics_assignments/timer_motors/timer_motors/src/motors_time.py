#!/usr/bin/env python

import rospy
import time
import random
from duckietown_msgs.msg import  WheelsCmdStamped
from std_msgs.msg import Float32
from math import sin,cos,pi
from geometry_msgs.msg import Vector3
import matplotlib.pyplot as plt
from std_msgs.msg import String


total_time = 0.0
xval = 0.0
yval = 0.0
stheta = 0.0
last_time = 0.0 

def initialize_globals():
     global last_time 
     last_time = rospy.get_time()

def callback(data):

    time_start = rospy.get_time() 
    global total_time
    global last_time
    if last_time != -1.0:
        delta_t =  time_start - last_time
    global xval
    global yval
    global pub
    global stheta
    left = delta_t * data.vel_left
    right = delta_t * data.vel_right
    delTheta = 0.0
    delS = (left+right)/2
    alpha = (right-left)/0.1
    delTheta = alpha
    delX = delS *(cos(stheta + delTheta/2))
    delY = delS *(sin(stheta + delTheta/2))
    xval = xval + delX
    yval = yval + delY
    stheta = stheta + delTheta
    poseX = xval
    poseY = yval
    poseTheta = stheta  
    poseData = "Xval:   " + str(poseX) + "   Yval:   " + str(poseY) + "  Theta:    " + str(poseTheta)
    rospy.loginfo(rospy.get_caller_id() + "entered callback")
    pub.publish(poseData)
    last_time = time_start

if __name__ == '__main__':
	rospy.init_node('timer')

        initialize_globals()

	sub = rospy.Subscriber("/ashwinsduckie/wheels_driver_node/wheels_cmd_executed", WheelsCmdStamped, callback)

	pub = rospy.Publisher("pose", String, queue_size=10)

	rospy.spin()
