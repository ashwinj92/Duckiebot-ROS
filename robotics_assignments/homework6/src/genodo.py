#!/usr/bin/env python

from math import sin,cos,pi
import rospy
from geometry_msgs.msg import Vector3
import matplotlib.pyplot as plt
from odometry_hw.msg import *
import time
import random 

xval = 0.0
yval = 0.0
stheta = 0.0
def callback(data):
        global xval
        global yval
        global pub
        global stheta
        pose = Pose2D()
        left = data.dist_wheel_left
        right = data.dist_wheel_right
        delTheta = 0.0
        delS = (left+right)/2
        alpha = (right-left)/0.1
        delTheta = alpha
        delX = delS *(cos(stheta + delTheta/2))
        delY = delS *(sin(stheta + delTheta/2))
        xval = xval + delX
        yval = yval + delY
        stheta = stheta + delTheta
        pose.x = xval
        pose.y = yval
        pose.theta = stheta
        rospy.loginfo(rospy.get_caller_id() + "entered callback")
        pub.publish(pose)


if __name__ == '__main__':
	rospy.init_node('genodo')

	sub = rospy.Subscriber("/dist_wheel", DistWheel, callback)

	pub = rospy.Publisher("pose", Pose2D, queue_size=10) 

	rospy.spin()


