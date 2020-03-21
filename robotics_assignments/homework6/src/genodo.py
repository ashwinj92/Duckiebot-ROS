#!/usr/bin/env python

import rospy
from odometry_hw.msg import *
import time
import random 

def callback(data):
        pose = Pose2D()
        pose.x = data.dist_wheel_left
        pose.y = data.dist_wheel_right
        pose.x = xval+0.1
        pose.y = yval+0.1
        pose.theta =  random.uniform(0.0,90.0)   
        rospy.loginfo(rospy.get_caller_id() + "entered callback")
        pub.publish(pose)


if __name__ == '__main__':
	rospy.init_node('genodo')

	sub = rospy.Subscriber("/dist_wheel", DistWheel, callback)

	pub = rospy.Publisher("pose", Pose2D, queue_size=10) 

	rospy.spin()


