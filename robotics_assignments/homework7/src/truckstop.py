#!/usr/bin/env python
import math
import time
import numpy as np
import rospy
from std_msgs.msg import Float32
from random import random


class pid_controller():

    def __init__(self):
		
	self.pub = rospy.Publisher("control_input", Float32, queue_size=10)
	rospy.Subscriber("error", Float32, self.callback, queue_size=10)
	self.kp = 2
	self.ki = 0
	self.kd = 1
	self.now = rospy.get_time()
	self.dt = 0
	self.error = 0
	self.error_sum = 0
	self.der = 0
	self.correction = 0
	self.prev_error = 0
	self.prev_time = 0

    def callback(self,data):
	self.now = rospy.get_time()
	self.dt = self.now - self.prev_time
	self.error = data.data
	self.error_sum += self.dt*self.error
	self.der = (self.error-self.prev_error)/self.dt
	self.correction = (self.kp*self.error) + (self.ki*self.error_sum)+ (self.kd*self.der)
	self.prev_error = self.error
	self.prev_time = self.now
        #rospy.loginfo("entered callback")
	print('call back')
        self.pub.publish(self.correction)


if __name__ == "__main__":

    rospy.init_node("stop_truck", anonymous=False)    
    pid_controller()
    rospy.loginfo("entered main")
    rospy.spin()
