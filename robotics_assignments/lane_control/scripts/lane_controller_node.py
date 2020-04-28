#!/usr/bin/env python
import math
import time
import numpy as np
import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading
import time
import numpy as np
import abc
from random import random


class PID:
    
    def __init__(self):

        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.PoseHandling, queue_size=1)
        self.error_accum = 0
        self.previous_error = 0
        self.desired =0.0
        self.Kp = 5.0
        self.Ki = 0.01
        self.Kd = 0
        self.inte = 0
        self.deri=0

            
        self.error_accum_phi = 0
        self.previous_error_phi = 0
        self.desired_phi =0.0
        self.Kp_phi = 7.0
        self.Ki_phi = 0.01
        self.Kd_phi = 0
        self.inte_phi = 0
        self.deri_phi=0

        self.msg =Twist2DStamped()
        self.t_start = rospy.get_time()


    def PoseHandling(self, input_pose_msg):
        x= input_pose_msg.d
        d = input_pose_msg.phi
        t = rospy.get_time()
        self.dt = t - self.t_start
        self.msg.v =0.3

        self.error_phi = self.desired_phi - d 
        self.error = self.desired -x

        self.error_accum += self.error * self.dt
        self.error_accum_phi += self.error_phi * self.dt

        self.inte += self.error_accum
        self.inte_phi += self.error_accum_phi


        if self.dt !=0:
            self.deri =(self.error - self.previous_error)/self.dt
            self.deri_phi =(self.error_phi - self.previous_error_phi)/self.dt
        else:
            self.deri =0.0
            self.deri_phi =0.0

        self.deri_phi = max(min(self.deri_phi,10.0),-30.0)
        self.deri = max(min(self.deri,10.0),-30.0)
        
        
        self.previous_error = self.error
        self.previous_error_phi = self.error_phi

        self.msg.omega = (self.Kp*self.error + self.Kd * self.deri + self.Ki* self.inte) + (self.Kp_phi*self.error_phi + self.Kd_phi * self.deri_phi + self.Ki_phi* self.inte_phi)
        self.msg.omega = max(min(self.msg.omega,10.0),-30.0)
        return self.msg.omega


if __name__ == "__main__":
    rospy.init_node("lane_controller_node", anonymous=False)
    lane_corrected = PID()
    rate = rospy.Rate(10) # 10hz
    while not  rospy.is_shutdown():
        lane_corrected.pub_car_cmd.publish(lane_corrected.msg)
        rate.sleep()

