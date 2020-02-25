#!/usr/bin/env python

import rospy
from homework4.srv import *
import sys
import time
import random 

def getRequest():

    rospy.wait_for_service('get_service')
    srv=rospy.ServiceProxy('get_service',get)
    service_example=srv()
    return service_example

def postRequest(modeChange):

    rospy.wait_for_service('post_service')
    srv=rospy.ServiceProxy('post_service',post)
    service_example=srv(modeChange)
    return service_example

def changeModeAndGet():
     modeSelect = random.randint(1, 10)
     rospy.loginfo("Requesting a mode change to: "+ str(modeSelect))
     postdata = postRequest(modeSelect)
     rospy.loginfo("Response: "+ postdata.response)
     getdata = getRequest()
     rospy.loginfo("Current Mode: "+ str(getdata))
     time.sleep(0.5)
   
if __name__ == "__main__":

      rospy.init_node('client1')
      # This section before the while loop below gets the first
      # Mode set in the params tag in the .launch file which is "5" 
      firstMode = "The first mode: " + str(getRequest())
      rospy.loginfo(firstMode)
      # This loop controls the mode change
      while True:
           changeModeAndGet()

