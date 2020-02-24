#!/usr/bin/env python

import rospy
from homework4.srv import testservice
import sys
import time
import random 

def request(modeChange):

    rospy.wait_for_service('service_example')
    srv=rospy.ServiceProxy('service_example',testservice)
    service_example=srv(modeChange)
    return service_example

def changemode():
     modeSelect = random.randint(1, 10)
     rospy.loginfo("Requesting a mode change to: "+ str(modeSelect))
     data = request(modeSelect)
     rospy.loginfo("sleep for 0.5 seconds ")
     time.sleep(0.5)
     modeSelect = random.randint(1, 10)
     rospy.loginfo("Requesting a mode change to: "+ str(modeSelect))
     time.sleep(0.5)
     data = request(modeSelect)
     message = "Current mode on the server is: " + str(data) 
     rospy.loginfo(message)
   
if __name__ == "__main__":

      rospy.init_node('client')
      # Send 11 to request the current mode
      # This section before the while loop below gets the first
      # Mode set in the params tag in the .launch file which is "5" 
      firstMode = "The first mode: " + str(request(11))
      rospy.loginfo(firstMode)
      # This loop controls the mode change
      while True:
           changemode()

