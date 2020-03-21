#!/usr/bin/env python

import rospy
from homework4.srv import *

# read the parameter mode from the launch file
param_name = rospy.search_param('mode')
mode = rospy.get_param(param_name)

# service that changes the mode
def changeMode(mess):
   global mode
   # for integers between 1 to 10 a mode change is performed
   if mess.post >= 1 and mess.post <= 10:
       mode = mess.post
       # sends a string message OK to indicate a mode change
       return  postResponse("ok")
   # if the client sends anything other then (1-10) it just returns the below message 
   else:
       return postResponse("Select a mode between 1-10")

# service that sends the current mode
def currentMode(mess):
   global mode
   return  getResponse(mode)

def sendstatus():
    global mode
    rospy.init_node('server1')
    servicePost=rospy.Service('post_service',post,changeMode)
    serviceGet=rospy.Service('get_service',get,currentMode)
    rospy.spin()

if __name__ == "__main__":
      sendstatus()
