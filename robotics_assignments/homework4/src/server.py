#!/usr/bin/env python

import rospy
from homework4.srv import testservice, testserviceResponse

# read the parameter mode from the launch file
mode = rospy.get_param('mode')

# funtion that sends the current mode
def sendCurrentMode(mess):
   global mode
   # for integers between 1 to 10 a mode change is performed
   if mess.statusrequest >= 1 and mess.statusrequest <= 10:
       mode = mess.statusrequest
       return  testserviceResponse(mode)
   # if the client sends a 11 then we just return the current mode 
   # no change is made
   elif mess.statusrequest == 11:
       return  testserviceResponse(mode)
   # if the request contains anything other than numbers between (0 to 11)
   # we respond with a 404 do nothing return an error code of 404
   # test this with the following command rosservice call service_example 33
   # corner case
   else:
       return testserviceResponse(404)

def sendstatus():
    global mode
    rospy.init_node('server')
    service=rospy.Service('service_example',testservice,sendCurrentMode)
    rospy.spin()

if __name__ == "__main__":
      sendstatus()
