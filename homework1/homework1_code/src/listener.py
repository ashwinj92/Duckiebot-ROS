#!/usr/bin/env python
# adapted from http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

import rospy
from std_msgs.msg import String

class listener:
    def __init__(self):
        rospy.Subscriber("chatter", String, self.callback)

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    listener()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()