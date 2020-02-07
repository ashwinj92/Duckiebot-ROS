#!/usr/bin/env python
import rospy
from std_msgs.msg import String



def callback(data):
	
	message_string = "MESSAGE: " + data.data 
	rospy.loginfo(message_string)
	pub.publish(message_string)

if __name__ == '__main__':
	rospy.init_node('mediator')

	sub = rospy.Subscriber("/homework1/chatter", String, callback)

	pub = rospy.Publisher("newtopic", String, queue_size=10)

	rospy.spin()

