#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from homework3.msg import homework3

counter = 0;

def callback(data):
	global counter 
	message_string = homework3()
        counter += 1
        message_string.num = counter
        message_string.myname = data.data    
        messageout = message_string.myname + " Packet number " + str(message_string.num)
        print messageout
        pub.publish(message_string)


if __name__ == '__main__':
	rospy.init_node('mediator')

	sub = rospy.Subscriber("chatter", String, callback)

	pub = rospy.Publisher("new1topic", homework3, queue_size=10)

	rospy.spin()

