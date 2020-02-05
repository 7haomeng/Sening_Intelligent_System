#!/usr/bin/python

# Test self-defined message
# Editer: Sean Lu
# Last modify: 9/26
# Changelog:
# 	Version 1.0

import rospy # ROS Python Client API Library
import message_filters # Multi-subscriber with one callback
# Message types we use
from tutorial.msg import my_message
from std_msgs.msg import Float64

# Get both temperature and pressure, publish the data with new message type
def cb(temp, pressure):
	my_msg.temp = temp.data
	my_msg.pressure = pressure.data
	pub.publish(my_msg)

def main():
	rospy.init_node("test_my_message_node", anonymous = False)
	# Message
	global my_msg
	my_msg = my_message()
	# Publisher
	global pub
	pub = rospy.Publisher("/sensor/my_msg", my_message, queue_size = 10)
	# Subscribers
	sub_temp = message_filters.Subscriber("/sensor/temp", Float64)
	sub_pressure = message_filters.Subscriber("/sensor/pressure", Float64)
	# Declare an ApproximateTimeSynchronizer object and register a callback
	ts = message_filters.ApproximateTimeSynchronizer([sub_temp, sub_pressure], 10, 0.1,\
							allow_headerless = True)
	ts.registerCallback(cb)
	
	rospy.spin()

if __name__ == "__main__":
	main()


