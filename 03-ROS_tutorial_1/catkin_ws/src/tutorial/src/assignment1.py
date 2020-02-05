#!/usr/bin/python

# Publisher/ Subscriber assignment
# Editer: Sean Lu
# Last modify: 9/22
# Changelog:
#   Version 1.0

import rospy
from std_msgs.msg import Float64

# Callback for /sensor/pressure
# Get the pressure in hPa, convert 
# it to mmHg and print the information 
def callback(msg):
	############### Student Implimentation ###############
	#						     #
	#	Complete your callback function here	     #
	#						     #
	############### Student Implimentation ###############

# Intialize the node, publisher and subscriber
def main():
	rospy.init_node("convert_pressure_unit_node", anonymous = False)
	# Message
	global mmhg
	mmhg = Float64()
	# Publisher
	global pub
	pub = rospy.Publisher("/sensor/pressure_mmhg", Float64, queue_size = 10)
	# Subscriber
	############### Student Implimentation ###############
        #						     #
	#               Subscribe to which topic             #
        #                                                    #
	############### Student Implimentation ###############
	rospy.spin()

if __name__ == "__main__":
	main()

