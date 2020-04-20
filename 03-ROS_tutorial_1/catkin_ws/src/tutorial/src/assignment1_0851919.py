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
def atm2mmhg(atm):
	mmhg = atm / 1.33332
	return (mmhg)

def callback(msg):
	atm = msg.data
	mmhg.data = atm2mmhg(atm)
	# Print information and publish
	rospy.loginfo("Pressure : %.2f ", mmhg.data)
	pub.publish(mmhg)

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
	sub = rospy.Subscriber("/sensor/pressure", Float64, callback, queue_size = 10)
	rospy.spin()

if __name__ == "__main__":
	main()

