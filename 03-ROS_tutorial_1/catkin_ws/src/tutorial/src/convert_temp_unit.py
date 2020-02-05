#!/usr/bin/python
# We need the line above to run the command 'rosrun ...'
# or it will be confused how to run this file

# Publisher/ Subscriber tutorial
# Editer: Sean Lu
# Last modify: 9/22
# Changelog:
#   Version 1.0

import rospy # ROS Python Client API Library
from std_msgs.msg import Float64 # Message type we use

# Convert Celsius to Fahrenheit
# Fah = (9/5) * Cel + 32
# Input: temperature in Celsius
# Output: temperature in Fahrenheit
def cel2fah(cel):
	fah_ = 9./5 * cel + 32
	return fah_

# Callback for /sensor/temp
# Get the temperature in Celsius, convert 
# it to Fahrenheit and print the information 
def callback(msg):
	cel = msg.data
	fah.data = cel2fah(cel)
	# Print information and publish
	rospy.loginfo("%.2f degrees Fahrenheit", fah.data)
	pub.publish(fah)
	
# Intialize the node, publisher and subscriber
def main():
	rospy.init_node("convert_temp_unit_node", anonymous = False)
	# Message
	global fah
	fah = Float64()
	# Publisher
	global pub
	pub = rospy.Publisher("/sensor/temp_fah", Float64, queue_size = 10)
	# Subscriber
	sub = rospy.Subscriber("/sensor/temp", Float64, callback, queue_size = 10)
	
	rospy.spin()

if __name__ == "__main__":
	main()

