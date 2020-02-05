#!/usr/bin/env python

####################################################
# Read temperature and atmosphere pressure data    #
# measured from BMP085 through serial and publish  #
# as ROS topic                                     #
####################################################

# Editor: Sean Lu
# Last update: 9/20, 2018

'''
    Changelog: 
    	Version 1.0 9/20
	Version 1.1 add baud parameter
		    change timer to 10.0 Hz
'''

import rospy
import serial
from std_msgs.msg import Float64

# ROS Publish
temp_pub = rospy.Publisher('/sensor/temp', Float64, queue_size = 10)
pressure_pub = rospy.Publisher('/sensor/pressure', Float64, queue_size = 10)

# Message
pressure = Float64()
temp = Float64()

# This callback costs about 0.14s to process, so the 
# maximum frequncy of the topics are about 7.0 Hz
def cb(event):
	res_str = ard.readline()
	try:
		data_arr = [float(x) for x in res_str.split(' ')]
		pressure.data = data_arr[0]
		temp.data = data_arr[1]
		# Publish
		temp_pub.publish(temp)
		pressure_pub.publish(pressure)
	except ValueError or IndexError as e:
		rospy.logerr(e)
		pass

if __name__ == '__main__':
	rospy.init_node('ard_serial', anonymous = False)
	# Parameters
	port = rospy.get_param("~port", "/dev/ttyUSB0")
	baud = rospy.get_param("~baud", 115200)
	# Remember to permit port first
	ard = serial.Serial(port, baud)
	# Flush first 10 data
	for i in range(10):
		_ = ard.readline()
	# Setup a timer with 10.0 Hz
	rospy.Timer(rospy.Duration.from_sec(0.10), cb)
	rospy.spin()
