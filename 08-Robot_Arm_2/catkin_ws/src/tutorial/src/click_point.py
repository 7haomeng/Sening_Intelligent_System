#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from yumipy_service_bridge.srv import GotoPose, GotoPoseRequest
meter_to_mm = 1000.0


class ClickPoint:
    def __init__(self):
        rospy.Subscriber("/clicked_point", PointStamped, self.click_point_cb, queue_size=1)
        self.go_pose_plan_srv = rospy.ServiceProxy("/goto_pose_plan", GotoPose)
        self.left_arm_quat = [0, 0, -0.9238795, 0.3826834]
        self.right_arm_quat = [0, 0, 0.9238795, 0.3826834]

    def click_point_cb(self, msg):
        req = GotoPoseRequest()
        
	req.arm = right
	req.position = [0, 0, 0]
	req.position[0] = (msg.point.x)*meter_to_mm
	req.position[1] = (msg.point.y)*meter_to_mm
	req.position[2] = (msg.point.z)*meter_to_mm
	req.quat = self.right_arm_quat
	#--------------------------------------#
	# please finish this callback function
	# this function takes the msg for input
	# and call the goto_pose_plan service 
	# after transform it to self defined 
	# service call GotoPose
	# The format of GotoPose is like below
	# 
	#-------------GotoPose.srv-------------#
	#
	# string arm
	# bool wait_for_res
	# float64[] position
	# float64[] quat
	# ---
	# bool success
	#
	#------------GotoPose.srv--------------#
	#
	# you only need to fill in the parameters
	# 'arm', 'position' and 'quat'
	# 'arm' should be either 'left' or 'right'
	# 'position' is a list where the content
	# is in mm unit while the 3D position of
	# the clicked point in msg is in meter unit
	# 'quat' is provided inside __init__()
	#
	# after you finish this callback function
	# please push it to your branch and test 
	# it with the real yumi robot.
	#--------------------------------------#

        ret = self.go_pose_plan_srv(req)


if __name__ == '__main__':
    rospy.init_node('click_point_node')
    node = ClickPoint()
    rospy.spin()
