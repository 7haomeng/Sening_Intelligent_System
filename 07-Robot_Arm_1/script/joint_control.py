from pyrobot import Robot
import numpy as np
robot = Robot('locobot')

target_joints = [0.408, 0.721, -0.471, -1.4, 0.920]

robot.arm.go_home()

robot.arm.set_joint_positions(target_joints, plan=False)
