from pyrobot import Robot
import numpy as np
robot = Robot('locobot')
import time

target_poses = {'position': np.array([0.279, 0.176, 0.217]),
                 'orientation': np.array([[0.5380200, -0.6650449, 0.5179283],
                                          [0.4758410, 0.7467951, 0.4646209],
                                          [-0.6957800, -0.0035238, 0.7182463]])}
robot.arm.go_home()

robot.arm.set_ee_pose(**target_poses)
