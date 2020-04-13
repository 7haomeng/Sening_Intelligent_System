from pyrobot import Robot
import numpy as np
import time
robot = Robot('locobot')

robot.gripper.open()
time.sleep(1)

robot.gripper.close()
