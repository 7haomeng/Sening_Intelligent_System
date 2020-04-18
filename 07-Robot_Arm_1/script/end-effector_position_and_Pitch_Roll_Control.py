from pyrobot import Robot
import numpy as np
robot = Robot('locobot')

target_poses = {'position': np.array([0.28, 0.17, 0.22]),
                'pitch': 0.5,
                'roll': 0.5,
                'numerical': False}

robot.arm.go_home()

robot.arm.set_ee_pose_pitch_roll(**target_poses)

