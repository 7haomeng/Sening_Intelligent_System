
#import appopriate dependencies
from pyrobot import Robot

# Create the Robot object that interfaces with the robot.
robot = Robot('locobot') 
# If you want to use LoCoBot-Lite instead, replace the argument 'locobot' with
# 'locobot_lite'

# linear_velocity in m/s
linear_velocity = 0.1 

# rotational_velocity in radian / s
rotational_velocity = 0.5 

# execution_time in seconds
execution_time = 4 

# Command to execute motion
robot.base.set_vel(fwd_speed=linear_velocity, 
                   turn_speed=rotational_velocity, 
                   exe_time=execution_time)

# To stop the robot at any time:
robot.base.stop() 
