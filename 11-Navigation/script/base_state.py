
#import appopriate dependencies
from pyrobot import Robot

# Create robot.
robot = Robot('locobot')

# Get the current pose of the base i.e, a state of the form (x, y, yaw). Note
# that over here we are querying the 'odom' state, which is state estimated
# from inertial sensors and wheel encoders on the base.
current_state = robot.base.get_state('odom')

print('{}'.format(current_state))

raw_input("Push the locobot forward and press Enter to continue...")

current_state = robot.base.get_state('odom')

print('{}'.format(current_state))
