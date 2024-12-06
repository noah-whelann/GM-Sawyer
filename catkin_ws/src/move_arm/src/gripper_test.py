#!/usr/bin/env python
import rospy

from intera_interface import gripper as robot_gripper

rospy.init_node('gripper_test')

# Set up the right gripper
right_gripper = robot_gripper.Gripper('right_gripper')

# Calibrate the gripper (other commands won't work unless you do this first)
print('Calibrating...')
right_gripper.calibrate()
rospy.sleep(0.5)
right_gripper.close()
rospy.sleep(0.5)
right_gripper.open()

print('Closing in 2 seconds')
rospy.sleep(2.0)

# Close the right gripper
print('Closing...')
right_gripper.close()

# Open the right gripper