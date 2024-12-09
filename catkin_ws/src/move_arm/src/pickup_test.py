#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander

from intera_interface import gripper as robot_gripper

import numpy as np
from numpy import linalg
import sys

##Test coordinates
## Pickup @ 0.730 0.302 -0.112 -0.128 0.991 -0.006 -0.022
## Place @ 0.793 0.032 -0.115 -0.069 0.996 -0.046 -0.014
# 0.730 0.302 -0.112 -0.128 0.991 -0.006 -0.022 0.793 0.032 -0.115 -0.069 0.996 -0.046 -0.014
def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    link = "right_gripper_tip"

    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')

    # Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(2.0)

    while not rospy.is_shutdown():
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
   
        # Open the right gripper
        print('Starting piece pickup')
        print('Opening...')
        right_gripper.open()
        rospy.sleep(1.0)
        print('Done!')
        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        coordinates = input("Enter a space-separated list of 14 coordinates (decimal format): (7 for piece pickup, 7 for piece place) ")

        coordinate_array = [float(coord) for coord in coordinates.split(" ")]
        
        print("split coords")
        #coordinates to pick up piece on board
        request.ik_request.pose_stamped.pose.position.x = coordinate_array[0]
        request.ik_request.pose_stamped.pose.position.y = coordinate_array[1]
        request.ik_request.pose_stamped.pose.position.z = coordinate_array[2]     
        request.ik_request.pose_stamped.pose.orientation.x = coordinate_array[3]
        request.ik_request.pose_stamped.pose.orientation.y = coordinate_array[4]
        request.ik_request.pose_stamped.pose.orientation.z = coordinate_array[5]
        request.ik_request.pose_stamped.pose.orientation.w = coordinate_array[6]

        print(coordinate_array)

        print("got request")
        # Set the desired orientation for the end effector HERE
        

        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # Plan IK
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan[1])

            
            #Pickup piece
            print('Closing...')
            right_gripper.close()
            rospy.sleep(3.0)
            print('Piece picked up!')

            
            ## piece is picked up, move straight up (prevent end effector from hitting other pieces)
            request.ik_request.pose_stamped.pose.position.x = coordinate_array[0]
            request.ik_request.pose_stamped.pose.position.y = coordinate_array[1]
            request.ik_request.pose_stamped.pose.position.z = 0.382   
            request.ik_request.pose_stamped.pose.orientation.x = coordinate_array[3]
            request.ik_request.pose_stamped.pose.orientation.y = coordinate_array[4]
            request.ik_request.pose_stamped.pose.orientation.z = 0.001
            request.ik_request.pose_stamped.pose.orientation.w = -0.013

            reponse = compute_ik(request)

            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # Plan IK
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan[1])

            rospy.sleep(2.0)
            
            print('Placing piece down')
             
            ## place piece in correct location
            request.ik_request.pose_stamped.pose.position.x = coordinate_array[7]
            request.ik_request.pose_stamped.pose.position.y = coordinate_array[8]
            request.ik_request.pose_stamped.pose.position.z = coordinate_array[9]  
            request.ik_request.pose_stamped.pose.orientation.x = coordinate_array[10]
            request.ik_request.pose_stamped.pose.orientation.y = coordinate_array[11]
            request.ik_request.pose_stamped.pose.orientation.z = coordinate_array[12]
            request.ik_request.pose_stamped.pose.orientation.w = coordinate_array[13]

            reponse = compute_ik(request)

            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # Plan IK
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan[1])

            rospy.sleep(2.0)
            print('Closing...')
            right_gripper.open()
            rospy.sleep(3.0)
            
            print('Resetting') 
            ## piece is picked up, move straight up (prevent end effector from hitting other pieces)
            request.ik_request.pose_stamped.pose.position.x = coordinate_array[7]
            request.ik_request.pose_stamped.pose.position.y = coordinate_array[8]
            request.ik_request.pose_stamped.pose.position.z = 0.382   
            request.ik_request.pose_stamped.pose.orientation.x = coordinate_array[10]
            request.ik_request.pose_stamped.pose.orientation.y = coordinate_array[11]
            request.ik_rexecute_pose(group, compute_ik, pickup_coords, "base", "right_gripper_tip")equest.pose_stamped.pose.orientation.z = 0.001
            request.ik_request.pose_stamped.pose.orientation.w = -0.013

            reponse = compute_ik(request)

            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # Plan IK
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan[1])

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            
            

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
