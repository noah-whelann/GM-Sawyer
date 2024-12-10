#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from intera_interface import gripper as robot_gripper
# 0.730 0.302 -0.112 -0.128 0.991 -0.006 -0.022 0.793 0.032 -0.115 -0.069 0.996 -0.046 -0.014

# 0.484, 0.443, -0.178, 0.0, 1.0, 0.0, 0.0, 0.865, 0.156, -0.178, 0.0, 1.0, 0.0, 0.0

#best depth -.178

#tuck -> move above the piece to pickup -> lower down to piece & grab it ->
#  move straight up -> move to new piece location -> then lower down to location -> then tuck -> loop
#0.484, 0.443, -0.178, 0.0, 1.0, 0.0, 0.0, 0.865, 0.156, -0.178, 0.0, 1.0, 0.0, 0.0
#0.484, 0.443, -0.007, 0.0, 1.0, 0.0, 0.0, 0.865, 0.156, -0.007, 0.0, 1.0, 0.0, 0.0 #abitrary higher location

#test 2
#0.594, -0.382, -0.007, 0.0, 1.0, 0.0, 0.0, 0.882, 0.347, -0.007, 0.0, 1.0, 0.0, 0.0

def main():
    #starts node
    rospy.init_node('service_query')
    rospy.wait_for_service('compute_ik')

    # starts ik service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    # calibrate grippy
    right_gripper = robot_gripper.Gripper('right_gripper')
    print('Calibrating gripper...')
    right_gripper.calibrate()
    rospy.sleep(2.0)

    # setup right arm
    group = MoveGroupCommander("right_arm")

    while not rospy.is_shutdown():
        input('Press [ Enter ] to start a pick-and-place operation: ')

        coordinates = input("Enter a comma-separated list of 14 coordinates (7 for piece pickup, 7 for piece place): ")
        try:
            coordinate_array = [float(coord) for coord in coordinates.split(',')]
            if len(coordinate_array) != 14:
                raise ValueError("You must enter exactly 14 coordinates.")
        except ValueError as e:
            print(f"Invalid input: {e}")
            continue

        pickup_coords = coordinate_array[:7]
        place_coords = coordinate_array[7:]

        try:
            # Pickup piece
            print('Starting piece pickup...')
            if execute_pose(group, compute_ik, pickup_coords, "base", "right_gripper_tip"):
                print('Opening gripper...')
                right_gripper.open()
                rospy.sleep(1.0)
                print('Gripper opened. Closing gripper to pick up...')
                right_gripper.close()
                rospy.sleep(1.0)

            # Move piece straight up
            print('Lifting piece...')
            pickup_coords[2] += 0.3  # Increase Z position to lift the piece
            execute_pose(group, compute_ik, pickup_coords, "base", "right_gripper_tip")

            # Place piece down
            print('Moving to place position...')
            if execute_pose(group, compute_ik, place_coords, "base", "right_gripper_tip"):
                print('Opening gripper to release piece...')
                right_gripper.open()
                rospy.sleep(1.0)

            # Reset upwards
            print('Resetting arm...')
            place_coords[2] += 0.3  # Increase Z position for reset
            execute_pose(group, compute_ik, place_coords, "base", "right_gripper_tip")
            ## Should tuck here instead

        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
        except Exception as ex:
            print(f"An error occurred: {ex}")

def execute_pose(group, compute_ik, coords, frame_id, link_name):

    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"
    request.ik_request.ik_link_name = link_name
    request.ik_request.pose_stamped.header.frame_id = frame_id
    request.ik_request.pose_stamped.pose.position.x = coords[0]
    request.ik_request.pose_stamped.pose.position.y = coords[1]
    request.ik_request.pose_stamped.pose.position.z = coords[2]
    request.ik_request.pose_stamped.pose.orientation.x = coords[3]
    request.ik_request.pose_stamped.pose.orientation.y = coords[4]
    request.ik_request.pose_stamped.pose.orientation.z = coords[5]
    request.ik_request.pose_stamped.pose.orientation.w = coords[6]

    response = compute_ik(request)

    if response.error_code.val != 1:
        print("IK solution not found!")
        return False

    # Set the target pose and plan
    group.set_pose_target(request.ik_request.pose_stamped)
    plan = group.plan()

    # confirm move
    user_input = input("Enter 'y' if the trajectory looks safe in RViz: ")
    if user_input.lower() == 'y':
        group.execute(plan[1], wait=True)
        rospy.sleep(2.0)
        return True

    print("Operation stopped by user.")
    return False


if __name__ == '__main__':
    main()
