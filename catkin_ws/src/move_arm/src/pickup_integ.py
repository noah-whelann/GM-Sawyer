#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from geometry_msgs.msg import Point, PoseStamped
from moveit_commander import MoveGroupCommander
from intera_interface import gripper as robot_gripper, Limb
import rospkg
import roslaunch

# 0.730 0.302 -0.112 -0.128 0.991 -0.006 -0.022 0.793 0.032 -0.115 -0.069 0.996 -0.046 -0.014
# best depth -.178

# tuck -> move above the piece to pickup -> lower down to piece & grab it ->
#  move straight up -> move to new piece location -> then lower down to location -> then tuck -> loop
# 0.484, 0.443, -0.178, 0.0, 1.0, 0.0, 0.0, 0.865, 0.156, -0.178, 0.0, 1.0, 0.0, 0.0
# 0.484, 0.443, -0.007, 0.0, 1.0, 0.0, 0.0, 0.865, 0.156, -0.007, 0.0, 1.0, 0.0, 0.0 #abitrary higher location

# test 2
# 0.594, -0.382, -0.007, 0.0, 1.0, 0.0, 0.0, 0.882, 0.347, -0.007, 0.0, 1.0, 0.0, 0.0
# 0.594, -0.382, 0.882, 0.347
# 0.788, 0.238, 0.694, 0.002
# 0.848, 0.100, 0.489, 0.250
# 0.823, 0.278, 0.741, -0.071
# 0.818, 0.396, 0.443, 0.085
# 0.439, 0.079, 0.818, 0.396

# test 3 (kill pawn w queen)
# pawn move: 0.699, 0.289, 0.722, -0.013
# queen move 0.682, 0.521, 0.699, 0.289

fast_v = 0.5  # slowed for auto test
med_v = 0.6
slow_v = 0.2


def pickup_and_place(pickup_coords: Point, place_coords: Point, capture: bool) -> None:
    # starts node
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

    group.set_max_velocity_scaling_factor(fast_v)  # 1.0 is fastest

    z_pos = {"hover": -0.141, "grab": -0.167, "reset": 0.231, "drop": 0.15}
    tuck(group, False)

    # while not rospy.is_shutdown():
    mode = input('is this auto mode? y/n')

    try:
        # Pickup piece (hover)
        print('Starting piece pickup...')
        group.set_max_velocity_scaling_factor(fast_v)
        if execute_pose(group, compute_ik, pickup_coords, z_pos['hover'], "base", "right_gripper_tip", mode):
            print('Opening')
            right_gripper.open()
            rospy.sleep(1.0)

        # Lowering to pickup piece (grab)
        print('Grabbing piece...')
        group.set_max_velocity_scaling_factor(slow_v)
        execute_pose(group, compute_ik, pickup_coords,
                     z_pos['grab'], "base", "right_gripper_tip", mode)
        right_gripper.close()
        rospy.sleep(1.0)

        # Move piece straight up (reset)
        print('Lifting piece...')
        group.set_max_velocity_scaling_factor(fast_v)
        execute_pose(group, compute_ik, pickup_coords,
                     z_pos['reset'], "base", "right_gripper_tip", mode)

        if capture:  # if performing a piece capture, set z differently and skip place movement
            # Place piece down (hover)
            print('Dropping captured piece')
            execute_pose(group, compute_ik, place_coords,
                         z_pos['drop'], "base", "right_gripper_tip", mode)

        else:
            print('Moving to place position...')
            execute_pose(group, compute_ik, place_coords,
                         z_pos['hover'], "base", "right_gripper_tip", mode)

            # Lowering to place piece (grab)
            print('Lowering piece..')
            group.set_max_velocity_scaling_factor(slow_v)
            execute_pose(group, compute_ik, place_coords,
                         z_pos['grab'], "base", "right_gripper_tip", mode)
            print('Piece released')

        right_gripper.open()
        rospy.sleep(1.0)

        # Reset upwards
        print('Resetting arm...')
        group.set_max_velocity_scaling_factor(fast_v)
        execute_pose(group, compute_ik, place_coords,
                     z_pos['reset'], "base", "right_gripper_tip", mode)

        tuck(group, True)

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
    except Exception as ex:
        print(f"An error occurred: {ex}")


def execute_pose(group, compute_ik, coords: Point, z, frame_id, link_name, mode):

    auto = True if mode == 'y' else False
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"
    request.ik_request.ik_link_name = link_name
    request.ik_request.pose_stamped.header.frame_id = frame_id
    request.ik_request.pose_stamped.pose.position.x = coords[0]
    request.ik_request.pose_stamped.pose.position.y = coords[1]
    request.ik_request.pose_stamped.pose.position.z = z
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0

    response = compute_ik(request)

    if response.error_code.val != 1:
        print("IK solution not found! Attempting to tuck")
        if tuck(group, False):
            print("tucked successfully!")
            return True
        print("ggs lol")  # probably need have some contingency here
        return False

    # Set the target pose and plan
    group.set_pose_target(request.ik_request.pose_stamped)
    plan = group.plan()

    # confirm move
    if auto:
        # be careful with the auto mode, it wont wait for you to preview
        group.execute(plan[1], wait=True)
        rospy.sleep(1.0)
        return True
    else:
        user_input = input(
            "Enter 'y' if the trajectory looks safe in RViz: Hit n to tuck and try again ")
        if user_input.lower() == 'y' and not auto:
            group.execute(plan[1], wait=True)
            rospy.sleep(1.0)
            return True
        elif user_input.lower() == 'n' and not auto:
            tuck(group, True)  # tucks robot and tries move again
            print('Tucked')
            rospy.sleep(1.0)
            print('Retrying')
            # will tuck and retry the move until user says yes or stops programto:
            return execute_pose(group, compute_ik, coords, z, frame_id, link_name)

    print("Operation stopped by user.")
    return False


def tuck(group, auto):
    """
    Tuck the robot arm to the start position. Use with caution
    """
    group.set_max_velocity_scaling_factor(slow_v)
    rospy.sleep(1.0)
    arm = Limb('right')  # used for tuck
    joint_angles = {  # tuck joints
        'right_j0': 0.0,
        'right_j1': -1.0,
        'right_j2': 0.0,
        'right_j3': 1.0,
        'right_j4': 0.0,
        'right_j5': 1.6,
        'right_j6': 1.57079632679
    }
    if auto:
        arm.move_to_joint_positions(joint_angles)
        return True
    if input('Would you like to tuck the arm? (y/n): ') == 'y':
        # DONT tuck if already at the position, will cause robot to stop
        arm.move_to_joint_positions(joint_angles)
        return True
    else:
        print('Canceled. Not tucking the arm.')
        return False


if __name__ == '__main__':
    pickup_and_place()
