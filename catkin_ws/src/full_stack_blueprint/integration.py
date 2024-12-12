#!/usr/bin/env python

import rospy
import intera_interface

from moveit_commander import MoveGroupCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

import numpy as np

class CLI:
    def __init__(self):
        rospy.init_node('service_query')

        ### SAWYER ARM CONFIGURATIONS ###
        self.standard_tuck_angles = { #tuck joints
            'right_j0': 0.0,
            'right_j1': -1.0,
            'right_j2': 0.0,
            'right_j3': 1.0,
            'right_j4': 0.0,
            'right_j5': 1.6,
            'right_j6': 1.57079632679
        }

        self.custom_tuck_angles = {
            'right_j0': 0, 
            'right_j1': -1, 
            "right_j2": 0,
            "right_j3": 1.5,
            "right_j4": 0,
            "right_j5": -0.5,
            "right_j6": 1.7
        }

        self.commander = MoveGroupCommander("right_arm")

        self.limb = intera_interface.Limb("right")

        self.arm_speeds = {"slow": 0.2, "fast": 0.6}

        self.tip_name = "right_gripper_tip"


        ### BOARD COORDINATES CONFIGURATIONS ###

        self.a1_xy = np.array([]) # TODO: hardcode a1's x and y coordinates
        self.a2_xy = np.array([]) # TODO: hardcode a2's x and y coordinates
        self.b1_xy = np.array([]) # TODO: hardcode b1's x and y coordinates

        self.right_increment = self.a2_xy - self.a1_xy
        self.down_increment = self.b1_xy - self.a1_xy

        self.z = {"hover":-0.141, "grab":-0.167, "reset":0.231, "drop":0.15}

        self.tile_to_coords = {}

    def run(self):
        self.calibrate()

    def calibrate(self):
        # Tuck the Sawyer Arm
        if input("Would you like to tuck the arm? (y/n) \n") == "y":
            self.tuck()

        input("Please confirm that the board is properly placed. (Enter) \n")

        return

    ######################
    ### ARM KINEMATICS ###
    ######################

    def tuck(self):
        print("Tucking Sawyer Robot...")

        self.commander.set_max_velocity_scaling_factor(self.arm_speeds["slow"])
        rospy.sleep(1.0)

        self.limb.move_to_joint_positions(self.standard_tuck_angles)

        print("Finished Tucking Sawyer Robot. \n \n")

    def arm_to_tile(self, tile_name):
        row = tile_name[0]  # alphabet
        col = tile_name[1]  # number

        num_down = ord(row) - ord('a')
        num_right = int(col) - 1

        target_xy = self.a1_xy + self.down_increment * num_down + self.right_increment + num_right

        self.move_to_coord(target_xy, self.z["hover"])

    def move_to_coord(self, xy_arr, z):
        rospy.wait_for_service('compute_ik')
        
        compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

        self.commander.set_max_velocity_scaling_factor(self.arm_speeds["slow"])

        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        request.ik_request.ik_link_name = self.tip_name
        request.ik_request.pose_stamped.header.frame_id = "base"
        request.ik_request.pose_stamped.pose.position.x = xy_arr[0]
        request.ik_request.pose_stamped.pose.position.y = xy_arr[1]
        request.ik_request.pose_stamped.pose.position.z = z
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0

        response = compute_ik(request)

        if response.error_code.val != 1:
            print("IK solution not found. Attempting to tuck.")
            self.tuck()
            return

        self.commander.set_pose_target(request.ik_request.pose_stamped)
        plan = self.commander.plan()

        if input("Enter 'y' if the trajectory looks safe in RViz: \n") == "y":
            self.commander.execute(plan[1], wait=True)
            rospy.sleep(1.0)

    ###################
    ### BOARD STATE ###
    ###################










if __name__ == "__main__":
    cli = CLI()

    cli.run()