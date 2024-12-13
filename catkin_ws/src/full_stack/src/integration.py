#!/usr/bin/env python

import rospy
import intera_interface

from moveit_commander import MoveGroupCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from chess_tracking.srv import Screenshot

import numpy as np

import requests
import json

from board_lib import BoardUpdater

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

        self.commander = MoveGroupCommander("right_arm", wait_for_servers=10)

        self.limb = intera_interface.Limb("right")

        self.arm_speeds = {"slow": 0.2, "fast": 0.6}

        self.tip_name = "right_gripper_tip"

        self.right_gripper = intera_interface.gripper.Gripper("right_gripper")


        ### BOARD COORDINATES CONFIGURATIONS ###

        self.a1_xy = np.array([0.365, -0.009]) # TODO: hardcode a1's x and y coordinates
        self.a2_xy = np.array([0.357, 0.049]) # TODO: hardcode a2's x and y coordinates
        self.b1_xy = np.array([0.422, 0.004]) # TODO: hardcode b1's x and y coordinates

        self.right_increment = self.a2_xy - self.a1_xy
        self.down_increment = self.b1_xy - self.a1_xy

        self.z = {"hover":-0.141, "grab":-0.167, "reset":0.231, "drop":0.15, "gripper": -0.1}

        self.running = True

        ### BOARD PROCESSING CONFIGURATIONS

        self.tile_to_piece = {}
        for alpha in range(ord('a'), ord('i')):
            for num in range(1, 9):
                self.tile_to_piece[f"{chr(alpha)}{num}"] = ""
        
        self.board_updater = BoardUpdater()
        
        self.drop_off = (0.722, -0.013)  # piece drop off spot (after taking)

    def run(self):
        self.calibrate()

        while self.running:
            if input("Press Enter to Calculate Board State. Press q to quit. \n") == "q":
                self.running = False
            else:
                print("Calculating board state...")

                rospy.wait_for_service('screenshot_service')

                # try:
                get_screenshot = rospy.ServiceProxy('screenshot_service', Screenshot)
                imgmsg = get_screenshot().img

                self.update_board(imgmsg)

                fen_string = self.get_fen_string()
                header = {"fen": fen_string}

                print("Making a request to Chess API...")

                resp = requests.post("https://chess-api.com/v1", json=header)
                resp.raise_for_status()

                data = json.loads(resp.text)

                from_tile = data["from"]
                to_tile = data["to"]
                capture = data["isCapture"] #boolean of capturing
                
                
                if capture:
                    if input(f"Begin picking up tile at {to_tile}? (y/n)") == "y":
                        self.pick_up_tile(to_tile)
                    else:
                        raise Exception("Aborting process")
                    
                    if input(f"Begin placing tile at drop off point? (y/n)") == "y":
                        self.move_to_coord(self.drop_off, self.z['drop'])
                        self.right_gripper.open()
                        rospy.sleep(2.0)
                        
                    else:
                        raise Exception("Aborting process")
                    
                if input(f"Begin picking up tile at {from_tile}? (y/n)") == "y":
                    self.pick_up_tile(from_tile)
                else:
                    raise Exception("Aborting process")
                
                if input(f"Begin placing tile at {to_tile}? (y/n)") == "y":
                    self.pick_up_tile(to_tile)
                else:
                    raise Exception("Aborting process")

                # except Exception as e:
                #     print(e)

    def calibrate(self):
        # Tuck the Sawyer Arm
        if input("Would you like to tuck the arm? (y/n) \n") == "y":
            self.tuck()

        if input("Would you like to calibrate the gripper? (y/n) \n") == "y":
            print('Calibrating gripper...')
            self.right_gripper.calibrate()
            rospy.sleep(2.0)

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

    def recover(self):
        print("Recovering Sawyer Robot...")

        self.commander.set_max_velocity_scaling_factor(self.arm_speeds["slow"])
        rospy.sleep(1.0)

        self.limb.move_to_joint_positions(self.standard_tuck_angles)

    def arm_to_tile(self, tile_name):
        row = tile_name[0]  # alphabet
        col = tile_name[1]  # number

        num_down = ord(row) - ord('a')
        num_right = int(col) - 1

        target_xy = self.a1_xy + self.down_increment * num_down + self.right_increment * num_right

        for height in ["reset", "drop", "gripper"]:
            self.move_to_coord(target_xy, self.z[height])

    def pick_up_tile(self, tile_name):
        row = tile_name[0]  # alphabet
        col = tile_name[1]  # number

        num_down = ord(row) - ord('a')
        num_right = int(col) - 1

        target_xy = self.a1_xy + self.down_increment * num_down + self.right_increment * num_right

        for height in ["reset", "drop", "gripper"]:
            self.move_to_coord(target_xy, self.z[height])

        self.right_gripper.close()
        rospy.sleep(2.0)

        self.recover()
    
    def place_tile(self, tile_name):
        row = tile_name[0]  # alphabet
        col = tile_name[1]  # number

        num_down = ord(row) - ord('a')
        num_right = int(col) - 1

        target_xy = self.a1_xy + self.down_increment * num_down + self.right_increment * num_right

        for height in ["reset", "drop", "gripper"]:
            self.move_to_coord(target_xy, self.z[height])
        
        self.right_gripper.open()
        rospy.sleep(2.0)

        self.recover()

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
            if input("IK solution not found. Would you like to tuck the arm? (y/n)") == "y":
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
    def update_board(self, imgmsg):
        tile_to_coords = self.board_updater.get_tile_coords(imgmsg)
        piece_to_coords = self.board_updater.get_piece_coords(imgmsg)

        curr_map = {}

        for elem in piece_to_coords:
            piece_name = elem["piece"]
            piece_coords = elem["location"]

            for tile_name in tile_to_coords:
                tile_coords = tile_to_coords[tile_name]

                bottom_left_x = tile_coords[0][0]
                bottom_left_y = tile_coords[0][1]
                top_right_x = tile_coords[1][0]
                top_right_y = tile_coords[1][1]

                if bottom_left_x > top_right_x:
                    bottom_left_x, top_right_x = top_right_x, bottom_left_x
                if bottom_left_y > top_right_y:
                    bottom_left_y, top_right_y = top_right_y, bottom_left_y

                if bottom_left_x <= piece_coords[0] <= top_right_x and bottom_left_y <= piece_coords[1] <= top_right_y:
                    curr_map[tile_name] = piece_name
                    break
        
        for key in self.tile_to_piece.keys():
            if key in curr_map:
                self.tile_to_piece[key] = curr_map[key]
            else:
                self.tile_to_piece[key] = ""

    def get_fen_string(self):
        fen_string = ""
        index = 0
        conseq_empty_space = 0
        for num in range(8, 0, -1):
            for alpha in range(ord('a'), ord('i')):
                tile_name = f"{chr(alpha)}{num}"

                if index == 8:
                    index = 0
                    if conseq_empty_space > 0:
                        fen_string += str(conseq_empty_space)
                        conseq_empty_space = 0
                    fen_string += "/"

                piece_name = self.tile_to_piece[tile_name]

                if piece_name == "":
                    conseq_empty_space += 1
                else:
                    if conseq_empty_space > 0:
                        fen_string += str(conseq_empty_space)
                        conseq_empty_space = 0
                    fen_string += piece_name

                index += 1

        if conseq_empty_space > 0:
            fen_string += str(conseq_empty_space)
        
        return fen_string + " b - - 0 1"








if __name__ == "__main__":
    cli = CLI()

    cli.run()