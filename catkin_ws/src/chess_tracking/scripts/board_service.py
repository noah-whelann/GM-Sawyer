#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String

from chess_tracking.srv import BoardString

import json


class BoardService:
    def __init__(self):
        self.camera_msg = None

        rospy.init_node("board_service", anonymous=True)

        self.web_cam_sub = rospy.Subscriber(
            "/logitech_c920/image_raw", Image, self.camera_callback)

        # self.debug_writer = rospy.Subscriber("/logitech_c920/image_raw", Image, self.debug_image)

        self.board_service = rospy.Service(
            "board_service", BoardString, self.board_callback)

        rospy.spin()

    def camera_callback(self, msg):
        self.camera_msg = msg

    def board_callback(self, huh):
        return ("YO")

        print(huh)
        all_corners = self.get_board_corners(self.camera_msg)

        all_corners.sort(key=lambda x: x[1])

        board_corners = []
        for i in range(9):
            board_corners.append([])
            for j in range(9):
                board_corners[i].append(all_corners[i * 9 + j])

        for row in board_corners:
            row.sort()

        mapper = {}
        for alpha in range(ord('a'), ord('i')):  # 8
            for num in range(1, 9):
                curr_key = f"{chr(alpha)}{num}"

                bottom_left_corner = board_corners[alpha -
                                                   ord('a') + 1][num - 1]
                top_right_corner = board_corners[alpha - ord('a')][num]

                mapper[curr_key] = ((bottom_left_corner), (top_right_corner))

        for key in mapper:
            print(key, mapper[key])

        return json.dumps(mapper)

    def get_board_corners(self, msg):

        bridge = CvBridge()

        image = bridge.imgmsg_to_cv2(msg, "bgr8")

        ret, corners = cv2.findChessboardCorners(image, (7, 7), None)

        if ret:
            corners = corners.squeeze()

            all_corners = np.empty((0, 2))

            # top left corner
            corner = corners[0] + corners[0] - corners[8]
            # Draw the circle
            cv2.circle(image, (int(corner[0]), int(
                corner[1])), 30, (0, 0, 255), 4)
            all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)

            # top right corner
            corner = corners[6] + corners[6] - corners[12]
            # Draw the circle
            cv2.circle(image, (int(corner[0]), int(
                corner[1])), 30, (0, 0, 255), 4)
            all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)

            # bottom left corner
            corner = corners[42] + corners[42] - corners[36]
            # Draw the circle
            cv2.circle(image, (int(corner[0]), int(
                corner[1])), 30, (0, 0, 255), 4)
            all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)

            # bottom right corner
            corner = corners[48] + corners[48] - corners[40]
            # Draw the circle
            cv2.circle(image, (int(corner[0]), int(
                corner[1])), 30, (0, 0, 255), 4)
            all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)

            for i in range(49):
                color = (0, 255, 0)
                if i // 7 == 0:  # first row
                    corner = 2 * corners[i] - corners[i+7]
                    # Draw the circle
                    cv2.circle(image, (int(corner[0]), int(
                        corner[1])), 30, (0, 0, 255), 4)
                    all_corners = np.append(
                        all_corners, corner.reshape(1, -1), axis=0)

                elif i // 7 == 6:
                    corner = 2 * corners[i] - corners[i-7]
                    # Draw the circle
                    cv2.circle(image, (int(corner[0]), int(
                        corner[1])), 30, (0, 0, 255), 4)
                    all_corners = np.append(
                        all_corners, corner.reshape(1, -1), axis=0)

                if i % 7 == 0:  # left edge
                    corner = 2 * corners[i] - corners[i+1]
                    # Draw the circle
                    cv2.circle(image, (int(corner[0]), int(
                        corner[1])), 30, (0, 0, 255), 4)
                    all_corners = np.append(
                        all_corners, corner.reshape(1, -1), axis=0)

                if i % 7 == 6:  # right edge
                    corner = 2 * corners[i] - corners[i-1]
                    # Draw the circle
                    cv2.circle(image, (int(corner[0]), int(
                        corner[1])), 30, (0, 0, 255), 4)
                    all_corners = np.append(
                        all_corners, corner.reshape(1, -1), axis=0)

                corner = corners[i]
                # Draw the circle
                cv2.circle(image, (int(corner[0]), int(
                    corner[1])), 30, (0, 0, 255), 4)
                all_corners = np.append(
                    all_corners, corner.reshape(1, -1), axis=0)

            np.round(all_corners, 2)

            all_corners = all_corners.tolist()

            return all_corners

        else:
            print("BOARD NOT FOUND")
            return "BOARD NOT FOUND"


if __name__ == "__main__":
    board = BoardService()
