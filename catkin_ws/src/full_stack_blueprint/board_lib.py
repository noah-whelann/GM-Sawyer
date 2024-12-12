#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge
import numpy as np

def get_board_corners(imgmsg):
    bridge = CvBridge()

    image = bridge.imgmsg_to_cv2(imgmsg, "bgr8")

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