#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from chess_tracking.msg import StringAndFloatsGrid


#find where one corner is on the board in terms of pixels
#transform that into coordinates relative to the webcam
#find a transformation from the webcam to the base of the robot
#move the arm to that coordinate

class ChessTileCoordinates:
    def __init__(self):
        rospy.init_node('chess_tile_coordinates', anonymous=True)

        self.web_cam_sub = rospy.Subscriber("/logitech_c920/image_raw", Image, self.image_callback)

        self.image_pub = rospy.Publisher("processed_cam", Image, queue_size=10)

        self.coord_pub = rospy.Publisher("tile_coordinates", StringAndFloatsGrid, queue_size=10)

        print("hi")
        rospy.spin()
    

    def image_callback(self, msg):
        print("callback")

        processed_img, corners = self.process_board(msg)

        self.image_pub.publish(processed_img)


        grid_locations = transform_corners(corners)
        print(len(corners))
        self.coord_pub.publish(grid_locations)

    def transform_corners(self, corners):

        grid_return = StringAndFloatsGrid()
        



    def process_board(self, img):

        bridge = CvBridge()

        image = bridge.imgmsg_to_cv2(img, "bgr8")

        # Convert the image from BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(hsv, (7,7),None)

        if ret:
            corners = corners.squeeze()

            # top left corner
            corner = corners[0] + corners[0] - corners[8]
            cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle

            # top right corner
            corner = corners[6] + corners[6] - corners[12]
            cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle

            # bottom left corner
            corner = corners[42] + corners[42] - corners[36]
            cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
            
            # bottom right corner
            corner = corners[48] + corners[48] - corners[40]
            cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle

            for i in range(49):
                color = (0, 255, 0)
                if i // 7 == 0: # first row
                    first_row_x = int(2 * corners[i][0] - corners[i+7][0])
                    first_row_y = int(2 * corners[i][1] - corners[i+7][1])
                    cv2.circle(image, (first_row_x, first_row_y), 30, color, 4)  # Draw the circle
                
                elif i // 7 == 6:
                    bottom_row_x = int(2 * corners[i][0] - corners[i-7][0])
                    bottom_row_y = int(2 * corners[i][1] - corners[i-7][1])
                    cv2.circle(image, (bottom_row_x, bottom_row_y), 30, color, 4)  # Draw the circle
                
                if i % 7 == 0: # left edge
                    left_col_x = int(2*corners[i][0] - corners[i+1][0])
                    left_col_y = int(2*corners[i][1] - corners[i+1][1])
                    cv2.circle(image, (left_col_x, left_col_y), 30, color, 4)  # Draw the circle

                if i % 7 == 6: # right edge
                    right_col_x = int(2*corners[i][0] - corners[i-1][0])
                    right_col_y = int(2*corners[i][1] - corners[i-1][1])
                    cv2.circle(image, (right_col_x, right_col_y), 30, color, 4)  # Draw the circle

                x, y = int(corners[i][0]), int(corners[i][1])
                cv2.circle(image, (x, y), 30, color, 4)  # Draw the circle

        else:
            print("NOT FOUND")
            corner = [0, 0]
        
        return bridge.cv2_to_imgmsg(image), corners

if __name__ == "__main__":
    ChessTileCoordinates()