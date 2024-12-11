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

        self.camera_transform_sub = rospy.Subscriber("transformed_coordinates", Point, self.tile_coords_callback)

        self.image_pub = rospy.Publisher("processed_cam", Image, queue_size=10)

        self.coord_pub = rospy.Publisher("tile_coordinates", StringAndFloatsGrid, queue_size=10)

        print("hi")
        rospy.spin()
    
    def tile_coords_callback(self, msg):

        pass


    def image_callback(self, msg):
        print("callback")

        processed_img, corners = self.process_board(msg)

        self.image_pub.publish(processed_img)

        #A1 [0.439, -0.061, -0.156]
        #H1 [0.835, -0.005, -0.161]
        #H8 [0.775, 0.391, -0.146]
        #A8 [0.393, 0.336, -0.136]

        # grid_locations = transform_corners(corners)

        print(corners)
        
        # self.coord_pub.publish(grid_locations)

    # def transform_corners(self, corners):

        # return_grid = StringAndFloatsGrid()

        # grid_letters = ["A", "B", "C", "D", "E", "F", "G", "H"]
        # grid_numbers  = [1, 2, 3, 4, 5, 6, 7, 8]
        
        # print("First corner: ", )

        # for i in range(8):
        #     for j in range(8):
        #         return_grid[i][j].name = grid_letters[i] + str(grid_numbers[j]) # assign a1 -> a8 and onwards for the entire chess board
        #         return_grid[i][j].x = 
        #         return_grid[i][j].y
        #         return_grid[i][j].z
                

                

        # return return_grid

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

'''
49 coords 
[[ 647.5286   137.4259 ]
 [ 771.8964   142.6874 ]
 [ 898.0685   147.61621]
 [1020.99976  149.65016]
 [1146.3932   156.35721]
 [1273.0488   160.55772]
 [1401.0702   163.33093]
 [ 642.6213   263.86096]
 [ 768.0286   269.53705]
 [ 891.7468   274.9615 ]
 [1016.1571   278.89566]
 [1140.5575   286.44254]
 [1266.2832   288.0092 ]
 [1392.0369   291.48767]
 [ 639.18896  389.41196]
 [ 762.5676   394.7501 ]
 [ 886.43756  400.86267]
 [1010.54614  405.98755]
 [1135.0403   410.0612 ]
 [1258.775    413.6757 ]
 [1386.0175   419.0489 ]
 [ 635.1926   513.32666]
 [ 758.72437  518.78015]
 [ 881.63495  524.5155 ]
 [1004.4566   530.75446]
 [1129.5245   535.415  ]
 [1254.2351   539.6399 ]
 [1377.8043   543.4602 ]
 [ 631.85284  635.8606 ]
 [ 754.0357   641.80707]
 [ 877.0244   646.96313]
 [1000.1061   652.81885]
 [1123.6398   657.7821 ]
 [1247.727    663.02563]
 [1372.0232   668.24347]
 [ 629.4015   758.1759 ]
 [ 750.27527  763.25104]
 [ 872.5131   768.7219 ]
 [ 994.85443  774.24976]
 [1118.0385   780.0498 ]
 [1240.9467   785.4582 ]
 [1365.5676   790.9377 ]
 [ 625.21375  877.5205 ]
 [ 746.4008   884.1761 ]
 [ 868.2948   889.57733]
 [ 989.49615  894.57495]
 [1111.9459   900.708  ]
 [1235.5393   906.8279 ]
 [1359.1764   913.14154]]

'''