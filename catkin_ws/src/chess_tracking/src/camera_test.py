#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

#find where one corner is on the board in terms of pixels
#transform that into coordinates relative to the webcam
#find a transformation from the webcam to the base of the robot
#move the arm to that coordinate

class CameraTest:
    def __init__(self):
        rospy.init_node('camera_test', anonymous=True)

        self.web_cam_sub = rospy.Subscriber("/logitech_c920/image_raw", Image, self.image_callback)

        self.image_pub = rospy.Publisher("processed_cam", Image, queue_size=10)

        self.coord_pub = rospy.Publisher("corner_coords", Point, queue_size=10)

        print("hi")
        rospy.spin()

        rospy.init_node('robot_camera_test', anonymous=True)

        # self.subscriber = rospy.Subscriber("/io/internal_camera/right_hand_camera/image_raw", Image, self.test_arm_camera)
        self.publisher = rospy.Publisher("processed_cam", Image, queue_size=10)

        # rospy.spin()
    

    def image_callback(self, msg):
        print("callback")

        processed_img, coords = self.process_board(msg)

        self.image_pub.publish(processed_img)


        coord_point = Point()
        coord_point.x = coords[0]
        coord_point.y = coords[1]
        
        self.coord_pub.publish(coord_point)

    def process_board(self, img):

        bridge = CvBridge()

        image = bridge.imgmsg_to_cv2(img, "bgr8")

        # Convert the image from BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(hsv, (7,7),None)

        if ret:
            corners = corners.squeeze()

            all_corners = np.empty((0, 2))

            # top left corner
            corner = corners[0] + corners[0] - corners[8]
            cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
<<<<<<< HEAD
=======
            all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)
>>>>>>> 22e1fb0ed08a842c4ae0e3dc4b60f58255fbb9db

            # top right corner
            corner = corners[6] + corners[6] - corners[12]
            cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
<<<<<<< HEAD
=======
            all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)
>>>>>>> 22e1fb0ed08a842c4ae0e3dc4b60f58255fbb9db

            # bottom left corner
            corner = corners[42] + corners[42] - corners[36]
            cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
<<<<<<< HEAD
=======
            all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)
>>>>>>> 22e1fb0ed08a842c4ae0e3dc4b60f58255fbb9db
            
            # bottom right corner
            corner = corners[48] + corners[48] - corners[40]
            cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
<<<<<<< HEAD
=======
            all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)
>>>>>>> 22e1fb0ed08a842c4ae0e3dc4b60f58255fbb9db

            for i in range(49):
                color = (0, 255, 0)
                if i // 7 == 0: # first row
<<<<<<< HEAD
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
=======
                    corner = 2 * corners[i] - corners[i+7]
                    cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
                    all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)
                
                elif i // 7 == 6:
                    corner = 2 * corners[i] - corners[i-7]
                    cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
                    all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)
                
                if i % 7 == 0: # left edge
                    corner = 2 * corners[i] - corners[i+1]
                    cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
                    all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)

                if i % 7 == 6: # right edge
                    corner = 2 * corners[i] - corners[i-1]
                    cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
                    all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)

                corner = corners[i]
                cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
                all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)

            all_corners = all_corners.tolist()
>>>>>>> 22e1fb0ed08a842c4ae0e3dc4b60f58255fbb9db

        else:
            print("NOT FOUND")
            corner = [0, 0]
        
        return bridge.cv2_to_imgmsg(image), corner


    
    def test_new(self, img):

        # Load the image
        image = cv2.imread('IMG_7381.png')

        # Convert the image from BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        _, binary_image = cv2.threshold(hsv, 127, 255, cv2.THRESH_BINARY)

        ret, corners = cv2.findChessboardCorners(hsv, (7,7),None)

        # print(np.shape(corners))
        corners = corners.squeeze()
        # print(corners)

        ### CALIBRATION MEASURES ###
        # # average rate of change
        # horizontal_incs = [np.array([0.0, 0.0]) for _ in range(7)]
        # vertical_incs = [np.array([0.0, 0.0]) for _ in range(7)]
        # diagonal_incs = [np.array([0.0, 0.0]), np.array([0.0, 0.0])]

        # for i in range(49):
        #     corner = corners[i]
        #     if i % 7 >= 2: # horizontal
        #         horizontal_incs[i//7] += (corners[i] - corners[i - 1]) - (corners[i - 1] - corners[i - 2])
        #     if i >= 14: # vertical
        #         vertical_incs[i % 7] += (corners[i] - corners[i - 7]) - (corners[i - 7] - corners[i - 14])
        #     if i % 8 == 0 and i >= 16: # left diagonal
        #         diagonal_incs[0] += (corners[i] - corners[i - 8]) - (corners[i-8] - corners[i - 16])
        #     if i % 6 == 0 and 18 <= i <= 42: # left diagonal
        #         diagonal_incs[1] += (corners[i] - corners[i - 6]) - (corners[i-6] - corners[i - 12])
        # horizontal_incs = np.divide(horizontal_incs, 5)
        # vertical_incs = np.divide(vertical_incs, 5)
        # diagonal_incs = np.divide(diagonal_incs, 5)

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
                bottom_row_x = CameraTestint(2 * corners[i][0] - corners[i-7][0])
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


            # print(corner, type(corner))
            x, y = int(corners[i][0]), int(corners[i][1])
            cv2.circle(image, (x, y), 30, color, 4)  # Draw the circle
            cv2.putText(image, f"{i}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 3, color, 2)
            cv2.putText(image, f"X: {x} Y: {y}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

        print(x, y)

    def test_arm_camera(self, msg):
        # distortion_matrix = np.array([-0.436077, 0.251043, 0.001148, -0.000265, -0.097807])
        # camera_matrix = np.array([616.505981, 0.0, 378.118988, 0.0, 615.140991, 225.830994, 0.0, 0.0, 1.0])

        distortion_matrix = np.array([-0.436077, 0.251043, 0.001148, -0.000265, -0.097807], dtype=np.float32) #for the wrist
        camera_matrix = np.array([616.505981, 0.0, 378.118988,
                                0.0, 615.140991, 225.830994,
                                0.0, 0.0,        1.0], dtype=np.float32).reshape((3, 3))

        bridge = CvBridge()

        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        output = cv2.undistort(src=cv_image, cameraMatrix=camera_matrix, distCoeffs=distortion_matrix)

        ret, corners = cv2.findChessboardCorners(output, (7,7),None)

        if ret:
            corners = corners.squeeze()

            # top left corner
            corner = corners[16]
            cv2.circle(output, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
            print(corner)

            coord_point = Point()
            coord_point.x = corner[0]
            coord_point.y = corner[1]
            self.coord_pub.publish(coord_point)
        else:
            print("NOT FOUND")
            corner = [0, 0]

        output_msg = bridge.cv2_to_imgmsg(output, encoding="bgr8")


        self.publisher.publish(output_msg)


if __name__ == "__main__":
    CameraTest()