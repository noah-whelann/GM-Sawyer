#!/usr/bin/env python

import rospy
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

#find where one corner is on the board in terms of pixels
#transform that into coordinates relative to the webcam
#find a transformation from the webcam to the base of the robot
#move the arm to that coordinate

class CameraTest:
    def __init__(self):
        rospy.init_node('camera_test', anonymous=True)

        # self.web_cam_sub = rospy.Subscriber("/logitech_c615/camera_info", Image)

        self.coord_pub = rospy.Subscriber("corner_coords", Point, self.transform_callback)

        self.webcam_coords = rospy.Publisher("webcam_coords", Point, queue_size=10)

        print("hi")
        rospy.spin()

    def transform_callback(self, msg):

        new_coords = self.process_coords(msg)

        self.webcam_coords.publish(new_coords)

    def process_coords(self, coords):
        intrinsic_matrix = [997.1410709758351, 0.0, 620.2712277820584,
                            0.0, 1000.62021150938, 401.6358113787741,
                            0.0, 0.0, 1.0]
        
        f_x  = intrinsic_matrix[0]
        f_y = intrinsic_matrix[4]
        c_x = intrinsic_matrix[2]
        c_y = intrinsic_matrix[5]

        u = coords.x
        v = coords.y

        x_cam = (u - c_x) / f_x
        y_cam = (v - c_y) / f_y
        z_cam = 1.0

        transformed_point = Point()
        transformed_point.x = x_cam
        transformed_point.y = y_cam
        transformed_point.z = z_cam

        print(transformed_point)

        return transformed_point


if __name__ == "__main__":
    CameraTest()