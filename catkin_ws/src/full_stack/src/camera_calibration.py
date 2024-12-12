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

        self.web_cam_sub = rospy.Subscriber("/logitech_c920/image_raw", Image, self.circle_image)

        self.image_pub = rospy.Publisher("calibrate_cam", Image, queue_size=10)

        rospy.spin()
    

    def circle_image(self, msg):

        bridge = CvBridge()

        image = bridge.imgmsg_to_cv2(msg, "bgr8")

        locs = np.array([[1377, 5], [1355, 1058], [299, 33], [366,1053]], dtype=np.int32)

        for corner in locs:
            cv2.circle(image, (int(corner[0]), int(corner[1])), 20, (0, 0, 255), 4)

        locs = locs.reshape((-1, 1, 2))
        print(type(locs))

        cv2.polylines(image, [locs], isClosed=True, color=(0, 0, 255), thickness=3)

        return_msg = bridge.cv2_to_imgmsg(image)

        self.image_pub.publish(return_msg)

    def image_callback(self, msg):
        print("callback")

        bridge = CvBridge()

        image = bridge.imgmsg_to_cv2(msg, "bgr8")

        ret, corners = cv2.findChessboardCorners(image, (7, 7), None)

        corners = corners.squeeze()

        output = []

        # top left corner
        corner = corners[0] + corners[0] - corners[8]
        output.append(corner)
        # Draw the circle
        cv2.circle(image, (int(corner[0]), int(
            corner[1])), 20, (0, 0, 255), 4)

        # top right corner
        corner = corners[6] + corners[6] - corners[12]
        output.append(corner)
        # Draw the circle
        cv2.circle(image, (int(corner[0]), int(
            corner[1])), 20, (0, 0, 255), 4)

        # bottom left corner
        corner = corners[42] + corners[42] - corners[36]
        output.append(corner)
        # Draw the circle
        cv2.circle(image, (int(corner[0]), int(
            corner[1])), 20, (0, 0, 255), 4)

        # bottom right corner
        corner = corners[48] + corners[48] - corners[40]
        output.append(corner)
        # Draw the circle
        cv2.circle(image, (int(corner[0]), int(
            corner[1])), 20, (0, 0, 255), 4)

        print(output)
        # cv2.polylines(image, ##)

        return_msg = bridge.cv2_to_imgmsg(image)

        self.image_pub.publish(return_msg)

if __name__ == "__main__":
    CameraTest()