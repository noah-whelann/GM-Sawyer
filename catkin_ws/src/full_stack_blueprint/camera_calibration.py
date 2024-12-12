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

        self.image_pub = rospy.Publisher("calibrate_cam", Image, queue_size=10)

        rospy.spin()
    

    def image_callback(self, msg):
        print("callback")

        bridge = CvBridge()

        image = bridge.imgmsg_to_cv2(img, "bgr8")

        # cv2.polylines(image, ##)

        return_msg = bridge.cv2_to_imgmsg(image)

        self.image_pub.publish(return_msg)

if __name__ == "__main__":
    CameraTest()