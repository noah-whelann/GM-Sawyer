#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

class CameraTest:
    def __init__(self):
        rospy.init_node('camera_test', anonymous=True)

        self.web_cam_sub = rospy.Subscriber("/logitech_c615/image_raw", Image, self.image_callback)

        self.image_pub = rospy.Publisher("processed_cam", Image, queue_size=10)

        rospy.spin()
    

    def image_callback(self, msg):
        processed_img = self.process_image(msg)

        self.image_pub.publish(processed_img)

    def process_image(self, img):
        
        bridge = CvBridge()

        image = bridge.imgmsg_to_cv2(img, "bgr8")

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = cv2.bitwise_or(mask1, mask2)

        result = cv2.bitwise_and(image, image, mask=mask)

        circles = cv2.HoughCircles(
            mask,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=100,
            param1=50,
            param2=20,
            minRadius=15,
            maxRadius=150
        )

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")

            for (x, y, r) in circles:
                cv2.circle(image, (x, y), r, (0, 255, 0), 4)

        return bridge.cv2_to_imgmsg(image)


if __name__ == "__main__":
    CameraTest()