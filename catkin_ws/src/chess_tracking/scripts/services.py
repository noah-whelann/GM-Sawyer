#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image
from chess_tracking.srv import Screenshot

class Services:
    def __init__(self):
        self.camera_msg = None

        rospy.init_node("custom_services", anonymous=True)

        self.web_cam_sub = rospy.Subscriber("/logitech_c920/image_raw", Image, self.camera_callback)

        self.screenshot = rospy.Service("screenshot_service", Screenshot, self.screenshot_callback)

        rospy.spin()

    def camera_callback(self, msg):
        self.camera_msg = msg

    def screenshot_callback(self, inputs):
        return self.camera_msg

if __name__ == "__main__":
    Services()