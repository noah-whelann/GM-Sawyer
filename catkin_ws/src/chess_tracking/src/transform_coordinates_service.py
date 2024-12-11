#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped


class TransformCoordinateService:
    def __init__(self):
        rospy.init_node('transform_coordinates_service', anonymous=True)

        self.tf_buffer() = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.service = rospy.Service(
            'transform_coordinates', Point, self.handle_transform_request)

    def handle_transform_request(self, req):
        # Create a PointStamped from the request
        point_in_camera = PointStamped()

        # will always transform from logitech frame
        point_in_camera.header.frame_id = "logitech_c920"
        point_in_camera.header.stamp = rospy.Time.now()
        point_in_camera.point.x = req.x
        point_in_camera.point.y = req.y
        point_in_camera.point.z = 1.0  # depth must be hardcoded when actually moving robot

        try:
            transformed_point = self.transform_to_base(point_in_camera)

            return TransformPointResponse(
                point=transformed_point.point
            )

        except Exception as e:
            rospy.logerr("Failed to transform point: %s", e)
            rospy.ServiceException("Failed to transform point: {}".format(e))

    def transform_to_base(self):
        transform = self.tf_buffer.lookup_transform(
            "base",                  # Target frame
            point_in_camera.header.frame_id,  # Source frame (camera frame)
            rospy.Time(0),            # Use the latest available transform
            rospy.Duration(1.0)       # Timeout duration
        )

        # Apply the transform to the point
        point_in_base = tf2_geometry_msgs.do_transform_point(
            point_in_camera, transform)
        rospy.loginfo("Point in base frame: x=%.3f, y=%.3f, z=%.3f",
                      point_in_base.point.x,
                      point_in_base.point.y,
                      point_in_base.point.z)
        return point_in_base


if __name__ == "__main__":
    try:
        TransformCoordinatesService()
    except rospy.ROSInterruptException:
        rospy.loginfo("TransformCoordinatesService terminated.")
