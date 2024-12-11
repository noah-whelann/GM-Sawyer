#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped
from chess_tracking.srv import TransformPoint

# Takes in a Point(x, y, z) in pixel coordinates and returns a Point in the base frame

class TransformCoordinatesService:
    def __init__(self):
        rospy.init_node('transform_coordinates_service', anonymous=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.service = rospy.Service(
            'transform_coordinates', TransformPoint, self.handle_transform_request)

        rospy.spin()

    def handle_transform_request(self, req):
        """
        Handle the service request by:
        1. Transforming pixel coordinates to camera coordinates.
        2. Transforming camera coordinates to the base frame.
        """
        try:
            # Transform pixel coordinates to camera frame
            point_in_camera = self.transform_to_camera(req.input_point) # pass in a Point()

            # Transform the point from the camera frame to the base frame
            point_in_base = self.transform_to_base(point_in_camera)
            print(point_in_base)

            # Return the transformed point in the base frame
            return point_in_base

        except Exception as e:
            rospy.logerr("Error transforming point: %s", e)
            raise rospy.ServiceException("Error transforming point: {}".format(e))

    def transform_to_camera(self, pixel_coords):
        intrinsic_matrix = [997.1410709758351, 0.0, 620.2712277820584, 0.0, 1000.62021150938, 401.6358113787741, 0.0, 0.0, 1.0]


        f_x = intrinsic_matrix[0]
        f_y = intrinsic_matrix[4]
        c_x = intrinsic_matrix[2]
        c_y = intrinsic_matrix[5]

        # Extract pixel coordinates
        u, v = pixel_coords.x, pixel_coords.y

        # Convert pixel coordinates to normalized camera coordinates
        x_cam = (u - c_x) / f_x
        y_cam = (v - c_y) / f_y
        z_cam = 1.0  # Assume a normalized depth value for simplicity

        # Create a PointStamped for the camera frame
        point_in_camera = PointStamped()
        point_in_camera.header.frame_id = "logitech_c920"
        point_in_camera.header.stamp = rospy.Time.now()
        point_in_camera.point.x = x_cam
        point_in_camera.point.y = y_cam
        point_in_camera.point.z = z_cam

        rospy.loginfo("Point in camera frame: x=%.3f, y=%.3f, z=%.3f",
                      x_cam, y_cam, z_cam)

        return point_in_camera

    def transform_to_base(self, point_in_camera):
        """
        Transform a PointStamped from the camera frame to the base frame.
        """
        try:
            # Lookup the transform from the camera frame to the base frame
            transform = self.tf_buffer.lookup_transform(
                "base",                   # Target frame
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

            return point_in_base.point

        except Exception as e:
            print("exception")
            rospy.logerr("Failed to transform point: %s", e)
            raise e


if __name__ == "__main__":
    try:
        TransformCoordinatesService()
    except rospy.ROSInterruptException:
        rospy.loginfo("TransformCoordinatesService terminated.")
