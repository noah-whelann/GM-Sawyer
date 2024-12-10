#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped
import tf2_ros
import tf2_geometry_msgs


class CameraTest:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('camera_test', anonymous=True)

        # Subscribe to the corner coordinates topic
        self.coord_sub = rospy.Subscriber("corner_coords", Point, self.transform_callback)

        # Publisher for transformed coordinates relative to the right_gripper_tip
        self.webcam_coords_pub = rospy.Publisher("webcam_coords", Point, queue_size=10)

        # TF2 buffer and listener for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("CameraTest node initialized and ready.")
        rospy.spin()

    def transform_callback(self, msg):
        # Process the coordinates to get the point in the camera frame
        camera_point = self.process_coords(msg)

        try:
            # Transform the point to the right_gripper_tip frame
            gripper_point = self.transform_to_base(camera_point)

            # Publish the transformed point
            transformed_point = Point(
                x=gripper_point.point.x,
                y=gripper_point.point.y,
                z=gripper_point.point.z
            )
            self.webcam_coords_pub.publish(transformed_point)

            rospy.loginfo("Transformed point published: %s", transformed_point)

        except Exception as e:
            rospy.logerr("Failed to transform point: %s", e)

    def process_coords(self, coords):
        intrinsic_matrix = [616.505981, 0.0, 378.118988,
                            0.0, 615.140991, 225.830994,
                            0.0, 0.0, 1.0]

        f_x = intrinsic_matrix[0]
        f_y = intrinsic_matrix[4]
        c_x = intrinsic_matrix[2]
        c_y = intrinsic_matrix[5]

        # Convert pixel coordinates to camera coordinates
        u, v = coords.x, coords.y
        x_cam = (u - c_x) / f_x
        y_cam = (v - c_y) / f_y
        z_cam = 1.0  # Assume a normalized depth value for simplicity

        point_in_camera = PointStamped()
        point_in_camera.header.frame_id = "right_hand_camera"
        point_in_camera.header.stamp = rospy.Time.now()
        point_in_camera.point.x = x_cam
        point_in_camera.point.y = y_cam
        point_in_camera.point.z = z_cam

        rospy.loginfo("Point in camera frame: x=%.3f, y=%.3f, z=%.3f",
                      x_cam, y_cam, z_cam)

        return point_in_camera

    def transform_to_base(self, point_in_camera):
        # Wait for the transform to be available and perform the transformation
        transform = self.tf_buffer.lookup_transform(
            "base",      # Target frame
            point_in_camera.header.frame_id,  # Source frame (camera frame)
            rospy.Time(0),            # Use the latest available transform
            rospy.Duration(1.0)       # Timeout duration
        )

        # Apply the transform to the point
        point_in_gripper = tf2_geometry_msgs.do_transform_point(point_in_camera, transform)
        rospy.loginfo("Point in base frame: x=%.3f, y=%.3f, z=%.3f",
                      point_in_gripper.point.x,
                      point_in_gripper.point.y,
                      point_in_gripper.point.z)

        return point_in_gripper


if __name__ == "__main__":
    try:
        CameraTest()
    except rospy.ROSInterruptException:
        rospy.loginfo("CameraTest node terminated.")
