#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PointStamped
import tf2_ros
import tf2_geometry_msgs

class TransformCoordinates:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('transform_coordinates', anonymous=True)

        # Subscriber for input coordinates (x, y)
        self.coord_sub = rospy.Subscriber("input_coords", Point, self.transform_callback)

        # Publisher for transformed coordinates in the base frame
        self.transformed_coords_pub = rospy.Publisher("transformed_coords", Point, queue_size=10) # 

        # TF2 buffer and listener for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("TransformCoordinates node initialized.")
        rospy.spin()

    def transform_callback(self, msg):
        point_in_camera = PointStamped()
        point_in_camera.header.frame_id = "logitech_c920" #change this to the frame of the webcam
        point_in_camera.header.stamp = rospy.Time.now()
        point_in_camera.point.x = msg.x
        point_in_camera.point.y = msg.y
        point_in_camera.point.z = 1.0  # Assume a normalized depth value for simplicity

        try:
            # Transform the point to the base frame
            transformed_point = self.transform_to_base(point_in_camera)

            # Publish the transformed point
            point = Point(
                x=transformed_point.point.x,
                y=transformed_point.point.y,
                z=transformed_point.point.z
            )
            self.transformed_coords_pub.publish(point)

            rospy.loginfo("Transformed coordinates published: x=%.3f, y=%.3f, z=%.3f", 
                          point.x, point.y, point.z)

        except Exception as e:
            rospy.logerr("Failed to transform point: %s", e)

    def transform_to_base(self, point_in_camera):
        # Wait for the transform and apply it to the point
        transform = self.tf_buffer.lookup_transform(
            "base",                  # Target frame
            point_in_camera.header.frame_id,  # Source frame (camera frame)
            rospy.Time(0),            # Use the latest available transform
            rospy.Duration(1.0)       # Timeout duration
        )

        # apply the transform to the point 
        point_in_base = tf2_geometry_msgs.do_transform_point(point_in_camera, transform)
        rospy.loginfo("Point in base frame: x=%.3f, y=%.3f, z=%.3f",
                      point_in_base.point.x,
                      point_in_base.point.y,
                      point_in_base.point.z)
        return point_in_base

if __name__ == "__main__":
    try:
        TransformCoordinates()
    except rospy.ROSInterruptException:
        rospy.loginfo("TransformCoordinates node terminated.")
