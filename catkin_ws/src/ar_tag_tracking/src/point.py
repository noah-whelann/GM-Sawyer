#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

class PointVisualizer:
    def __init__(self):
        rospy.init_node('point_visualizer', anonymous=True)

        # Publisher to publish markers for visualization
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

        # Subscriber to listen to clicked points
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_callback)

        # Marker ID counter to make each point unique
        self.marker_id = 0

    def clicked_point_callback(self, msg):
        # Create a marker for the clicked point
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id  # Same frame as the clicked point
        marker.header.stamp = rospy.Time.now()
        marker.ns = "clicked_points"
        marker.id = self.marker_id  # Unique ID for each point
        marker.type = Marker.SPHERE  # Sphere to mark the point
        marker.action = Marker.ADD
        marker.pose.position = msg.point
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02  # Size of the sphere
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.r = 1.0  # Red color
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque

        # Publish the marker
        self.marker_pub.publish(marker)

        # Increment the marker ID for the next point
        self.marker_id += 1

if __name__ == '__main__':
    try:
        PointVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
