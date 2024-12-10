#!/usr/bin/env python

import rospy
import numpy as np
import tf
from ar_track_alvar_msgs.msg import AlvarMarkers

class TFPublisher:
    def __init__(self):
        rospy.init_node('tf_publisher_logitech_to_base')

        # TF broadcaster and listener
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        # Subscribers for AR tag detection
        self.robot_cam_sub = rospy.Subscriber('/sawyer/ar_pose_marker', AlvarMarkers, self.robot_cam_callback)
        self.webcam_sub = rospy.Subscriber('/logitech/ar_pose_marker', AlvarMarkers, self.webcam_callback)

        # Store AR tag poses
        self.robot_cam_pose = None
        self.webcam_pose = None

        # Store the last known transform for persistent publishing
        self.T_logitech_to_base = None

    def robot_cam_callback(self, msg):
        # Process AR tag pose from the wrist camera
        if msg.markers:
            self.robot_cam_pose = self.get_pose_matrix(msg.markers[0].pose.pose)
        else:
            self.robot_cam_pose = None

    def webcam_callback(self, msg):
        # Process AR tag pose from the Logitech webcam
        if msg.markers:
            self.webcam_pose = self.get_pose_matrix(msg.markers[0].pose.pose)

            if self.robot_cam_pose is not None:
                try:
                    # Get transform from wrist to robot base
                    T_wrist_to_base = self.lookup_transform("base", "right_hand_camera")

                    # Compute transform from webcam to robot base
                    self.T_logitech_to_base = self.compute_transform(self.webcam_pose, self.robot_cam_pose, T_wrist_to_base)

                except tf.Exception as e:
                    rospy.logerr(f"Failed to lookup wrist-to-base transform: {e}")

        # Publish the last known transform
        if self.T_logitech_to_base is not None:
            self.publish_transform(self.T_logitech_to_base, "logitech_c615", "base")

    def compute_transform(self, T_webcam_to_tag, T_wrist_to_tag, T_wrist_to_base):
        """
        Compute the transform from the Logitech webcam to the robot base.
        T_webcam_to_tag: Pose of AR tag in the webcam frame.
        T_wrist_to_tag: Pose of AR tag in the wrist camera frame.
        T_wrist_to_base: Transform from wrist camera to robot base.
        """
        T_tag_to_wrist = np.linalg.inv(T_wrist_to_tag)
        T_webcam_to_base = np.dot(T_wrist_to_base, np.dot(T_tag_to_wrist, T_webcam_to_tag))
        return T_webcam_to_base

    def publish_transform(self, T, child_frame, parent_frame):
        """
        Publish the transform using tf.TransformBroadcaster.
        """
        translation = T[:3, 3]
        rotation = tf.transformations.quaternion_from_matrix(T)

        self.tf_broadcaster.sendTransform(
            translation,
            rotation,
            rospy.Time.now(),
            child_frame,
            parent_frame
        )

    def lookup_transform(self, target_frame, source_frame):
        """
        Look up a transform using tf.TransformListener and return it as a 4x4 matrix.
        """
        self.tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(1.0))
        (trans, rot) = self.tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        T = tf.transformations.quaternion_matrix(rot)
        T[:3, 3] = trans
        return T

    @staticmethod
    def get_pose_matrix(pose):
        """
        Convert geometry_msgs/Pose to a 4x4 transformation matrix.
        """
        translation = [pose.position.x, pose.position.y, pose.position.z]
        rotation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        T = tf.transformations.quaternion_matrix(rotation)
        T[:3, 3] = translation
        return T


if __name__ == '__main__':
    try:
        TFPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
