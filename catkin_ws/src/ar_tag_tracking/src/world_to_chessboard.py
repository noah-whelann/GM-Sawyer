#!/usr/bin/env python

import rospy
import tf


def calculate_world_to_chessboard():
    rospy.init_node('transform_listener')

    listener = tf.TransformListener()

    try:
        rospy.loginfo("Waiting for transforms...")

        # Wait for the full transform to become available
        listener.waitForTransform(
            'world', 'chessboard', rospy.Time(), rospy.Duration(4.0))

        # Get the transform
        (trans, rot) = listener.lookupTransform(
            'world', 'chessboard', rospy.Time(0))
        rospy.loginfo(f"World to Chessboard: Translation={
                      trans}, Rotation={rot}")

    except tf.Exception as e:
        rospy.logerr(f"Error calculating transform: {e}")


if __name__ == "__main__":
    calculate_world_to_chessboard()
