#!/usr/bin/env python

import rospy
import tf

def calculate_webcam_to_chessboard():
    rospy.init_node('transform_listener')

    listener = tf.TransformListener()

    try:
        rospy.loginfo("Waiting for transforms...")

        # Transform from Logitech webcam to AR tag (ar_tag_0)
        listener.waitForTransform('logitech_c615', 'ar_tag_0', rospy.Time(), rospy.Duration(4.0))
        (webcam_to_ar_trans, webcam_to_ar_rot) = listener.lookupTransform('logitech_c615', 'ar_tag_0', rospy.Time(0))
        rospy.loginfo(f"Webcam to AR Tag: Translation={webcam_to_ar_trans}, Rotation={webcam_to_ar_rot}")

        # Transform from Logitech webcam to Chessboard
        listener.waitForTransform('logitech_c615', 'chessboard', rospy.Time(), rospy.Duration(4.0))
        (webcam_to_chessboard_trans, webcam_to_chessboard_rot) = listener.lookupTransform('logitech_c615', 'chessboard', rospy.Time(0))
        rospy.loginfo(f"Webcam to Chessboard: Translation={webcam_to_chessboard_trans}, Rotation={webcam_to_chessboard_rot}")

    except tf.Exception as e:
        rospy.logerr(f"Error calculating transforms: {e}")

if __name__ == "__main__":
    calculate_webcam_to_chessboard()
