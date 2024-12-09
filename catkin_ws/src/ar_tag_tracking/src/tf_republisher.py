#!/usr/bin/env python

import rospy
import tf

class TFRepublisher:
    def __init__(self, original_frame, new_frame, new_parent_frame):
        self.original_frame = original_frame
        self.new_frame = new_frame
        self.new_parent_frame = new_parent_frame
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

    def republish(self):
        try:
            # Look up the transform from the original AR marker frame to /world
            (trans, rot) = self.tf_listener.lookupTransform('/world', self.original_frame, rospy.Time(0))

            # Transform the position into the new parent frame
            (new_trans, new_rot) = self.tf_listener.lookupTransform(self.new_parent_frame, self.original_frame, rospy.Time(0))

            # Republish the transform with the new parent frame
            self.tf_broadcaster.sendTransform(
                new_trans,
                new_rot,
                rospy.Time.now(),
                self.new_frame,
                self.new_parent_frame
            )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Could not transform {self.original_frame} to {self.new_parent_frame}: {e}")

def main():
    rospy.init_node('tf_republisher')

    # Republish sawyer marker relative to /right_hand_camera
    sawyer_republisher = TFRepublisher('ar_marker_1', 'sawyer_marker_1', 'right_hand_camera')

    # Republish logitech marker relative to /logitech_c615
    logitech_republisher = TFRepublisher('ar_marker_1', 'logitech_marker_1', 'logitech_c615')

    rate = rospy.Rate(10.0)  # 10 Hz
    while not rospy.is_shutdown():
        sawyer_republisher.republish()
        logitech_republisher.republish()
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
