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
            # Look up the transform from the original frame
            (trans, rot) = self.tf_listener.lookupTransform('/world', self.original_frame, rospy.Time(0))

            # Publish the transform with a new parent frame
            self.tf_broadcaster.sendTransform(
                trans,
                rot,
                rospy.Time.now(),
                self.new_frame,
                self.new_parent_frame
            )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn(f"Could not find transform from /world to {self.original_frame}")

def main():
    rospy.init_node('tf_republisher')

    # Parameters for Sawyer camera
    sawyer_republisher = TFRepublisher('ar_marker_0', 'sawyer_marker_0', 'right_hand_camera')

    # Parameters for Logitech camera
    logitech_republisher = TFRepublisher('ar_marker_0', 'logitech_marker_0', 'logitech_c615')

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
