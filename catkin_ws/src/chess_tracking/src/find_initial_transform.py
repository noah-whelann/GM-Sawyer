#!/usr/bin/env python

import rospy
import tf
from tf.transformations import quaternion_multiply, quaternion_inverse, translation_matrix, quaternion_matrix, concatenate_matrices, translation_from_matrix, quaternion_from_matrix


def publish_static_transform(translation, rotation, child_frame, parent_frame):
    static_broadcaster = tf.TransformBroadcaster()
    rospy.loginfo(f"Publishing static transform: {
                  child_frame} -> {parent_frame}")
    static_broadcaster.sendTransform(
        translation,
        rotation,
        rospy.Time.now(),
        child_frame,
        parent_frame
    )


#returns translation and rotation from parent_frame to child_frame -> trans_ab, rot_ab
def get_transform(listener, parent_frame, child_frame):
    try:
        listener.waitForTransform(
            parent_frame, child_frame, rospy.Time(), rospy.Duration(5.0))
        (trans, rot) = listener.lookupTransform(
            parent_frame, child_frame, rospy.Time(0))
        rospy.loginfo(f"Transform from {parent_frame} to {
                      child_frame}: Translation {trans}, Rotation {rot}")
        return trans, rot
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr(f"Error getting transform from {
                     parent_frame} to {child_frame}: {e}")
        return None, None


#given trans_ab, rot_ab, trans_bc, rot_bc -> returns trans_ac and rot_ac
def combine_transforms(parent_trans, parent_rot, child_trans, child_rot):
    # Convert to transformation matrices
    parent_matrix = concatenate_matrices(translation_matrix(
        parent_trans), quaternion_matrix(parent_rot))
    child_matrix = concatenate_matrices(translation_matrix(
        child_trans), quaternion_matrix(child_rot))

    # Combine transforms
    combined_matrix = concatenate_matrices(parent_matrix, child_matrix)

    # Extract translation and rotation
    combined_translation = translation_from_matrix(combined_matrix)
    combined_rotation = quaternion_from_matrix(combined_matrix)

    return combined_translation, combined_rotation



def interactive_calculate_camera_transform():
    rospy.init_node("ar_tag_transformer_for_cam")
    listener = tf.TransformListener()

    rospy.loginfo("move ar tag in view of webcam")
    input("press enter when in view")

    #If ar_marker_0 is in view of the c920 -> it will be published on the TF tree
    # grab the ar marker, relative to the webcam
    trans_c920_ar, rot_c920_ar = get_transform(
        listener, "logitech_c920", "ar_marker_0")
    if trans_c920_ar is None:
        rospy.logerr("Failed to get AR tag relative to Logitech C920.")
        return

    rospy.loginfo(
        "move ar tag in view of the wrist camera (right_hand_camera).")
    input("press enter when in view")

    #do the same here, grabbing the ar marker relative to the right hand camera
    trans_wrist_ar, rot_wrist_ar = get_transform(
        listener, "right_hand_camera", "ar_marker_0")
    if trans_wrist_ar is None:
        rospy.logerr("Failed to get AR tag relative to wrist camera.")
        return


    trans_base_wrist, rot_base_wrist = get_transform(
        listener, "robot_base", "right_hand_camera")
    if trans_base_wrist is None:
        rospy.logerr(
            "Failed to get wrist camera transform relative to robot base.")
        return

    trans_ar_base, rot_ar_base = combine_transforms(
        trans_base_wrist, rot_base_wrist, trans_wrist_ar, rot_wrist_ar)
    rospy.loginfo(f"AR tag transform relative to robot base: {
                  trans_ar_base}, {rot_ar_base}")

    rospy.loginfo(
        "Calculating Logitech C920 transform relative to robot base...")
    trans_c920_base, rot_c920_base = combine_transforms(
        trans_ar_base, rot_ar_base, [-t for t in trans_c920_ar], quaternion_inverse(rot_c920_ar))
    rospy.loginfo(f"Logitech C920 transform relative to robot base: {
                  trans_c920_base}, {rot_c920_base}")

    # Publish calculated static transforms
    publish_static_transform(trans_ar_base, rot_ar_base,
                             "ar_marker_0_static", "robot_base")
    publish_static_transform(
        trans_c920_base, rot_c920_base, "logitech_c920_static", "robot_base")

    rospy.loginfo("Transforms successfully calculated and published.")


if __name__ == "__main__":
    try:
        interactive_calculate_camera_transform()
    except rospy.ROSInterruptException:
        pass
