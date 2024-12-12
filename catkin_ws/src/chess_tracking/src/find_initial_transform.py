#!/usr/bin/env python

import rospy
import tf
from tf.transformations import quaternion_multiply, quaternion_inverse, translation_matrix, quaternion_matrix, concatenate_matrices, translation_from_matrix, quaternion_from_matrix


# will publish static transform on tf tree with parent_frame->child_frame
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

# returns translation and rotation from parent_frame to child_frame -> trans_ab, rot_ab gives child_frame in terms of parent_frame


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


# given trans_ab, rot_ab, trans_bc, rot_bc -> returns trans_ac and rot_ac
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


def calculate_full_camera_transform():
    rospy.init_node("ar_tag_transformer_for_cam")
    listener = tf.TransformListener()

    rospy.loginfo("move ar tag in view of webcam")
    input("press enter when in view")

    # If ar_marker_0 is in view of the c920 -> it will be published on the TF tree
    # grab the ar marker, relative to the webcam
    trans_ar_c920, rot_ar_c920 = get_transform(
        listener, "ar_marker_0", "logitech_c920")
    if trans_ar_c920 is None:
        rospy.logerr("Failed to get AR tag relative to Logitech C920.")
        return

    rospy.loginfo(
        "move ar tag in view of the wrist camera (right_hand_camera).")
    input("press enter when in view")

    # do the same here, grabbing the ar marker relative to the right hand camera
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

    # base->wrist->ar -> ar relative to the base
    trans_base_ar, rot_base_ar = combine_transforms(
        trans_base_wrist, rot_base_wrist, trans_wrist_ar, rot_wrist_ar)
    rospy.loginfo(f"AR tag transform relative to robot base: {
                  trans_base_ar}, {rot_base_ar}")

    rospy.loginfo(
        "Calculating Logitech C920 transform relative to robot base...")

    # base->ar->c920

    trans_base_c920, rot_base_c920 = combine_transforms(
        trans_base_ar, rot_base_ar, trans_ar_c920, rot_ar_c920)
    rospy.loginfo(f"Logitech relative to robot base: {
                  trans_base_c920}, {rot_base_c920}")

    # transform from base->logitech will be published as static transform
    publish_static_transform(
        trans_base_c920, rot_base_c920, "logitech_c920_fixed", "base")

    publish_static_transform(trans_base_ar, rot_base_ar,
                             "ar_marker_0_static", "base")

    rospy.loginfo("Transforms done and published")


if __name__ == "__main__":
    try:
        calculate_full_camera_transform()
    except rospy.ROSInterruptException:
        pass
