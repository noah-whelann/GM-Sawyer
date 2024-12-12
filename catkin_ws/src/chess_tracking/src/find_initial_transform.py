#!/usr/bin/env python

import rospy
import tf
from tf.transformations import quaternion_multiply, quaternion_inverse, translation_matrix, quaternion_matrix, concatenate_matrices, translation_from_matrix, quaternion_from_matrix

# logic for this file:
''' 
Let frame A be the origin of the right_gripper_base while the AR tag is in view of the c920 camera. 
In frame A: get the transform from AR->c920, right_gripper_base-> base
move the robot so that the ar tag is now in view of the right_hand_camera 
Let this new origin of right gripper_base be frame B origin
In frame B: get the transform from base->right_gripper_base
Using base->right_gripper_base in frame b along with frame A
We can compute the transform between Frame A and Frame B
To get the transform from ar->c920 in frame B, just apply the transformation from Frame A to Frame B that we just got
While in frame b: get the transform from, right_hand_camera->ar and base->right_hand_camera
In frame B, compute the transform of Base to c920 using this: base->right_hand_camera * right_hand_camera->ar * ar->c920
Effectively we have the c920 in terms of base just for that one position, but because we know the transform from right_gripper_base->base and base->c920
We can statically transform the transform from right_gripper_base->c920, anchoring it to follow the right_gripper_base
and now we have the exact translation and rotation 
'''


# will publish static transform on tf tree with parent_frame->child_frame
def publish_static_transform(translation, rotation, child_frame, parent_frame):
    static_broadcaster = tf.StaticTransformBroadcaster()
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


def invert_transform(trans, rot):
    mat = concatenate_matrices(
        translation_matrix(trans), quaternion_matrix(rot))
    inv = tf.transformations.inverse_matrix(mat)
    return translation_from_matrix(inv), quaternion_from_matrix(inv)

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

    # get the transform from ar->c920 in frame A
    trans_ar_c920_A, rot_ar_c920_A = get_transform(
        listener, "ar_marker_0", "logitech_c920")
    if trans_ar_c920_A is None:
        return

    # transform of base->gripper base in frame A
    trans_base_right_gripper_base_A, rot_base_right_gripper_base_A = get_transform(
        listener, "base", "right_gripper_base")
    if trans_base_right_gripper_base_A is None:
        return

    rospy.loginfo(
        "move ar tag in view of the right_hand_camera  (right_hand_camera).")
    input("press enter when in view")

    # transform from right_hand_camera to ar in frame B
    trans_right_hand_camera_ar_B, rot_right_hand_camera_ar_B = get_transform(
        listener, "right_hand_camera", "ar_marker_0")
    if trans_right_hand_camera_ar_B is None:
        return

    # transform from base to gripper base in frame B
    trans_base_right_gripper_base_B, rot_base_right_gripper_base_B = get_transform(
        listener, "base", "right_gripper_base")
    if trans_base_right_gripper_base_B is None:
        return

    # transform from base ro right_hand_camera in frame B
    trans_base_right_hand_camera_B, rot_base_right_hand_camera_B = get_transform(
        listener, "base", "right_hand_camera")
    if trans_base_right_hand_camera_B is None:
        return

    # We can get the transfrom from A to B through the right gripper base in each frame
    # Compute T(A->B)
    inv_trans_base_right_gripper_base_A, inv_rot_base_right_gripper_base_A = invert_transform(
        trans_base_right_gripper_base_A, rot_base_right_gripper_base_A
    )
    # gives us right_gripper_base -> base in frame A

    trans_A_B, rot_A_B = combine_transforms(inv_trans_base_right_gripper_base_A, inv_rot_base_right_gripper_base_A,
                                            trans_base_right_gripper_base_B, rot_base_right_gripper_base_B)

    # use the transformation of a->b on ar->c920
    # computes ar->c920 in terms of B
    trans_ar_c920_B, rot_ar_c920_B = combine_transforms(
        trans_A_B, rot_A_B, trans_ar_c920_A, rot_ar_c920_A)

    trans_base_c920_B_step1, rot_base_c920_B_step1 = combine_transforms(
        trans_base_right_hand_camera_B, rot_base_right_hand_camera_B,
        trans_right_hand_camera_ar_B, rot_right_hand_camera_ar_B
    )
    trans_base_c920_B, rot_base_c920_B = combine_transforms(
        trans_base_c920_B_step1, rot_base_c920_B_step1,
        trans_ar_c920_B, rot_ar_c920_B
    )

    # transform from right_gripper_base to base in frame B
    inv_trans_base_right_gripper_base_B, inv_rot_base_right_gripper_base_B = invert_transform(
        trans_base_right_gripper_base_B, rot_base_right_gripper_base_B
    )

    # transfrom from right_gripper_base -> base -> c920 in frame B
    trans_right_gripper_base_c920, rot_right_gripper_base_c920 = combine_transforms(
        inv_trans_base_right_gripper_base_B, inv_rot_base_right_gripper_base_B,
        trans_base_c920_B, rot_base_c920_B
    )

    # Publish the static transform
    publish_static_transform(trans_right_gripper_base_c920, rot_right_gripper_base_c920,
                             "logitech_c920", "right_gripper_base")

    rospy.loginfo("Calibration complete. Static transform published.")


if __name__ == "__main__":
    try:
        calculate_full_camera_transform()

    except rospy.ROSInterruptException:
        pass
