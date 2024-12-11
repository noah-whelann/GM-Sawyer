import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped


def transform_point_to_base(point, source_frame="logitech_c920", target_frame="base"):
    # Initialize the TF2 buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Create a PointStamped from the input point
    point_in_camera = PointStamped()
    point_in_camera.header.frame_id = source_frame
    point_in_camera.header.stamp = rospy.Time.now()
    point_in_camera.point = point

    try:
        # Lookup the transform from source to target frame
        transform = tf_buffer.lookup_transform(
            target_frame,              # Target frame
            source_frame,              # Source frame
            rospy.Time(0),             # Use the latest available transform
            rospy.Duration(1.0)        # Timeout duration
        )

        # Transform the point
        point_in_base = tf2_geometry_msgs.do_transform_point(
            point_in_camera, transform)

        rospy.loginfo("Point in base frame: x=%.3f, y=%.3f, z=%.3f",
                      point_in_base.point.x,
                      point_in_base.point.y,
                      point_in_base.point.z)
        return point_in_base.point

    except Exception as e:
        rospy.logerr("Failed to transform point: %s", e)
        raise Exception("Failed to transform point: {}".format(e))
