#!/usr/bin/env python

import rospy
import json
from chess_tracking.srv import BoardString, TransformPoint
from geometry_msgs.msg import Point

if __name__ == "__main__":
    rospy.wait_for_service('board_service')
    rospy.wait_for_service("transform_coordinates")

    try:
        call_board_service = rospy.ServiceProxy("board_service", BoardString)
        transform_service = rospy.ServiceProxy(
            'transform_coordinates', TransformPoint)

        resp = call_board_service().output

        dictionary_mapping = json.loads(resp)

        # Example dictionary format: {"A1": [(0, 0), (2, 2)]}
        for currTile in dictionary_mapping:
            bottom_left_corner = dictionary_mapping[currTile][0]
            top_right_corner = dictionary_mapping[currTile][1]

            center_x_coord = (bottom_left_corner[0] + top_right_corner[0]) / 2
            center_y_coord = (bottom_left_corner[1] + top_right_corner[1]) / 2

            # Create the TransformPoint request
            point_request = Point()

            point_request.x = center_x_coord
            point_request.y = center_y_coord
            point_request.z = 1.0  # Hardcoded depth

            # Call the transform_coordinates service

            # return type is of Point.transformed_point (consists of x, y, z)
            real_coords = transform_service(point_request)

            # Print the transformed point in the base frame
            print(f"Transformed point for tile {currTile}: x={real_coords.transformed_point.x}, "
                  f"y={real_coords.transformed_point.y}, z={real_coords.transformed_point.z}")

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
