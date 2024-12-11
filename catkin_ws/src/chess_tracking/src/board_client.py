#!/usr/bin/env python

import rospy

from chess_tracking.srv import BoardString

if __name__ == "__main__":
    rospy.wait_for_service('board_service')
    try:
        output_string = rospy.ServiceProxy('board_service', BoardString)
        resp = output_string()
        print(resp)
    except rospy.ServiceException as e:
        print("bro what", e)
