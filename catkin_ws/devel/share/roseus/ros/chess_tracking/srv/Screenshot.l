;; Auto-generated. Do not edit!


(when (boundp 'chess_tracking::Screenshot)
  (if (not (find-package "CHESS_TRACKING"))
    (make-package "CHESS_TRACKING"))
  (shadow 'Screenshot (find-package "CHESS_TRACKING")))
(unless (find-package "CHESS_TRACKING::SCREENSHOT")
  (make-package "CHESS_TRACKING::SCREENSHOT"))
(unless (find-package "CHESS_TRACKING::SCREENSHOTREQUEST")
  (make-package "CHESS_TRACKING::SCREENSHOTREQUEST"))
(unless (find-package "CHESS_TRACKING::SCREENSHOTRESPONSE")
  (make-package "CHESS_TRACKING::SCREENSHOTRESPONSE"))

(in-package "ROS")



(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass chess_tracking::ScreenshotRequest
  :super ros::object
  :slots ())

(defmethod chess_tracking::ScreenshotRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass chess_tracking::ScreenshotResponse
  :super ros::object
  :slots (_img ))

(defmethod chess_tracking::ScreenshotResponse
  (:init
   (&key
    ((:img __img) (instance sensor_msgs::Image :init))
    )
   (send-super :init)
   (setq _img __img)
   self)
  (:img
   (&rest __img)
   (if (keywordp (car __img))
       (send* _img __img)
     (progn
       (if __img (setq _img (car __img)))
       _img)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/Image _img
    (send _img :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/Image _img
       (send _img :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/Image _img
     (send _img :deserialize buf ptr-) (incf ptr- (send _img :serialization-length))
   ;;
   self)
  )

(defclass chess_tracking::Screenshot
  :super ros::object
  :slots ())

(setf (get chess_tracking::Screenshot :md5sum-) "b4274f524cc812fc54ca8ebeeda2deb2")
(setf (get chess_tracking::Screenshot :datatype-) "chess_tracking/Screenshot")
(setf (get chess_tracking::Screenshot :request) chess_tracking::ScreenshotRequest)
(setf (get chess_tracking::Screenshot :response) chess_tracking::ScreenshotResponse)

(defmethod chess_tracking::ScreenshotRequest
  (:response () (instance chess_tracking::ScreenshotResponse :init)))

(setf (get chess_tracking::ScreenshotRequest :md5sum-) "b4274f524cc812fc54ca8ebeeda2deb2")
(setf (get chess_tracking::ScreenshotRequest :datatype-) "chess_tracking/ScreenshotRequest")
(setf (get chess_tracking::ScreenshotRequest :definition-)
      "---
sensor_msgs/Image img

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
")

(setf (get chess_tracking::ScreenshotResponse :md5sum-) "b4274f524cc812fc54ca8ebeeda2deb2")
(setf (get chess_tracking::ScreenshotResponse :datatype-) "chess_tracking/ScreenshotResponse")
(setf (get chess_tracking::ScreenshotResponse :definition-)
      "---
sensor_msgs/Image img

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
")



(provide :chess_tracking/Screenshot "b4274f524cc812fc54ca8ebeeda2deb2")


