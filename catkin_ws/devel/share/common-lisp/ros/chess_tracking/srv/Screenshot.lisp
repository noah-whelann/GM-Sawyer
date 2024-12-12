; Auto-generated. Do not edit!


(cl:in-package chess_tracking-srv)


;//! \htmlinclude Screenshot-request.msg.html

(cl:defclass <Screenshot-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Screenshot-request (<Screenshot-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Screenshot-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Screenshot-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chess_tracking-srv:<Screenshot-request> is deprecated: use chess_tracking-srv:Screenshot-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Screenshot-request>) ostream)
  "Serializes a message object of type '<Screenshot-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Screenshot-request>) istream)
  "Deserializes a message object of type '<Screenshot-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Screenshot-request>)))
  "Returns string type for a service object of type '<Screenshot-request>"
  "chess_tracking/ScreenshotRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Screenshot-request)))
  "Returns string type for a service object of type 'Screenshot-request"
  "chess_tracking/ScreenshotRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Screenshot-request>)))
  "Returns md5sum for a message object of type '<Screenshot-request>"
  "b4274f524cc812fc54ca8ebeeda2deb2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Screenshot-request)))
  "Returns md5sum for a message object of type 'Screenshot-request"
  "b4274f524cc812fc54ca8ebeeda2deb2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Screenshot-request>)))
  "Returns full string definition for message of type '<Screenshot-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Screenshot-request)))
  "Returns full string definition for message of type 'Screenshot-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Screenshot-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Screenshot-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Screenshot-request
))
;//! \htmlinclude Screenshot-response.msg.html

(cl:defclass <Screenshot-response> (roslisp-msg-protocol:ros-message)
  ((img
    :reader img
    :initarg :img
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass Screenshot-response (<Screenshot-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Screenshot-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Screenshot-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chess_tracking-srv:<Screenshot-response> is deprecated: use chess_tracking-srv:Screenshot-response instead.")))

(cl:ensure-generic-function 'img-val :lambda-list '(m))
(cl:defmethod img-val ((m <Screenshot-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chess_tracking-srv:img-val is deprecated.  Use chess_tracking-srv:img instead.")
  (img m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Screenshot-response>) ostream)
  "Serializes a message object of type '<Screenshot-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'img) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Screenshot-response>) istream)
  "Deserializes a message object of type '<Screenshot-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'img) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Screenshot-response>)))
  "Returns string type for a service object of type '<Screenshot-response>"
  "chess_tracking/ScreenshotResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Screenshot-response)))
  "Returns string type for a service object of type 'Screenshot-response"
  "chess_tracking/ScreenshotResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Screenshot-response>)))
  "Returns md5sum for a message object of type '<Screenshot-response>"
  "b4274f524cc812fc54ca8ebeeda2deb2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Screenshot-response)))
  "Returns md5sum for a message object of type 'Screenshot-response"
  "b4274f524cc812fc54ca8ebeeda2deb2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Screenshot-response>)))
  "Returns full string definition for message of type '<Screenshot-response>"
  (cl:format cl:nil "sensor_msgs/Image img~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Screenshot-response)))
  "Returns full string definition for message of type 'Screenshot-response"
  (cl:format cl:nil "sensor_msgs/Image img~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Screenshot-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'img))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Screenshot-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Screenshot-response
    (cl:cons ':img (img msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Screenshot)))
  'Screenshot-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Screenshot)))
  'Screenshot-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Screenshot)))
  "Returns string type for a service object of type '<Screenshot>"
  "chess_tracking/Screenshot")