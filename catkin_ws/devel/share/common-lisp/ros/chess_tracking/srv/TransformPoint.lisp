; Auto-generated. Do not edit!


(cl:in-package chess_tracking-srv)


;//! \htmlinclude TransformPoint-request.msg.html

(cl:defclass <TransformPoint-request> (roslisp-msg-protocol:ros-message)
  ((input_point
    :reader input_point
    :initarg :input_point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass TransformPoint-request (<TransformPoint-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TransformPoint-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TransformPoint-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chess_tracking-srv:<TransformPoint-request> is deprecated: use chess_tracking-srv:TransformPoint-request instead.")))

(cl:ensure-generic-function 'input_point-val :lambda-list '(m))
(cl:defmethod input_point-val ((m <TransformPoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chess_tracking-srv:input_point-val is deprecated.  Use chess_tracking-srv:input_point instead.")
  (input_point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TransformPoint-request>) ostream)
  "Serializes a message object of type '<TransformPoint-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input_point) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TransformPoint-request>) istream)
  "Deserializes a message object of type '<TransformPoint-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input_point) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TransformPoint-request>)))
  "Returns string type for a service object of type '<TransformPoint-request>"
  "chess_tracking/TransformPointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformPoint-request)))
  "Returns string type for a service object of type 'TransformPoint-request"
  "chess_tracking/TransformPointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TransformPoint-request>)))
  "Returns md5sum for a message object of type '<TransformPoint-request>"
  "3ee7093c6df18d4364892f198e4fb79b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TransformPoint-request)))
  "Returns md5sum for a message object of type 'TransformPoint-request"
  "3ee7093c6df18d4364892f198e4fb79b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TransformPoint-request>)))
  "Returns full string definition for message of type '<TransformPoint-request>"
  (cl:format cl:nil "# TransformPoint.srv~%geometry_msgs/Point input_point~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TransformPoint-request)))
  "Returns full string definition for message of type 'TransformPoint-request"
  (cl:format cl:nil "# TransformPoint.srv~%geometry_msgs/Point input_point~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TransformPoint-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input_point))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TransformPoint-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TransformPoint-request
    (cl:cons ':input_point (input_point msg))
))
;//! \htmlinclude TransformPoint-response.msg.html

(cl:defclass <TransformPoint-response> (roslisp-msg-protocol:ros-message)
  ((transformed_point
    :reader transformed_point
    :initarg :transformed_point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass TransformPoint-response (<TransformPoint-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TransformPoint-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TransformPoint-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chess_tracking-srv:<TransformPoint-response> is deprecated: use chess_tracking-srv:TransformPoint-response instead.")))

(cl:ensure-generic-function 'transformed_point-val :lambda-list '(m))
(cl:defmethod transformed_point-val ((m <TransformPoint-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chess_tracking-srv:transformed_point-val is deprecated.  Use chess_tracking-srv:transformed_point instead.")
  (transformed_point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TransformPoint-response>) ostream)
  "Serializes a message object of type '<TransformPoint-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'transformed_point) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TransformPoint-response>) istream)
  "Deserializes a message object of type '<TransformPoint-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'transformed_point) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TransformPoint-response>)))
  "Returns string type for a service object of type '<TransformPoint-response>"
  "chess_tracking/TransformPointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformPoint-response)))
  "Returns string type for a service object of type 'TransformPoint-response"
  "chess_tracking/TransformPointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TransformPoint-response>)))
  "Returns md5sum for a message object of type '<TransformPoint-response>"
  "3ee7093c6df18d4364892f198e4fb79b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TransformPoint-response)))
  "Returns md5sum for a message object of type 'TransformPoint-response"
  "3ee7093c6df18d4364892f198e4fb79b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TransformPoint-response>)))
  "Returns full string definition for message of type '<TransformPoint-response>"
  (cl:format cl:nil "geometry_msgs/Point transformed_point~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TransformPoint-response)))
  "Returns full string definition for message of type 'TransformPoint-response"
  (cl:format cl:nil "geometry_msgs/Point transformed_point~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TransformPoint-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'transformed_point))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TransformPoint-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TransformPoint-response
    (cl:cons ':transformed_point (transformed_point msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TransformPoint)))
  'TransformPoint-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TransformPoint)))
  'TransformPoint-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformPoint)))
  "Returns string type for a service object of type '<TransformPoint>"
  "chess_tracking/TransformPoint")