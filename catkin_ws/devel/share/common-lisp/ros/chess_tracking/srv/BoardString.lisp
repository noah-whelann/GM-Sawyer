; Auto-generated. Do not edit!


(cl:in-package chess_tracking-srv)


;//! \htmlinclude BoardString-request.msg.html

(cl:defclass <BoardString-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass BoardString-request (<BoardString-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BoardString-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BoardString-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chess_tracking-srv:<BoardString-request> is deprecated: use chess_tracking-srv:BoardString-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BoardString-request>) ostream)
  "Serializes a message object of type '<BoardString-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BoardString-request>) istream)
  "Deserializes a message object of type '<BoardString-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BoardString-request>)))
  "Returns string type for a service object of type '<BoardString-request>"
  "chess_tracking/BoardStringRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BoardString-request)))
  "Returns string type for a service object of type 'BoardString-request"
  "chess_tracking/BoardStringRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BoardString-request>)))
  "Returns md5sum for a message object of type '<BoardString-request>"
  "0825d95fdfa2c8f4bbb4e9c74bccd3fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BoardString-request)))
  "Returns md5sum for a message object of type 'BoardString-request"
  "0825d95fdfa2c8f4bbb4e9c74bccd3fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BoardString-request>)))
  "Returns full string definition for message of type '<BoardString-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BoardString-request)))
  "Returns full string definition for message of type 'BoardString-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BoardString-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BoardString-request>))
  "Converts a ROS message object to a list"
  (cl:list 'BoardString-request
))
;//! \htmlinclude BoardString-response.msg.html

(cl:defclass <BoardString-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type cl:string
    :initform ""))
)

(cl:defclass BoardString-response (<BoardString-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BoardString-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BoardString-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chess_tracking-srv:<BoardString-response> is deprecated: use chess_tracking-srv:BoardString-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <BoardString-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chess_tracking-srv:output-val is deprecated.  Use chess_tracking-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BoardString-response>) ostream)
  "Serializes a message object of type '<BoardString-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'output))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'output))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BoardString-response>) istream)
  "Deserializes a message object of type '<BoardString-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'output) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'output) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BoardString-response>)))
  "Returns string type for a service object of type '<BoardString-response>"
  "chess_tracking/BoardStringResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BoardString-response)))
  "Returns string type for a service object of type 'BoardString-response"
  "chess_tracking/BoardStringResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BoardString-response>)))
  "Returns md5sum for a message object of type '<BoardString-response>"
  "0825d95fdfa2c8f4bbb4e9c74bccd3fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BoardString-response)))
  "Returns md5sum for a message object of type 'BoardString-response"
  "0825d95fdfa2c8f4bbb4e9c74bccd3fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BoardString-response>)))
  "Returns full string definition for message of type '<BoardString-response>"
  (cl:format cl:nil "string output~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BoardString-response)))
  "Returns full string definition for message of type 'BoardString-response"
  (cl:format cl:nil "string output~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BoardString-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BoardString-response>))
  "Converts a ROS message object to a list"
  (cl:list 'BoardString-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'BoardString)))
  'BoardString-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'BoardString)))
  'BoardString-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BoardString)))
  "Returns string type for a service object of type '<BoardString>"
  "chess_tracking/BoardString")