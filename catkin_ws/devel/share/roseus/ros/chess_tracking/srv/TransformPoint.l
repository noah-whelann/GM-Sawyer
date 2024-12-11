;; Auto-generated. Do not edit!


(when (boundp 'chess_tracking::TransformPoint)
  (if (not (find-package "CHESS_TRACKING"))
    (make-package "CHESS_TRACKING"))
  (shadow 'TransformPoint (find-package "CHESS_TRACKING")))
(unless (find-package "CHESS_TRACKING::TRANSFORMPOINT")
  (make-package "CHESS_TRACKING::TRANSFORMPOINT"))
(unless (find-package "CHESS_TRACKING::TRANSFORMPOINTREQUEST")
  (make-package "CHESS_TRACKING::TRANSFORMPOINTREQUEST"))
(unless (find-package "CHESS_TRACKING::TRANSFORMPOINTRESPONSE")
  (make-package "CHESS_TRACKING::TRANSFORMPOINTRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass chess_tracking::TransformPointRequest
  :super ros::object
  :slots (_input_point ))

(defmethod chess_tracking::TransformPointRequest
  (:init
   (&key
    ((:input_point __input_point) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _input_point __input_point)
   self)
  (:input_point
   (&rest __input_point)
   (if (keywordp (car __input_point))
       (send* _input_point __input_point)
     (progn
       (if __input_point (setq _input_point (car __input_point)))
       _input_point)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _input_point
    (send _input_point :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _input_point
       (send _input_point :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _input_point
     (send _input_point :deserialize buf ptr-) (incf ptr- (send _input_point :serialization-length))
   ;;
   self)
  )

(defclass chess_tracking::TransformPointResponse
  :super ros::object
  :slots (_transformed_point ))

(defmethod chess_tracking::TransformPointResponse
  (:init
   (&key
    ((:transformed_point __transformed_point) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _transformed_point __transformed_point)
   self)
  (:transformed_point
   (&rest __transformed_point)
   (if (keywordp (car __transformed_point))
       (send* _transformed_point __transformed_point)
     (progn
       (if __transformed_point (setq _transformed_point (car __transformed_point)))
       _transformed_point)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _transformed_point
    (send _transformed_point :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _transformed_point
       (send _transformed_point :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _transformed_point
     (send _transformed_point :deserialize buf ptr-) (incf ptr- (send _transformed_point :serialization-length))
   ;;
   self)
  )

(defclass chess_tracking::TransformPoint
  :super ros::object
  :slots ())

(setf (get chess_tracking::TransformPoint :md5sum-) "3ee7093c6df18d4364892f198e4fb79b")
(setf (get chess_tracking::TransformPoint :datatype-) "chess_tracking/TransformPoint")
(setf (get chess_tracking::TransformPoint :request) chess_tracking::TransformPointRequest)
(setf (get chess_tracking::TransformPoint :response) chess_tracking::TransformPointResponse)

(defmethod chess_tracking::TransformPointRequest
  (:response () (instance chess_tracking::TransformPointResponse :init)))

(setf (get chess_tracking::TransformPointRequest :md5sum-) "3ee7093c6df18d4364892f198e4fb79b")
(setf (get chess_tracking::TransformPointRequest :datatype-) "chess_tracking/TransformPointRequest")
(setf (get chess_tracking::TransformPointRequest :definition-)
      "# TransformPoint.srv
geometry_msgs/Point input_point

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
geometry_msgs/Point transformed_point


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")

(setf (get chess_tracking::TransformPointResponse :md5sum-) "3ee7093c6df18d4364892f198e4fb79b")
(setf (get chess_tracking::TransformPointResponse :datatype-) "chess_tracking/TransformPointResponse")
(setf (get chess_tracking::TransformPointResponse :definition-)
      "# TransformPoint.srv
geometry_msgs/Point input_point

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
geometry_msgs/Point transformed_point


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")



(provide :chess_tracking/TransformPoint "3ee7093c6df18d4364892f198e4fb79b")


