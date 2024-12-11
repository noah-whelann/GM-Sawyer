;; Auto-generated. Do not edit!


(when (boundp 'chess_tracking::BoardString)
  (if (not (find-package "CHESS_TRACKING"))
    (make-package "CHESS_TRACKING"))
  (shadow 'BoardString (find-package "CHESS_TRACKING")))
(unless (find-package "CHESS_TRACKING::BOARDSTRING")
  (make-package "CHESS_TRACKING::BOARDSTRING"))
(unless (find-package "CHESS_TRACKING::BOARDSTRINGREQUEST")
  (make-package "CHESS_TRACKING::BOARDSTRINGREQUEST"))
(unless (find-package "CHESS_TRACKING::BOARDSTRINGRESPONSE")
  (make-package "CHESS_TRACKING::BOARDSTRINGRESPONSE"))

(in-package "ROS")





(defclass chess_tracking::BoardStringRequest
  :super ros::object
  :slots ())

(defmethod chess_tracking::BoardStringRequest
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

(defclass chess_tracking::BoardStringResponse
  :super ros::object
  :slots (_output ))

(defmethod chess_tracking::BoardStringResponse
  (:init
   (&key
    ((:output __output) "")
    )
   (send-super :init)
   (setq _output (string __output))
   self)
  (:output
   (&optional __output)
   (if __output (setq _output __output)) _output)
  (:serialization-length
   ()
   (+
    ;; string _output
    4 (length _output)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _output
       (write-long (length _output) s) (princ _output s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _output
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _output (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass chess_tracking::BoardString
  :super ros::object
  :slots ())

(setf (get chess_tracking::BoardString :md5sum-) "0825d95fdfa2c8f4bbb4e9c74bccd3fd")
(setf (get chess_tracking::BoardString :datatype-) "chess_tracking/BoardString")
(setf (get chess_tracking::BoardString :request) chess_tracking::BoardStringRequest)
(setf (get chess_tracking::BoardString :response) chess_tracking::BoardStringResponse)

(defmethod chess_tracking::BoardStringRequest
  (:response () (instance chess_tracking::BoardStringResponse :init)))

(setf (get chess_tracking::BoardStringRequest :md5sum-) "0825d95fdfa2c8f4bbb4e9c74bccd3fd")
(setf (get chess_tracking::BoardStringRequest :datatype-) "chess_tracking/BoardStringRequest")
(setf (get chess_tracking::BoardStringRequest :definition-)
      "---
string output
")

(setf (get chess_tracking::BoardStringResponse :md5sum-) "0825d95fdfa2c8f4bbb4e9c74bccd3fd")
(setf (get chess_tracking::BoardStringResponse :datatype-) "chess_tracking/BoardStringResponse")
(setf (get chess_tracking::BoardStringResponse :definition-)
      "---
string output
")



(provide :chess_tracking/BoardString "0825d95fdfa2c8f4bbb4e9c74bccd3fd")


