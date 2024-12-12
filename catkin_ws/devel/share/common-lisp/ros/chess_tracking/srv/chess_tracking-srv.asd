
(cl:in-package :asdf)

(defsystem "chess_tracking-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :chess_tracking-msg
               :geometry_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "BoardString" :depends-on ("_package_BoardString"))
    (:file "_package_BoardString" :depends-on ("_package"))
    (:file "PieceMatches" :depends-on ("_package_PieceMatches"))
    (:file "_package_PieceMatches" :depends-on ("_package"))
    (:file "Screenshot" :depends-on ("_package_Screenshot"))
    (:file "_package_Screenshot" :depends-on ("_package"))
    (:file "TransformPoint" :depends-on ("_package_TransformPoint"))
    (:file "_package_TransformPoint" :depends-on ("_package"))
  ))