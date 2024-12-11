
(cl:in-package :asdf)

(defsystem "chess_tracking-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "BoardString" :depends-on ("_package_BoardString"))
    (:file "_package_BoardString" :depends-on ("_package"))
    (:file "TransformPoint" :depends-on ("_package_TransformPoint"))
    (:file "_package_TransformPoint" :depends-on ("_package"))
  ))