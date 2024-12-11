
(cl:in-package :asdf)

(defsystem "chess_tracking-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "StringAndFloatsGrid" :depends-on ("_package_StringAndFloatsGrid"))
    (:file "_package_StringAndFloatsGrid" :depends-on ("_package"))
  ))