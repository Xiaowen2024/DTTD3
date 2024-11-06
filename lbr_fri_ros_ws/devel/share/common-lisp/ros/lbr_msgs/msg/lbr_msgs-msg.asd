
(cl:in-package :asdf)

(defsystem "lbr_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "LBRState" :depends-on ("_package_LBRState"))
    (:file "_package_LBRState" :depends-on ("_package"))
  ))