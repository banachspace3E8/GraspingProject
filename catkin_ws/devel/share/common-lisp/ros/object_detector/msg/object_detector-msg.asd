
(cl:in-package :asdf)

(defsystem "object_detector-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "detector" :depends-on ("_package_detector"))
    (:file "_package_detector" :depends-on ("_package"))
  ))