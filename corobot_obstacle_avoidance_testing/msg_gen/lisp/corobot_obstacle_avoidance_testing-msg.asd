
(cl:in-package :asdf)

(defsystem "corobot_obstacle_avoidance_testing-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Wall" :depends-on ("_package_Wall"))
    (:file "_package_Wall" :depends-on ("_package"))
  ))