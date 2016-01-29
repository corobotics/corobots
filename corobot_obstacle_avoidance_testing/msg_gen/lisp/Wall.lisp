; Auto-generated. Do not edit!


(cl:in-package corobot_obstacle_avoidance_testing-msg)


;//! \htmlinclude Wall.msg.html

(cl:defclass <Wall> (roslisp-msg-protocol:ros-message)
  ((rleft
    :reader rleft
    :initarg :rleft
    :type cl:float
    :initform 0.0)
   (thetaleft
    :reader thetaleft
    :initarg :thetaleft
    :type cl:float
    :initform 0.0)
   (conf_left
    :reader conf_left
    :initarg :conf_left
    :type cl:integer
    :initform 0)
   (is_wall_left
    :reader is_wall_left
    :initarg :is_wall_left
    :type cl:boolean
    :initform cl:nil)
   (rright
    :reader rright
    :initarg :rright
    :type cl:float
    :initform 0.0)
   (thetaright
    :reader thetaright
    :initarg :thetaright
    :type cl:float
    :initform 0.0)
   (conf_right
    :reader conf_right
    :initarg :conf_right
    :type cl:integer
    :initform 0)
   (is_wall_right
    :reader is_wall_right
    :initarg :is_wall_right
    :type cl:boolean
    :initform cl:nil)
   (tdiv
    :reader tdiv
    :initarg :tdiv
    :type cl:integer
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0)
   (accumulator
    :reader accumulator
    :initarg :accumulator
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass Wall (<Wall>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Wall>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Wall)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name corobot_obstacle_avoidance_testing-msg:<Wall> is deprecated: use corobot_obstacle_avoidance_testing-msg:Wall instead.")))

(cl:ensure-generic-function 'rleft-val :lambda-list '(m))
(cl:defmethod rleft-val ((m <Wall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader corobot_obstacle_avoidance_testing-msg:rleft-val is deprecated.  Use corobot_obstacle_avoidance_testing-msg:rleft instead.")
  (rleft m))

(cl:ensure-generic-function 'thetaleft-val :lambda-list '(m))
(cl:defmethod thetaleft-val ((m <Wall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader corobot_obstacle_avoidance_testing-msg:thetaleft-val is deprecated.  Use corobot_obstacle_avoidance_testing-msg:thetaleft instead.")
  (thetaleft m))

(cl:ensure-generic-function 'conf_left-val :lambda-list '(m))
(cl:defmethod conf_left-val ((m <Wall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader corobot_obstacle_avoidance_testing-msg:conf_left-val is deprecated.  Use corobot_obstacle_avoidance_testing-msg:conf_left instead.")
  (conf_left m))

(cl:ensure-generic-function 'is_wall_left-val :lambda-list '(m))
(cl:defmethod is_wall_left-val ((m <Wall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader corobot_obstacle_avoidance_testing-msg:is_wall_left-val is deprecated.  Use corobot_obstacle_avoidance_testing-msg:is_wall_left instead.")
  (is_wall_left m))

(cl:ensure-generic-function 'rright-val :lambda-list '(m))
(cl:defmethod rright-val ((m <Wall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader corobot_obstacle_avoidance_testing-msg:rright-val is deprecated.  Use corobot_obstacle_avoidance_testing-msg:rright instead.")
  (rright m))

(cl:ensure-generic-function 'thetaright-val :lambda-list '(m))
(cl:defmethod thetaright-val ((m <Wall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader corobot_obstacle_avoidance_testing-msg:thetaright-val is deprecated.  Use corobot_obstacle_avoidance_testing-msg:thetaright instead.")
  (thetaright m))

(cl:ensure-generic-function 'conf_right-val :lambda-list '(m))
(cl:defmethod conf_right-val ((m <Wall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader corobot_obstacle_avoidance_testing-msg:conf_right-val is deprecated.  Use corobot_obstacle_avoidance_testing-msg:conf_right instead.")
  (conf_right m))

(cl:ensure-generic-function 'is_wall_right-val :lambda-list '(m))
(cl:defmethod is_wall_right-val ((m <Wall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader corobot_obstacle_avoidance_testing-msg:is_wall_right-val is deprecated.  Use corobot_obstacle_avoidance_testing-msg:is_wall_right instead.")
  (is_wall_right m))

(cl:ensure-generic-function 'tdiv-val :lambda-list '(m))
(cl:defmethod tdiv-val ((m <Wall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader corobot_obstacle_avoidance_testing-msg:tdiv-val is deprecated.  Use corobot_obstacle_avoidance_testing-msg:tdiv instead.")
  (tdiv m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <Wall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader corobot_obstacle_avoidance_testing-msg:height-val is deprecated.  Use corobot_obstacle_avoidance_testing-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'accumulator-val :lambda-list '(m))
(cl:defmethod accumulator-val ((m <Wall>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader corobot_obstacle_avoidance_testing-msg:accumulator-val is deprecated.  Use corobot_obstacle_avoidance_testing-msg:accumulator instead.")
  (accumulator m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Wall>) ostream)
  "Serializes a message object of type '<Wall>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rleft))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thetaleft))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'conf_left)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_wall_left) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rright))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thetaright))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'conf_right)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_wall_right) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tdiv)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tdiv)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tdiv)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tdiv)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'accumulator))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'accumulator))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Wall>) istream)
  "Deserializes a message object of type '<Wall>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rleft) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thetaleft) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'conf_left) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'is_wall_left) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rright) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thetaright) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'conf_right) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'is_wall_right) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tdiv)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tdiv)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tdiv)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tdiv)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'accumulator) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'accumulator)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Wall>)))
  "Returns string type for a message object of type '<Wall>"
  "corobot_obstacle_avoidance_testing/Wall")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Wall)))
  "Returns string type for a message object of type 'Wall"
  "corobot_obstacle_avoidance_testing/Wall")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Wall>)))
  "Returns md5sum for a message object of type '<Wall>"
  "b9dcd15249d280cbfabbb015f7ba5fce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Wall)))
  "Returns md5sum for a message object of type 'Wall"
  "b9dcd15249d280cbfabbb015f7ba5fce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Wall>)))
  "Returns full string definition for message of type '<Wall>"
  (cl:format cl:nil "float32 rleft~%float32 thetaleft~%int32 conf_left~%bool is_wall_left~%~%float32 rright~%float32 thetaright~%int32 conf_right~%bool is_wall_right~%~%uint32 tdiv~%uint32 height~%int32[] accumulator~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Wall)))
  "Returns full string definition for message of type 'Wall"
  (cl:format cl:nil "float32 rleft~%float32 thetaleft~%int32 conf_left~%bool is_wall_left~%~%float32 rright~%float32 thetaright~%int32 conf_right~%bool is_wall_right~%~%uint32 tdiv~%uint32 height~%int32[] accumulator~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Wall>))
  (cl:+ 0
     4
     4
     4
     1
     4
     4
     4
     1
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'accumulator) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Wall>))
  "Converts a ROS message object to a list"
  (cl:list 'Wall
    (cl:cons ':rleft (rleft msg))
    (cl:cons ':thetaleft (thetaleft msg))
    (cl:cons ':conf_left (conf_left msg))
    (cl:cons ':is_wall_left (is_wall_left msg))
    (cl:cons ':rright (rright msg))
    (cl:cons ':thetaright (thetaright msg))
    (cl:cons ':conf_right (conf_right msg))
    (cl:cons ':is_wall_right (is_wall_right msg))
    (cl:cons ':tdiv (tdiv msg))
    (cl:cons ':height (height msg))
    (cl:cons ':accumulator (accumulator msg))
))
