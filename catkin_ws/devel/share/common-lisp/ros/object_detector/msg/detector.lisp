; Auto-generated. Do not edit!


(cl:in-package object_detector-msg)


;//! \htmlinclude detector.msg.html

(cl:defclass <detector> (roslisp-msg-protocol:ros-message)
  ((color
    :reader color
    :initarg :color
    :type std_msgs-msg:ColorRGBA
    :initform (cl:make-instance 'std_msgs-msg:ColorRGBA)))
)

(cl:defclass detector (<detector>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <detector>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'detector)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name object_detector-msg:<detector> is deprecated: use object_detector-msg:detector instead.")))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <detector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detector-msg:color-val is deprecated.  Use object_detector-msg:color instead.")
  (color m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <detector>) ostream)
  "Serializes a message object of type '<detector>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'color) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <detector>) istream)
  "Deserializes a message object of type '<detector>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'color) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<detector>)))
  "Returns string type for a message object of type '<detector>"
  "object_detector/detector")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'detector)))
  "Returns string type for a message object of type 'detector"
  "object_detector/detector")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<detector>)))
  "Returns md5sum for a message object of type '<detector>"
  "3e04b62b1b39cd97e873789f0bb130e7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'detector)))
  "Returns md5sum for a message object of type 'detector"
  "3e04b62b1b39cd97e873789f0bb130e7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<detector>)))
  "Returns full string definition for message of type '<detector>"
  (cl:format cl:nil "std_msgs/ColorRGBA color~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'detector)))
  "Returns full string definition for message of type 'detector"
  (cl:format cl:nil "std_msgs/ColorRGBA color~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <detector>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'color))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <detector>))
  "Converts a ROS message object to a list"
  (cl:list 'detector
    (cl:cons ':color (color msg))
))
