;; Auto-generated. Do not edit!


(when (boundp 'object_detector::detector)
  (if (not (find-package "OBJECT_DETECTOR"))
    (make-package "OBJECT_DETECTOR"))
  (shadow 'detector (find-package "OBJECT_DETECTOR")))
(unless (find-package "OBJECT_DETECTOR::DETECTOR")
  (make-package "OBJECT_DETECTOR::DETECTOR"))

(in-package "ROS")
;;//! \htmlinclude detector.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass object_detector::detector
  :super ros::object
  :slots (_color ))

(defmethod object_detector::detector
  (:init
   (&key
    ((:color __color) (instance std_msgs::ColorRGBA :init))
    )
   (send-super :init)
   (setq _color __color)
   self)
  (:color
   (&rest __color)
   (if (keywordp (car __color))
       (send* _color __color)
     (progn
       (if __color (setq _color (car __color)))
       _color)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/ColorRGBA _color
    (send _color :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/ColorRGBA _color
       (send _color :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/ColorRGBA _color
     (send _color :deserialize buf ptr-) (incf ptr- (send _color :serialization-length))
   ;;
   self)
  )

(setf (get object_detector::detector :md5sum-) "3e04b62b1b39cd97e873789f0bb130e7")
(setf (get object_detector::detector :datatype-) "object_detector/detector")
(setf (get object_detector::detector :definition-)
      "std_msgs/ColorRGBA color

================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a

")



(provide :object_detector/detector "3e04b62b1b39cd97e873789f0bb130e7")


