;; Auto-generated. Do not edit!


(when (boundp 'lbr_msgs::LBRState)
  (if (not (find-package "LBR_MSGS"))
    (make-package "LBR_MSGS"))
  (shadow 'LBRState (find-package "LBR_MSGS")))
(unless (find-package "LBR_MSGS::LBRSTATE")
  (make-package "LBR_MSGS::LBRSTATE"))

(in-package "ROS")
;;//! \htmlinclude LBRState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass lbr_msgs::LBRState
  :super ros::object
  :slots (_stamp _sample_time _position _torque _external_torque ))

(defmethod lbr_msgs::LBRState
  (:init
   (&key
    ((:stamp __stamp) (instance std_msgs::Time :init))
    ((:sample_time __sample_time) 0.0)
    ((:position __position) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:torque __torque) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:external_torque __external_torque) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _sample_time (float __sample_time))
   (setq _position __position)
   (setq _torque __torque)
   (setq _external_torque __external_torque)
   self)
  (:stamp
   (&rest __stamp)
   (if (keywordp (car __stamp))
       (send* _stamp __stamp)
     (progn
       (if __stamp (setq _stamp (car __stamp)))
       _stamp)))
  (:sample_time
   (&optional __sample_time)
   (if __sample_time (setq _sample_time __sample_time)) _sample_time)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:torque
   (&optional __torque)
   (if __torque (setq _torque __torque)) _torque)
  (:external_torque
   (&optional __external_torque)
   (if __external_torque (setq _external_torque __external_torque)) _external_torque)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Time _stamp
    (send _stamp :serialization-length)
    ;; float64 _sample_time
    8
    ;; float64[] _position
    (* 8    (length _position)) 4
    ;; float64[] _torque
    (* 8    (length _torque)) 4
    ;; float64[] _external_torque
    (* 8    (length _external_torque)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Time _stamp
       (send _stamp :serialize s)
     ;; float64 _sample_time
       (sys::poke _sample_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _position
     (write-long (length _position) s)
     (dotimes (i (length _position))
       (sys::poke (elt _position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _torque
     (write-long (length _torque) s)
     (dotimes (i (length _torque))
       (sys::poke (elt _torque i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _external_torque
     (write-long (length _external_torque) s)
     (dotimes (i (length _external_torque))
       (sys::poke (elt _external_torque i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Time _stamp
     (send _stamp :deserialize buf ptr-) (incf ptr- (send _stamp :serialization-length))
   ;; float64 _sample_time
     (setq _sample_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _position (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _torque
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _torque (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _torque i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _external_torque
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _external_torque (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _external_torque i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get lbr_msgs::LBRState :md5sum-) "71e053993298b574a690156a73db8a42")
(setf (get lbr_msgs::LBRState :datatype-) "lbr_msgs/LBRState")
(setf (get lbr_msgs::LBRState :definition-)
      "std_msgs/Time stamp
float64 sample_time
float64[] position
float64[] torque
float64[] external_torque

================================================================================
MSG: std_msgs/Time
time data

")



(provide :lbr_msgs/LBRState "71e053993298b574a690156a73db8a42")


