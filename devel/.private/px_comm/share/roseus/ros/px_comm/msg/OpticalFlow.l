;; Auto-generated. Do not edit!


(when (boundp 'px_comm::OpticalFlow)
  (if (not (find-package "PX_COMM"))
    (make-package "PX_COMM"))
  (shadow 'OpticalFlow (find-package "PX_COMM")))
(unless (find-package "PX_COMM::OPTICALFLOW")
  (make-package "PX_COMM::OPTICALFLOW"))

(in-package "ROS")
;;//! \htmlinclude OpticalFlow.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass px_comm::OpticalFlow
  :super ros::object
  :slots (_header _ground_distance _flow_x _flow_y _velocity_x _velocity_y _quality ))

(defmethod px_comm::OpticalFlow
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:ground_distance __ground_distance) 0.0)
    ((:flow_x __flow_x) 0)
    ((:flow_y __flow_y) 0)
    ((:velocity_x __velocity_x) 0.0)
    ((:velocity_y __velocity_y) 0.0)
    ((:quality __quality) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _ground_distance (float __ground_distance))
   (setq _flow_x (round __flow_x))
   (setq _flow_y (round __flow_y))
   (setq _velocity_x (float __velocity_x))
   (setq _velocity_y (float __velocity_y))
   (setq _quality (round __quality))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:ground_distance
   (&optional __ground_distance)
   (if __ground_distance (setq _ground_distance __ground_distance)) _ground_distance)
  (:flow_x
   (&optional __flow_x)
   (if __flow_x (setq _flow_x __flow_x)) _flow_x)
  (:flow_y
   (&optional __flow_y)
   (if __flow_y (setq _flow_y __flow_y)) _flow_y)
  (:velocity_x
   (&optional __velocity_x)
   (if __velocity_x (setq _velocity_x __velocity_x)) _velocity_x)
  (:velocity_y
   (&optional __velocity_y)
   (if __velocity_y (setq _velocity_y __velocity_y)) _velocity_y)
  (:quality
   (&optional __quality)
   (if __quality (setq _quality __quality)) _quality)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _ground_distance
    4
    ;; int16 _flow_x
    2
    ;; int16 _flow_y
    2
    ;; float32 _velocity_x
    4
    ;; float32 _velocity_y
    4
    ;; uint8 _quality
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _ground_distance
       (sys::poke _ground_distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int16 _flow_x
       (write-word _flow_x s)
     ;; int16 _flow_y
       (write-word _flow_y s)
     ;; float32 _velocity_x
       (sys::poke _velocity_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _velocity_y
       (sys::poke _velocity_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _quality
       (write-byte _quality s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _ground_distance
     (setq _ground_distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int16 _flow_x
     (setq _flow_x (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _flow_y
     (setq _flow_y (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _velocity_x
     (setq _velocity_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _velocity_y
     (setq _velocity_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _quality
     (setq _quality (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get px_comm::OpticalFlow :md5sum-) "6705fe0e94fea14978a508d00cf97427")
(setf (get px_comm::OpticalFlow :datatype-) "px_comm/OpticalFlow")
(setf (get px_comm::OpticalFlow :definition-)
      "Header header

float32 ground_distance  # distance to ground in meters
int16   flow_x           # x-component of optical flow in pixels
int16   flow_y           # y-component of optical flow in pixels
float32 velocity_x       # x-component of scaled optical flow in m/s
float32 velocity_y       # y-component of scaled optical flow in m/s
uint8   quality          # quality of optical flow estimate

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :px_comm/OpticalFlow "6705fe0e94fea14978a508d00cf97427")


