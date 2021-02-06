;; Auto-generated. Do not edit!


(when (boundp 'px_comm::CameraInfo)
  (if (not (find-package "PX_COMM"))
    (make-package "PX_COMM"))
  (shadow 'CameraInfo (find-package "PX_COMM")))
(unless (find-package "PX_COMM::CAMERAINFO")
  (make-package "PX_COMM::CAMERAINFO"))

(in-package "ROS")
;;//! \htmlinclude CameraInfo.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass px_comm::CameraInfo
  :super ros::object
  :slots (_header _camera_model _camera_name _camera_type _image_width _image_height _D _P _M _pose ))

(defmethod px_comm::CameraInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:camera_model __camera_model) "")
    ((:camera_name __camera_name) "")
    ((:camera_type __camera_type) "")
    ((:image_width __image_width) 0)
    ((:image_height __image_height) 0)
    ((:D __D) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:P __P) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:M __M) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _camera_model (string __camera_model))
   (setq _camera_name (string __camera_name))
   (setq _camera_type (string __camera_type))
   (setq _image_width (round __image_width))
   (setq _image_height (round __image_height))
   (setq _D __D)
   (setq _P __P)
   (setq _M __M)
   (setq _pose __pose)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:camera_model
   (&optional __camera_model)
   (if __camera_model (setq _camera_model __camera_model)) _camera_model)
  (:camera_name
   (&optional __camera_name)
   (if __camera_name (setq _camera_name __camera_name)) _camera_name)
  (:camera_type
   (&optional __camera_type)
   (if __camera_type (setq _camera_type __camera_type)) _camera_type)
  (:image_width
   (&optional __image_width)
   (if __image_width (setq _image_width __image_width)) _image_width)
  (:image_height
   (&optional __image_height)
   (if __image_height (setq _image_height __image_height)) _image_height)
  (:D
   (&optional __D)
   (if __D (setq _D __D)) _D)
  (:P
   (&optional __P)
   (if __P (setq _P __P)) _P)
  (:M
   (&optional __M)
   (if __M (setq _M __M)) _M)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _camera_model
    4 (length _camera_model)
    ;; string _camera_name
    4 (length _camera_name)
    ;; string _camera_type
    4 (length _camera_type)
    ;; uint32 _image_width
    4
    ;; uint32 _image_height
    4
    ;; float64[] _D
    (* 8    (length _D)) 4
    ;; float64[] _P
    (* 8    (length _P)) 4
    ;; float64[] _M
    (* 8    (length _M)) 4
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _camera_model
       (write-long (length _camera_model) s) (princ _camera_model s)
     ;; string _camera_name
       (write-long (length _camera_name) s) (princ _camera_name s)
     ;; string _camera_type
       (write-long (length _camera_type) s) (princ _camera_type s)
     ;; uint32 _image_width
       (write-long _image_width s)
     ;; uint32 _image_height
       (write-long _image_height s)
     ;; float64[] _D
     (write-long (length _D) s)
     (dotimes (i (length _D))
       (sys::poke (elt _D i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _P
     (write-long (length _P) s)
     (dotimes (i (length _P))
       (sys::poke (elt _P i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _M
     (write-long (length _M) s)
     (dotimes (i (length _M))
       (sys::poke (elt _M i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _camera_model
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _camera_model (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _camera_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _camera_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _camera_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _camera_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _image_width
     (setq _image_width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _image_height
     (setq _image_height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64[] _D
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _D (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _D i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _P
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _P (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _P i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _M
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _M (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _M i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(setf (get px_comm::CameraInfo :md5sum-) "014513fdee9cefabe3cec97bca5e5c57")
(setf (get px_comm::CameraInfo :datatype-) "px_comm/CameraInfo")
(setf (get px_comm::CameraInfo :definition-)
      "#######################################################################
#                     Image acquisition info                          #
#######################################################################

# Time of image acquisition, camera coordinate frame ID
Header header    # Header timestamp should be acquisition time of image
                 # Header frame_id should be optical frame of camera
                 # origin of frame should be optical center of camera
                 # +x should point to the right in the image
                 # +y should point down in the image
                 # +z should point into the plane of the image


#######################################################################
#                      Calibration Parameters                         #
#######################################################################
# These are fixed during camera calibration. Their values will be the #
# same in all messages until the camera is recalibrated. Note that    #
# self-calibrating systems may \"recalibrate\" frequently.              #
#######################################################################

# The camera model used.
string camera_model

# The name of the camera.
string camera_name

# The type of the camera.
string camera_type

# The image dimensions with which the camera was calibrated. Normally
# this will be the full camera resolution in pixels.
uint32 image_width
uint32 image_height

# The distortion parameters, size depending on the distortion model.
float64[] D

# The projection parameters, size depending on the projection model.
float64[] P

# Other parameters which are not defined by either the distortion or
# projection model.
float64[] M

# Pose of camera with respect to a specific reference frame.
geometry_msgs/Pose pose
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

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :px_comm/CameraInfo "014513fdee9cefabe3cec97bca5e5c57")


