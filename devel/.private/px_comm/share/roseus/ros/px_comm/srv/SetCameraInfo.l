;; Auto-generated. Do not edit!


(when (boundp 'px_comm::SetCameraInfo)
  (if (not (find-package "PX_COMM"))
    (make-package "PX_COMM"))
  (shadow 'SetCameraInfo (find-package "PX_COMM")))
(unless (find-package "PX_COMM::SETCAMERAINFO")
  (make-package "PX_COMM::SETCAMERAINFO"))
(unless (find-package "PX_COMM::SETCAMERAINFOREQUEST")
  (make-package "PX_COMM::SETCAMERAINFOREQUEST"))
(unless (find-package "PX_COMM::SETCAMERAINFORESPONSE")
  (make-package "PX_COMM::SETCAMERAINFORESPONSE"))

(in-package "ROS")





(defclass px_comm::SetCameraInfoRequest
  :super ros::object
  :slots (_camera_info ))

(defmethod px_comm::SetCameraInfoRequest
  (:init
   (&key
    ((:camera_info __camera_info) (instance px_comm::CameraInfo :init))
    )
   (send-super :init)
   (setq _camera_info __camera_info)
   self)
  (:camera_info
   (&rest __camera_info)
   (if (keywordp (car __camera_info))
       (send* _camera_info __camera_info)
     (progn
       (if __camera_info (setq _camera_info (car __camera_info)))
       _camera_info)))
  (:serialization-length
   ()
   (+
    ;; px_comm/CameraInfo _camera_info
    (send _camera_info :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; px_comm/CameraInfo _camera_info
       (send _camera_info :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; px_comm/CameraInfo _camera_info
     (send _camera_info :deserialize buf ptr-) (incf ptr- (send _camera_info :serialization-length))
   ;;
   self)
  )

(defclass px_comm::SetCameraInfoResponse
  :super ros::object
  :slots (_success _status_message ))

(defmethod px_comm::SetCameraInfoResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:status_message __status_message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _status_message (string __status_message))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:status_message
   (&optional __status_message)
   (if __status_message (setq _status_message __status_message)) _status_message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _status_message
    4 (length _status_message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _status_message
       (write-long (length _status_message) s) (princ _status_message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _status_message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status_message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass px_comm::SetCameraInfo
  :super ros::object
  :slots ())

(setf (get px_comm::SetCameraInfo :md5sum-) "b7b33d05bd0d56b83943d2370771da4c")
(setf (get px_comm::SetCameraInfo :datatype-) "px_comm/SetCameraInfo")
(setf (get px_comm::SetCameraInfo :request) px_comm::SetCameraInfoRequest)
(setf (get px_comm::SetCameraInfo :response) px_comm::SetCameraInfoResponse)

(defmethod px_comm::SetCameraInfoRequest
  (:response () (instance px_comm::SetCameraInfoResponse :init)))

(setf (get px_comm::SetCameraInfoRequest :md5sum-) "b7b33d05bd0d56b83943d2370771da4c")
(setf (get px_comm::SetCameraInfoRequest :datatype-) "px_comm/SetCameraInfoRequest")
(setf (get px_comm::SetCameraInfoRequest :definition-)
      "







CameraInfo camera_info

================================================================================
MSG: px_comm/CameraInfo
#######################################################################
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
---
bool success
string status_message
")

(setf (get px_comm::SetCameraInfoResponse :md5sum-) "b7b33d05bd0d56b83943d2370771da4c")
(setf (get px_comm::SetCameraInfoResponse :datatype-) "px_comm/SetCameraInfoResponse")
(setf (get px_comm::SetCameraInfoResponse :definition-)
      "







CameraInfo camera_info

================================================================================
MSG: px_comm/CameraInfo
#######################################################################
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
---
bool success
string status_message
")



(provide :px_comm/SetCameraInfo "b7b33d05bd0d56b83943d2370771da4c")


