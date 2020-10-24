;; Auto-generated. Do not edit!


(when (boundp 'px_comm::Mavlink)
  (if (not (find-package "PX_COMM"))
    (make-package "PX_COMM"))
  (shadow 'Mavlink (find-package "PX_COMM")))
(unless (find-package "PX_COMM::MAVLINK")
  (make-package "PX_COMM::MAVLINK"))

(in-package "ROS")
;;//! \htmlinclude Mavlink.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass px_comm::Mavlink
  :super ros::object
  :slots (_header _len _seq _sysid _compid _msgid _fromlcm _payload64 ))

(defmethod px_comm::Mavlink
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:len __len) 0)
    ((:seq __seq) 0)
    ((:sysid __sysid) 0)
    ((:compid __compid) 0)
    ((:msgid __msgid) 0)
    ((:fromlcm __fromlcm) nil)
    ((:payload64 __payload64) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _header __header)
   (setq _len (round __len))
   (setq _seq (round __seq))
   (setq _sysid (round __sysid))
   (setq _compid (round __compid))
   (setq _msgid (round __msgid))
   (setq _fromlcm __fromlcm)
   (setq _payload64 __payload64)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:len
   (&optional __len)
   (if __len (setq _len __len)) _len)
  (:seq
   (&optional __seq)
   (if __seq (setq _seq __seq)) _seq)
  (:sysid
   (&optional __sysid)
   (if __sysid (setq _sysid __sysid)) _sysid)
  (:compid
   (&optional __compid)
   (if __compid (setq _compid __compid)) _compid)
  (:msgid
   (&optional __msgid)
   (if __msgid (setq _msgid __msgid)) _msgid)
  (:fromlcm
   (&optional __fromlcm)
   (if __fromlcm (setq _fromlcm __fromlcm)) _fromlcm)
  (:payload64
   (&optional __payload64)
   (if __payload64 (setq _payload64 __payload64)) _payload64)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _len
    1
    ;; uint8 _seq
    1
    ;; uint8 _sysid
    1
    ;; uint8 _compid
    1
    ;; uint8 _msgid
    1
    ;; bool _fromlcm
    1
    ;; uint64[] _payload64
    (* 8    (length _payload64)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _len
       (write-byte _len s)
     ;; uint8 _seq
       (write-byte _seq s)
     ;; uint8 _sysid
       (write-byte _sysid s)
     ;; uint8 _compid
       (write-byte _compid s)
     ;; uint8 _msgid
       (write-byte _msgid s)
     ;; bool _fromlcm
       (if _fromlcm (write-byte -1 s) (write-byte 0 s))
     ;; uint64[] _payload64
     (write-long (length _payload64) s)
     (dotimes (i (length _payload64))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _payload64 i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _payload64 i)) (= (length ((elt _payload64 i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _payload64 i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _payload64 i) . bv) 1) -1) s))
             ((and (class (elt _payload64 i)) (= (length ((elt _payload64 i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _payload64 i) . bv) 0) s)
              (write-long (if (>= (elt _payload64 i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _payload64 i) s)(write-long (if (>= (elt _payload64 i) 0) 0 #xffffffff) s)))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _len
     (setq _len (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _seq
     (setq _seq (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _sysid
     (setq _sysid (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _compid
     (setq _compid (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _msgid
     (setq _msgid (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _fromlcm
     (setq _fromlcm (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint64[] _payload64
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _payload64 (instantiate integer-vector n))
     (dotimes (i n)
     
#+(or :alpha :irix6 :x86_64)
      (setf (elt _payload64 i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _payload64 i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;;
   self)
  )

(setf (get px_comm::Mavlink :md5sum-) "3513d96e87d97146fed4643086047ddd")
(setf (get px_comm::Mavlink :datatype-) "px_comm/Mavlink")
(setf (get px_comm::Mavlink :definition-)
      "Header header
uint8 len
uint8 seq
uint8 sysid
uint8 compid
uint8 msgid
bool fromlcm
uint64[] payload64

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



(provide :px_comm/Mavlink "3513d96e87d97146fed4643086047ddd")


