;; Auto-generated. Do not edit!


(when (boundp 'sounddevice_ros::AudioInfo)
  (if (not (find-package "SOUNDDEVICE_ROS"))
    (make-package "SOUNDDEVICE_ROS"))
  (shadow 'AudioInfo (find-package "SOUNDDEVICE_ROS")))
(unless (find-package "SOUNDDEVICE_ROS::AUDIOINFO")
  (make-package "SOUNDDEVICE_ROS::AUDIOINFO"))

(in-package "ROS")
;;//! \htmlinclude AudioInfo.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass sounddevice_ros::AudioInfo
  :super ros::object
  :slots (_header _num_channels _sample_rate _subtype ))

(defmethod sounddevice_ros::AudioInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:num_channels __num_channels) 0)
    ((:sample_rate __sample_rate) 0)
    ((:subtype __subtype) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _num_channels (round __num_channels))
   (setq _sample_rate (round __sample_rate))
   (setq _subtype (string __subtype))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:num_channels
   (&optional __num_channels)
   (if __num_channels (setq _num_channels __num_channels)) _num_channels)
  (:sample_rate
   (&optional __sample_rate)
   (if __sample_rate (setq _sample_rate __sample_rate)) _sample_rate)
  (:subtype
   (&optional __subtype)
   (if __subtype (setq _subtype __subtype)) _subtype)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _num_channels
    4
    ;; int32 _sample_rate
    4
    ;; string _subtype
    4 (length _subtype)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _num_channels
       (write-long _num_channels s)
     ;; int32 _sample_rate
       (write-long _sample_rate s)
     ;; string _subtype
       (write-long (length _subtype) s) (princ _subtype s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _num_channels
     (setq _num_channels (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _sample_rate
     (setq _sample_rate (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _subtype
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _subtype (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get sounddevice_ros::AudioInfo :md5sum-) "d52fdd030548864e37e9bab9114e6549")
(setf (get sounddevice_ros::AudioInfo :datatype-) "sounddevice_ros/AudioInfo")
(setf (get sounddevice_ros::AudioInfo :definition-)
      "Header header
int32 num_channels
int32 sample_rate
string subtype
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
string frame_id

")



(provide :sounddevice_ros/AudioInfo "d52fdd030548864e37e9bab9114e6549")


