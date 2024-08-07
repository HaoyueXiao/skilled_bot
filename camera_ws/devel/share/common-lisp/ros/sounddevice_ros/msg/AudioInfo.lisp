; Auto-generated. Do not edit!


(cl:in-package sounddevice_ros-msg)


;//! \htmlinclude AudioInfo.msg.html

(cl:defclass <AudioInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (num_channels
    :reader num_channels
    :initarg :num_channels
    :type cl:integer
    :initform 0)
   (sample_rate
    :reader sample_rate
    :initarg :sample_rate
    :type cl:integer
    :initform 0)
   (subtype
    :reader subtype
    :initarg :subtype
    :type cl:string
    :initform ""))
)

(cl:defclass AudioInfo (<AudioInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AudioInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AudioInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sounddevice_ros-msg:<AudioInfo> is deprecated: use sounddevice_ros-msg:AudioInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AudioInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sounddevice_ros-msg:header-val is deprecated.  Use sounddevice_ros-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'num_channels-val :lambda-list '(m))
(cl:defmethod num_channels-val ((m <AudioInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sounddevice_ros-msg:num_channels-val is deprecated.  Use sounddevice_ros-msg:num_channels instead.")
  (num_channels m))

(cl:ensure-generic-function 'sample_rate-val :lambda-list '(m))
(cl:defmethod sample_rate-val ((m <AudioInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sounddevice_ros-msg:sample_rate-val is deprecated.  Use sounddevice_ros-msg:sample_rate instead.")
  (sample_rate m))

(cl:ensure-generic-function 'subtype-val :lambda-list '(m))
(cl:defmethod subtype-val ((m <AudioInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sounddevice_ros-msg:subtype-val is deprecated.  Use sounddevice_ros-msg:subtype instead.")
  (subtype m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AudioInfo>) ostream)
  "Serializes a message object of type '<AudioInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'num_channels)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'sample_rate)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'subtype))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'subtype))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AudioInfo>) istream)
  "Deserializes a message object of type '<AudioInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_channels) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sample_rate) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'subtype) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'subtype) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AudioInfo>)))
  "Returns string type for a message object of type '<AudioInfo>"
  "sounddevice_ros/AudioInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AudioInfo)))
  "Returns string type for a message object of type 'AudioInfo"
  "sounddevice_ros/AudioInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AudioInfo>)))
  "Returns md5sum for a message object of type '<AudioInfo>"
  "d52fdd030548864e37e9bab9114e6549")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AudioInfo)))
  "Returns md5sum for a message object of type 'AudioInfo"
  "d52fdd030548864e37e9bab9114e6549")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AudioInfo>)))
  "Returns full string definition for message of type '<AudioInfo>"
  (cl:format cl:nil "Header header~%int32 num_channels~%int32 sample_rate~%string subtype~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AudioInfo)))
  "Returns full string definition for message of type 'AudioInfo"
  (cl:format cl:nil "Header header~%int32 num_channels~%int32 sample_rate~%string subtype~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AudioInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4 (cl:length (cl:slot-value msg 'subtype))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AudioInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'AudioInfo
    (cl:cons ':header (header msg))
    (cl:cons ':num_channels (num_channels msg))
    (cl:cons ':sample_rate (sample_rate msg))
    (cl:cons ':subtype (subtype msg))
))
