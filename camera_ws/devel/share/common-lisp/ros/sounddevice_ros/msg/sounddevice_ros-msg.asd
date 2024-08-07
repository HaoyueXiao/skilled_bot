
(cl:in-package :asdf)

(defsystem "sounddevice_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AudioData" :depends-on ("_package_AudioData"))
    (:file "_package_AudioData" :depends-on ("_package"))
    (:file "AudioInfo" :depends-on ("_package_AudioInfo"))
    (:file "_package_AudioInfo" :depends-on ("_package"))
  ))