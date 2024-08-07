execute_process(COMMAND "/home/jianrenw/Documents/skilled/cobot_magic/camera_ws/build/audio_common/sound_play/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/jianrenw/Documents/skilled/cobot_magic/camera_ws/build/audio_common/sound_play/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
