execute_process(COMMAND "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/build/sawyer_full_stack/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/build/sawyer_full_stack/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
