# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "chess_tracking: 1 messages, 3 services")

set(MSG_I_FLAGS "-Ichess_tracking:/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(chess_tracking_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/msg/StringAndFloatsGrid.msg" NAME_WE)
add_custom_target(_chess_tracking_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "chess_tracking" "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/msg/StringAndFloatsGrid.msg" ""
)

get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/BoardString.srv" NAME_WE)
add_custom_target(_chess_tracking_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "chess_tracking" "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/BoardString.srv" ""
)

get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/TransformPoint.srv" NAME_WE)
add_custom_target(_chess_tracking_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "chess_tracking" "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/TransformPoint.srv" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/Screenshot.srv" NAME_WE)
add_custom_target(_chess_tracking_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "chess_tracking" "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/Screenshot.srv" "std_msgs/Header:sensor_msgs/Image"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(chess_tracking
  "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/msg/StringAndFloatsGrid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/chess_tracking
)

### Generating Services
_generate_srv_cpp(chess_tracking
  "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/BoardString.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/chess_tracking
)
_generate_srv_cpp(chess_tracking
  "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/TransformPoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/chess_tracking
)
_generate_srv_cpp(chess_tracking
  "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/Screenshot.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/chess_tracking
)

### Generating Module File
_generate_module_cpp(chess_tracking
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/chess_tracking
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(chess_tracking_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(chess_tracking_generate_messages chess_tracking_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/msg/StringAndFloatsGrid.msg" NAME_WE)
add_dependencies(chess_tracking_generate_messages_cpp _chess_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/BoardString.srv" NAME_WE)
add_dependencies(chess_tracking_generate_messages_cpp _chess_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/TransformPoint.srv" NAME_WE)
add_dependencies(chess_tracking_generate_messages_cpp _chess_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/Screenshot.srv" NAME_WE)
add_dependencies(chess_tracking_generate_messages_cpp _chess_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(chess_tracking_gencpp)
add_dependencies(chess_tracking_gencpp chess_tracking_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS chess_tracking_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(chess_tracking
  "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/msg/StringAndFloatsGrid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/chess_tracking
)

### Generating Services
_generate_srv_eus(chess_tracking
  "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/BoardString.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/chess_tracking
)
_generate_srv_eus(chess_tracking
  "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/TransformPoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/chess_tracking
)
_generate_srv_eus(chess_tracking
  "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/Screenshot.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/chess_tracking
)

### Generating Module File
_generate_module_eus(chess_tracking
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/chess_tracking
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(chess_tracking_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(chess_tracking_generate_messages chess_tracking_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/msg/StringAndFloatsGrid.msg" NAME_WE)
add_dependencies(chess_tracking_generate_messages_eus _chess_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/BoardString.srv" NAME_WE)
add_dependencies(chess_tracking_generate_messages_eus _chess_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/TransformPoint.srv" NAME_WE)
add_dependencies(chess_tracking_generate_messages_eus _chess_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/Screenshot.srv" NAME_WE)
add_dependencies(chess_tracking_generate_messages_eus _chess_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(chess_tracking_geneus)
add_dependencies(chess_tracking_geneus chess_tracking_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS chess_tracking_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(chess_tracking
  "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/msg/StringAndFloatsGrid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/chess_tracking
)

### Generating Services
_generate_srv_lisp(chess_tracking
  "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/BoardString.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/chess_tracking
)
_generate_srv_lisp(chess_tracking
  "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/TransformPoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/chess_tracking
)
_generate_srv_lisp(chess_tracking
  "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/Screenshot.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/chess_tracking
)

### Generating Module File
_generate_module_lisp(chess_tracking
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/chess_tracking
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(chess_tracking_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(chess_tracking_generate_messages chess_tracking_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/msg/StringAndFloatsGrid.msg" NAME_WE)
add_dependencies(chess_tracking_generate_messages_lisp _chess_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/BoardString.srv" NAME_WE)
add_dependencies(chess_tracking_generate_messages_lisp _chess_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/TransformPoint.srv" NAME_WE)
add_dependencies(chess_tracking_generate_messages_lisp _chess_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/Screenshot.srv" NAME_WE)
add_dependencies(chess_tracking_generate_messages_lisp _chess_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(chess_tracking_genlisp)
add_dependencies(chess_tracking_genlisp chess_tracking_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS chess_tracking_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(chess_tracking
  "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/msg/StringAndFloatsGrid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/chess_tracking
)

### Generating Services
_generate_srv_nodejs(chess_tracking
  "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/BoardString.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/chess_tracking
)
_generate_srv_nodejs(chess_tracking
  "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/TransformPoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/chess_tracking
)
_generate_srv_nodejs(chess_tracking
  "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/Screenshot.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/chess_tracking
)

### Generating Module File
_generate_module_nodejs(chess_tracking
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/chess_tracking
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(chess_tracking_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(chess_tracking_generate_messages chess_tracking_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/msg/StringAndFloatsGrid.msg" NAME_WE)
add_dependencies(chess_tracking_generate_messages_nodejs _chess_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/BoardString.srv" NAME_WE)
add_dependencies(chess_tracking_generate_messages_nodejs _chess_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/TransformPoint.srv" NAME_WE)
add_dependencies(chess_tracking_generate_messages_nodejs _chess_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/Screenshot.srv" NAME_WE)
add_dependencies(chess_tracking_generate_messages_nodejs _chess_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(chess_tracking_gennodejs)
add_dependencies(chess_tracking_gennodejs chess_tracking_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS chess_tracking_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(chess_tracking
  "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/msg/StringAndFloatsGrid.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/chess_tracking
)

### Generating Services
_generate_srv_py(chess_tracking
  "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/BoardString.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/chess_tracking
)
_generate_srv_py(chess_tracking
  "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/TransformPoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/chess_tracking
)
_generate_srv_py(chess_tracking
  "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/Screenshot.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/chess_tracking
)

### Generating Module File
_generate_module_py(chess_tracking
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/chess_tracking
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(chess_tracking_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(chess_tracking_generate_messages chess_tracking_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/msg/StringAndFloatsGrid.msg" NAME_WE)
add_dependencies(chess_tracking_generate_messages_py _chess_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/BoardString.srv" NAME_WE)
add_dependencies(chess_tracking_generate_messages_py _chess_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/TransformPoint.srv" NAME_WE)
add_dependencies(chess_tracking_generate_messages_py _chess_tracking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/Screenshot.srv" NAME_WE)
add_dependencies(chess_tracking_generate_messages_py _chess_tracking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(chess_tracking_genpy)
add_dependencies(chess_tracking_genpy chess_tracking_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS chess_tracking_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/chess_tracking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/chess_tracking
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(chess_tracking_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(chess_tracking_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(chess_tracking_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/chess_tracking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/chess_tracking
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(chess_tracking_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(chess_tracking_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(chess_tracking_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/chess_tracking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/chess_tracking
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(chess_tracking_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(chess_tracking_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(chess_tracking_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/chess_tracking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/chess_tracking
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(chess_tracking_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(chess_tracking_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(chess_tracking_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/chess_tracking)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/chess_tracking\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/chess_tracking
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(chess_tracking_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(chess_tracking_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(chess_tracking_generate_messages_py std_msgs_generate_messages_py)
endif()
