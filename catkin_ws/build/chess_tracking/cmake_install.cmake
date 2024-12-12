# Install script for directory: /home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/chess_tracking/msg" TYPE FILE FILES "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/msg/StringAndFloatsGrid.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/chess_tracking/srv" TYPE FILE FILES
    "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/BoardString.srv"
    "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/TransformPoint.srv"
    "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/srv/Screenshot.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/chess_tracking/cmake" TYPE FILE FILES "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/build/chess_tracking/catkin_generated/installspace/chess_tracking-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/devel/include/chess_tracking")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/devel/share/roseus/ros/chess_tracking")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/devel/share/common-lisp/ros/chess_tracking")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/devel/share/gennodejs/ros/chess_tracking")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/devel/lib/python3/dist-packages/chess_tracking")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/devel/lib/python3/dist-packages/chess_tracking")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/build/chess_tracking/catkin_generated/installspace/chess_tracking.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/chess_tracking/cmake" TYPE FILE FILES "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/build/chess_tracking/catkin_generated/installspace/chess_tracking-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/chess_tracking/cmake" TYPE FILE FILES
    "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/build/chess_tracking/catkin_generated/installspace/chess_trackingConfig.cmake"
    "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/build/chess_tracking/catkin_generated/installspace/chess_trackingConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/chess_tracking" TYPE FILE FILES "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/src/chess_tracking/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/chess_tracking" TYPE PROGRAM FILES "/home/cc/ee106a/fa24/class/ee106a-aho/GM-Sawyer/catkin_ws/build/chess_tracking/catkin_generated/installspace/board_service.py")
endif()

