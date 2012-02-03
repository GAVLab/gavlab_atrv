cmake_minimum_required(VERSION 2.4.6)

set(ROS_ROOT $ENV{ROS_ROOT})

option(ATRV_BUILD_WIHOUT_ROS "Build without ROS?" OFF)

if(DEFINED ROS_ROOT AND NOT ATRV_BUILD_WIHOUT_ROS)
  # Build with ROS
  include(atrv_ros.cmake)
  message("Building with ROS")
  build_atrv()
else(DEFINED ROS_ROOT AND NOT ATRV_BUILD_WIHOUT_ROS)
  # Build normally
  include(atrv.cmake)
  message("Building stand alone")
  build_atrv()
endif(DEFINED ROS_ROOT AND NOT ATRV_BUILD_WIHOUT_ROS)
