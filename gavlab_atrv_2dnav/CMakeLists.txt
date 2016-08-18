cmake_minimum_required(VERSION 2.4.6)
project(gavlab_atrv_2dnav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  move_base
  nav_msgs
  sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS 
      rospy
	  tf
	  move_base
	  nav_msgs
	  sensor_msgs
    DEPENDS system_lib
)

## Include headers
include_directories(
	${catkin_INCLUDE_DIRS}
)

## Install executable
install(PROGRAMS
  scripts/laser_filter.py
  scripts/odom_to_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)