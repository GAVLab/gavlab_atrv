cmake_minimum_required(VERSION 2.8.3)
project(gavlab_atrv_launch)

find_package(catkin REQUIRED COMPONENTS 
  gavlab_atrv_node
  lms1xx
  nodelet
  openni_camera
  throttle_pointcloud
  urdf
  tf
  joy
)

catkin_package(
  CATKIN_DEPENDS
    gavlab_atrv_node
    lms1xx
    nodelet
    openni_camera
    throttle_pointcloud
    urdf
    tf
    joy
)

install(
  DIRECTORY
    launch
    scripts
    displays
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)