cmake_minimum_required(VERSION 2.4.6)
project(gavlab_atrv_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
)

catkin_package(
    CATKIN_DEPENDS urdf
    DEPENDS system_lib
)

## Install executable
# install(PROGRAMS
#   scripts/laser_filter.py
#   scripts/odom_to_tf.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )