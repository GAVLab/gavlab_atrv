cmake_minimum_required(VERSION 2.8.3)
project(throttle_pointcloud)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  sensor_msgs
)

find_package(cmake_modules REQUIRED)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
      nodelet
      sensor_msgs
    DEPENDS system_lib
)

add_library(${PROJECT_NAME} src/cloud_throttle.cpp ${catkin_INCLUDE_DIRS})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Include headers
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install executable
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)