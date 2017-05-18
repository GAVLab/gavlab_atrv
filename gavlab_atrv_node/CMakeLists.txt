cmake_minimum_required(VERSION 2.4.6)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(gavlab_atrv_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  message_generation
  geometry_msgs
  nav_msgs
  gavlab_atrv_core
)

find_package(cmake_modules REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    StampedEncoders.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS 
      roscpp
      tf
      message_generation
      message_runtime
      geometry_msgs
      nav_msgs
      gavlab_atrv_core
    DEPENDS system_lib
)

add_library(${PROJECT_NAME} src/atrv_node.cc ${catkin_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Include headers
include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/atrv_node.cc)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Install executable
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install headers
# install(FILES include/atrv/atrv.h DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/gavlab_atrv_node)
