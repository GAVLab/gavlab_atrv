cmake_minimum_required(VERSION 2.8.3)
project(gavlab_atrv_core)

find_package(catkin REQUIRED COMPONENTS
  mdc2250
)

find_package(cmake_modules REQUIRED)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    CATKIN_DEPENDS mdc2250
    DEPENDS system_lib
)

## Sources
set(gavlab_atrv_core_SRCS
    src/atrv.cc
    include/atrv/atrv.h
)

add_library(${PROJECT_NAME} ${gavlab_atrv_node_SRCS} ${catkin_INCLUDE_DIRS})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE C)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Uncomment for example
# add_executable(atrv_example examples/atrv_example.cc)
# add_dependencies(atrv_example ${PROJECT_NAME})
# target_link_libraries(atrv_example ${PROJECT_NAME})

## Include headers
include_directories(
	include
	include/${PROJECT_NAME}
	${catkin_INCLUDE_DIRS}
)

## Install executable
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Install headers
install(FILES include/atrv/atrv.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/gavlab_atrv_core)

## Tests
# if(CATKIN_ENABLE_TESTING)
#     add_subdirectory(tests)
# endif()