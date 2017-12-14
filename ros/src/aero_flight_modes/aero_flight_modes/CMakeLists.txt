cmake_minimum_required(VERSION 2.8.3)
project(aero_flight_modes)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  roscpp
)

catkin_package(
  CATKIN_DEPENDS mavros_msgs roscpp
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  INCLUDE_DIRECTORIES include
)

add_executable(${PROJECT_NAME}_node
  src/aero_flight_modes_node.cpp
  src/aero_drone_flight_modes.cpp
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
install(
  PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
