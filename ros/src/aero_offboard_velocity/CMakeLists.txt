cmake_minimum_required(VERSION 2.8.3)
project(aero_offboard_velocity)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  INCLUDE_DIRECTORIES include
)


add_executable(${PROJECT_NAME}_node 
 src/aero_offboard_velocity_node.cpp
 src/aero_drone_offboard.cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )

#############
## Install ##
#############

install(
  PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
