cmake_minimum_required(VERSION 2.8.3)
project(swarm_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros
  mav_msgs
  nav_msgs
  geometry_msgs
 )
 
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp mavros mav_msgs nav_msgs geometry_msgs
)

include_directories(
	include 
	${catkin_INCLUDE_DIRS}
)

set(H_FILES
	#include/Node.h
)

set(CPP_FILES
	#src/Node.cpp
)

add_executable(uav_control_node src/uav_control_node.cpp ${CPP_FILES})
add_dependencies(uav_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(uav_control_node ${catkin_LIBRARIES})
