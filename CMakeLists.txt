cmake_minimum_required(VERSION 2.8.3)
project(swarm_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros
  std_msgs
  mav_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  message_generation
  tf
 )

add_message_files(
    DIRECTORY msg
    FILES OdometryWithUavId.msg
)

generate_messages(DEPENDENCIES nav_msgs)
 
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp mavros std_msgs mav_msgs nav_msgs sensor_msgs geometry_msgs visualization_msgs message_runtime tf
)

include_directories(
	include 
	${catkin_INCLUDE_DIRS}
)

set(H_FILES
    include/Node.h
    include/UavControllerNode.h
    include/SwarmControllerNode.h
    include/MarkerPublisher.h
)

set(CPP_FILES
    src/Node.cpp
    src/UavControllerNode.cpp
    src/SwarmControllerNode.cpp
    src/MarkerPublisher.cpp
)

add_executable(uav_controller_node src/uav_controller_node.cpp ${CPP_FILES})
add_dependencies(uav_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(uav_controller_node ${catkin_LIBRARIES})

add_executable(swarm_controller_node src/swarm_controller_node.cpp ${CPP_FILES})
add_dependencies(swarm_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(swarm_controller_node ${catkin_LIBRARIES})

add_executable(marker_publisher src/marker_publisher.cpp ${CPP_FILES})
add_dependencies(marker_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(marker_publisher ${catkin_LIBRARIES})
