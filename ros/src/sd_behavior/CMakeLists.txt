cmake_minimum_required(VERSION 2.8.3)
project(sd_behavior)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  sd_led_matrix
  geometry_msgs
  behaviortree_cpp_v3
  actionlib
  tf
  sd_sensor_msgs
  rosbag
  rosbag_storage
  topic_tools
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${BEHAVIOR_TREE_LIBRARY})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_nodes 
  src/gripper_nodes.cpp
  src/laser_nodes.cpp
  src/move_nodes.cpp
  src/score_nodes.cpp
  src/sensors_nodes.cpp
  src/status_nodes.cpp
  src/timer_nodes.cpp
  src/rosbag_nodes.cpp
  src/eye_nodes.cpp
  src/control_nodes.cpp)
add_dependencies(${PROJECT_NAME}_nodes ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_nodes ${catkin_LIBRARIES} ${BEHAVIOR_TREE_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_nodes)

#############
## Install ##
#############

install(
  TARGETS 
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS 
    ${PROJECT_NAME}_nodes
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  PROGRAMS
    scripts/sd_behavior_base_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    config
    launch
    bags
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)