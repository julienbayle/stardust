cmake_minimum_required(VERSION 2.8.3)
project(sd_behavior)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  behaviortree_cpp_v3
  actionlib
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
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${BEHAVIOR_TREE_LIBRARY})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(gripper_nodes STATIC src/gripper_nodes.cpp)
target_link_libraries(gripper_nodes ${catkin_LIBRARIES} ${BEHAVIOR_TREE_LIBRARY})
target_link_libraries(${PROJECT_NAME} gripper_nodes)

add_library(move_nodes STATIC src/move_nodes.cpp)
target_link_libraries(move_nodes ${catkin_LIBRARIES} ${BEHAVIOR_TREE_LIBRARY})
add_dependencies(move_nodes ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} move_nodes)

add_library(score_nodes STATIC src/score_nodes.cpp)
target_link_libraries(score_nodes ${catkin_LIBRARIES} ${BEHAVIOR_TREE_LIBRARY})
target_link_libraries(${PROJECT_NAME} score_nodes)

add_library(sensors_nodes STATIC src/sensors_nodes.cpp)
target_link_libraries(sensors_nodes ${catkin_LIBRARIES} ${BEHAVIOR_TREE_LIBRARY})
target_link_libraries(${PROJECT_NAME} sensors_nodes)

add_library(timer_nodes STATIC src/timer_nodes.cpp)
target_link_libraries(timer_nodes ${catkin_LIBRARIES} ${BEHAVIOR_TREE_LIBRARY})
target_link_libraries(${PROJECT_NAME} timer_nodes)

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
#     display_nodes
#     gripper_nodes
#     move_nodes
    score_nodes
#     sensors_nodes
#     timer_nodes
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
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)