cmake_minimum_required(VERSION 2.8.3)
project(sd_behavior)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  behaviortree_cpp_v3
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sd_behavior
#  CATKIN_DEPENDS rospy sensor_msgs smach
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ./include/
  ./src/
)

add_executable(maintree src/main.cpp)
target_link_libraries(maintree ${catkin_LIBRARIES} ${BEHAVIOR_TREE_LIBRARY} approachobject_nodes)

add_library(approachobject_nodes STATIC src/ApproachObject.cpp)
target_link_libraries(approachobject_nodes ${catkin_LIBRARIES} ${BEHAVIOR_TREE_LIBRARY})
target_link_libraries(maintree approachobject_nodes)

add_library(robot_sensors_nodes STATIC src/robot_sensors_nodes.cpp)
target_link_libraries(robot_sensors_nodes ${catkin_LIBRARIES} ${BEHAVIOR_TREE_LIBRARY})
target_link_libraries(maintree robot_sensors_nodes)

#############
## Install ##
#############

install(PROGRAMS
  scripts/sd_behavior_base_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)