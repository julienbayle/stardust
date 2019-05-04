cmake_minimum_required(VERSION 2.8.3)
project(sd_behavior)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  behaviortree_cpp_v3
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
  ${catkin_INCLUDE_DIRS}
  include
  src
)

add_executable(maintree src/main.cpp)
target_link_libraries(maintree ${catkin_LIBRARIES} ${BEHAVIOR_TREE_LIBRARY} approachobject_nodes afficherlescore_nodes)
add_dependencies(maintree ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(approachobject_nodes STATIC src/ApproachObject.cpp)
target_link_libraries(approachobject_nodes ${catkin_LIBRARIES} ${BEHAVIOR_TREE_LIBRARY})
target_link_libraries(maintree approachobject_nodes)

add_library(isnottimeout_nodes STATIC src/IsNotTimeOut.cpp)
target_link_libraries(isnottimeout_nodes ${catkin_LIBRARIES} ${BEHAVIOR_TREE_LIBRARY})
target_link_libraries(maintree isnottimeout_nodes)

add_library(robot_sensors_nodes STATIC src/robot_sensors_nodes.cpp)
target_link_libraries(robot_sensors_nodes ${catkin_LIBRARIES} ${BEHAVIOR_TREE_LIBRARY})
target_link_libraries(maintree robot_sensors_nodes)

add_library(afficherlescore_nodes STATIC src/AfficherLeScore.cpp)
target_link_libraries(afficherlescore_nodes ${catkin_LIBRARIES} ${BEHAVIOR_TREE_LIBRARY})
target_link_libraries(maintree afficherlescore_nodes)

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
