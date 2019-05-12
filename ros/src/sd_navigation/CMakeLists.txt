cmake_minimum_required(VERSION 2.8.3)
project(sd_navigation)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  map_server
  move_base
  teb_local_planner
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sd_navigation
#  CATKIN_DEPENDS costmap_2d map_server move_base teb_local_planner
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)