cmake_minimum_required(VERSION 2.8.3)
project(sd_localization)

find_package(catkin REQUIRED COMPONENTS
  robot_localization
  amcl
  dynamic_reconfigure
  laser_geometry
  pcl_ros
  pcl_conversions
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  SnapMapMonitor.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/SnapMap.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sd_localization
#  CATKIN_DEPENDS nav_msgs robot_localization roscpp sensor_msgs tf
#  DEPENDS system_lib
)

include_directories( ${catkin_INCLUDE_DIRS} )

add_executable( ${PROJECT_NAME}_snap_map src/sd_localization_snap_map.cpp )
target_link_libraries( ${PROJECT_NAME}_snap_map ${catkin_LIBRARIES} )
add_dependencies(${PROJECT_NAME}_snap_map 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable( odom_to_map src/odom_to_map.cpp )
target_link_libraries( odom_to_map ${catkin_LIBRARIES} )
add_dependencies(odom_to_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}_snap_map odom_to_map
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)