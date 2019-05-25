cmake_minimum_required(VERSION 2.8.3)
project(sd_sensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  ydlidar
  message_generation
  sd_sensor_msgs
)

add_service_files(
  FILES
  GetBias.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sd_sensor
#  CATKIN_DEPENDS roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Laser filter
add_library(sd_sensor_filters src/sd_sensors_filters.cpp)
add_dependencies(sd_sensor_filters ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sd_sensor_filters ${catkin_LIBRARIES} )

## Sonar to point cloud
add_executable(r2_sensor_node src/r2_sensor_node.cpp)
add_dependencies(r2_sensor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(r2_sensor_node ${catkin_LIBRARIES} )

## IMU 
add_executable(mpu6050_node src/mpu6050_node.cpp )
add_dependencies(mpu6050_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpu6050_node ${catkin_LIBRARIES} i2c)

add_executable(mpu6050_fifo_node src/mpu6050_fifo_node.cpp )
add_dependencies(mpu6050_fifo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpu6050_fifo_node ${catkin_LIBRARIES} i2c)

## Laser pattern detector 
add_executable(laser_pattern_detector src/laser_pattern_detector.cpp )
add_dependencies(laser_pattern_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(laser_pattern_detector ${catkin_LIBRARIES})

install(TARGETS sd_sensor_filters r2_sensor_node mpu6050_node mpu6050_fifo_node laser_pattern_detector 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES
  sd_sensor_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)