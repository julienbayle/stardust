cmake_minimum_required(VERSION 2.8.3)
project(sd_calibration)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS message_runtime 
)

