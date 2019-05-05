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

install(PROGRAMS
  scripts/sd_calibrate/eval_static_friction.py
  scripts/sd_calibrate/pid_tuning_move.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)