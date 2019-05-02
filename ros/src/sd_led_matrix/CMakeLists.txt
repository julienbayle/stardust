cmake_minimum_required(VERSION 2.8.3)
project(sd_led_matrix)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  Eye.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

install(PROGRAMS
  scripts/sd_led_matrix/led_matrix.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  images
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)