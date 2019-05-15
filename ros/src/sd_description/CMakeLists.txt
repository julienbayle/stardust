cmake_minimum_required(VERSION 2.8.3)
project(sd_description)

find_package(catkin REQUIRED
  ydlidar
)

catkin_package(
)

install(DIRECTORY
  launch
  urdf
  meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)