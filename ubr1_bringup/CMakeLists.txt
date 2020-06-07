cmake_minimum_required(VERSION 3.5)
project(ubr1_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

#install(
#  PROGRAMS scripts/controller_reset.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

ament_package()
