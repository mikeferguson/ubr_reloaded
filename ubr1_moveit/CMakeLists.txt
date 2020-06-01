cmake_minimum_required(VERSION 3.0.2)
project(ubr1_moveit)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
