cmake_minimum_required(VERSION 3.5)
project(ubr1_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
