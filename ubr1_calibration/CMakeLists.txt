cmake_minimum_required(VERSION 3.5)
project(ubr1_calibration)

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
  PROGRAMS scripts/camera_reconfigure.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
