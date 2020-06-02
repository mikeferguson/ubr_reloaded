cmake_minimum_required(VERSION 2.8.12)
project(ubr_calibration)

find_package(catkin)
catkin_package()

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  PROGRAMS scripts/camera_reconfigure.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
