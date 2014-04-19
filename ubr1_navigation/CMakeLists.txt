cmake_minimum_required(VERSION 2.8.3)
project(ubr1_navigation)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
)

add_service_files(
  FILES
    TiltControl.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
    message_runtime
)

install(
  DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  PROGRAMS scripts/tilt_head.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
