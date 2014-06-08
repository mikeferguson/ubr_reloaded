cmake_minimum_required(VERSION 2.8.3)
project(ubr1_gazebo)

find_package(gazebo REQUIRED)

find_package(catkin
  REQUIRED
    control_toolbox
    gazebo_ros
    ubr_controllers
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    control_toolbox
    gazebo_ros
    ubr_controllers
  DEPENDS
    gazebo
#    INCLUDE_DIRS
#    LIBRARIES
)

add_library(UBR1GazeboPlugin src/ubr1_gazebo_plugin.cpp)
add_dependencies(UBR1GazeboPlugin ${PROJECT_NAME}_gencfg)
target_link_libraries(UBR1GazeboPlugin
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(DIRECTORY
  launch robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS
  UBR1GazeboPlugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
