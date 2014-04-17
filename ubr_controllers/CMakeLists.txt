cmake_minimum_required(VERSION 2.8.3)
project(ubr_controllers)

find_package(orocos_kdl REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    actionlib_msgs
    control_msgs
    geometry_msgs
    kdl_parser
    nav_msgs
    pluginlib
    roscpp
    sensor_msgs
    tf
    tf_conversions
    trajectory_msgs
    urdf
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    control_msgs
    pluginlib
    roscpp
    sensor_msgs
  LIBRARIES
    ubr_controllers
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

# this is a hack, will eventually be unneeded once orocos-kdl is fixed
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_library(ubr_controllers
  src/base_controller.cpp
  src/follow_joint_trajectory.cpp
  src/gravity_compensation.cpp
  src/point_head.cpp
)
target_link_libraries(ubr_controllers
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

install(DIRECTORY include/ DESTINATION include)

install(FILES ubr_controllers.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ubr_controllers LIBRARY
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
