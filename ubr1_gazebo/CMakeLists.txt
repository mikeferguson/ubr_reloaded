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
    ubr_controllers
#    INCLUDE_DIRS
#    LIBRARIES
)

add_library(UBR1GazeboPlugin src/ubr1_gazebo_plugin.cpp)
add_dependencies(UBR1GazeboPlugin ${PROJECT_NAME}_gencfg)
target_link_libraries(UBR1GazeboPlugin
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_library(ubr1_gazebo_controllers
  src/simulated_bellows_controller.cpp
  src/simulated_gripper_controller.cpp
)
target_link_libraries(ubr1_gazebo_controllers
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(DIRECTORY
  launch robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES ubr1_gazebo_controllers.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS
  UBR1GazeboPlugin ubr1_gazebo_controllers
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
