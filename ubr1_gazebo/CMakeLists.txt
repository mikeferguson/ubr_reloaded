cmake_minimum_required(VERSION 3.0.2)
project(ubr1_gazebo)

find_package(gazebo REQUIRED)

find_package(catkin
  REQUIRED
    control_toolbox
    gazebo_ros
    robot_controllers
    robot_controllers_interface
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
    robot_controllers
    robot_controllers_interface
  DEPENDS
    gazebo
)

add_library(UBR1GazeboPlugin src/ubr1_gazebo_plugin.cpp)
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

install(
  DIRECTORY
    config
    launch
    robots
    worlds
    models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  PROGRAMS
    scripts/stop_controllers.py
    test/pick_up_cube_test.py
    test/scripts/generate_grasp_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    test/pick_up_cube_trajectories.bag
    test/pick_up_cube_test.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES ubr1_gazebo_controllers.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS
    UBR1GazeboPlugin
    ubr1_gazebo_controllers
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
