cmake_minimum_required(VERSION 3.8)
project(ubr1_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(robot_controllers REQUIRED)
find_package(robot_controllers_interface REQUIRED)
find_package(simple_actions REQUIRED)

set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})
set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
set(GZ_PLUGIN ignition-plugin${GZ_PLUGIN_VER}::register)
set(GZ_SIM ignition-gazebo${GZ_SIM_VER}::core)

#add_library(UBR1GazeboPlugin src/ubr1_gazebo_plugin.cpp)
#target_link_libraries(UBR1GazeboPlugin
#  ${Boost_LIBRARIES}
#  ${GAZEBO_LIBRARIES}
#)
#target_include_directories(UBR1GazeboPlugin PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#ament_target_dependencies(
#  UBR1GazeboPlugin
#  control_toolbox
#  robot_controllers
#  robot_controllers_interface
#  ros_gz
#)

add_library(ubr1_gazebo_controllers
  src/simulated_bellows_controller.cpp
  src/simulated_gripper_controller.cpp
)
target_include_directories(ubr1_gazebo_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ubr1_gazebo_controllers
  ${GZ_SIM}
  ${GZ_PLUGIN}
)
ament_target_dependencies(
  ubr1_gazebo_controllers
  angles
  control_toolbox
  pluginlib
  robot_controllers
  robot_controllers_interface
  simple_actions
)

install(
  DIRECTORY config launch robots worlds models
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/stop_controllers.py
    test/pick_up_cube_test.py
    test/scripts/generate_grasp_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    test/pick_up_cube_trajectories.bag
    test/pick_up_cube_test.launch
    ubr1_gazebo_controllers.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
#    UBR1GazeboPlugin
    ubr1_gazebo_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PACKAGE_NAME}
)

ament_package()