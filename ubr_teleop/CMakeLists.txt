cmake_minimum_required(VERSION 3.5)
project(ubr_teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(keyboard_teleop src/keyboard_teleop.cpp)
ament_target_dependencies(keyboard_teleop
  control_msgs
  geometry_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
)

add_executable(joystick_teleop src/joystick_teleop.cpp)
ament_target_dependencies(joystick_teleop
  control_msgs
  geometry_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
)

install(
  TARGETS
    keyboard_teleop
    joystick_teleop
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/set_gripper_pose.py
    scripts/set_head_pose.py
    scripts/set_torso_pose.py
    scripts/tuck_arm.py
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  control_msgs
  geometry_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
)
ament_package()

