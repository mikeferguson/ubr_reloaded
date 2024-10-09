cmake_minimum_required(VERSION 3.8)
project(ubr1_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(grasping_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(simple_actions REQUIRED)

add_executable(pick_and_place
  src/generate_grasps_from_msg.cpp
  src/pick_and_place.cpp
  src/pick_place_task.cpp
)
target_include_directories(pick_and_place PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pick_and_place PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pick_and_place
  grasping_msgs
  moveit_task_constructor_core
  rclcpp
  simple_actions
)

install(TARGETS pick_and_place
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
