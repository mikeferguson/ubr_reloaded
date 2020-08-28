cmake_minimum_required(VERSION 3.5)
project(ubr1_navigation)

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_library(depth_layer SHARED
  src/depth_layer.cpp
)
ament_target_dependencies(depth_layer
  nav2_costmap_2d
  pluginlib
  sensor_msgs
  tf2_ros
)
target_link_libraries(depth_layer
  ${OpenCV_LIBRARIES}
)

install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}/
)

install(
  FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/tilt_head.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS depth_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)
ament_package()
