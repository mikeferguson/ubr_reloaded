cmake_minimum_required(VERSION 2.8.3)
project(ubr1_demo)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
)

link_directories(${catkin_LIBRARY_DIRS})

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
)
