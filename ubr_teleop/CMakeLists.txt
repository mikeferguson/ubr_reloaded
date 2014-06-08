cmake_minimum_required(VERSION 2.8.3)
project(ubr_teleop)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED roscpp sensor_msgs control_msgs geometry_msgs actionlib topic_tools)
catkin_package()

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(keyboard_teleop src/keyboard_teleop.cpp)
target_link_libraries(keyboard_teleop ${Boost_LIBRARIES}
                                      ${catkin_LIBRARIES})

add_executable(joystick_teleop src/joystick_teleop.cpp)
target_link_libraries(joystick_teleop ${Boost_LIBRARIES}
                                      ${catkin_LIBRARIES})

install(TARGETS keyboard_teleop joystick_teleop
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
