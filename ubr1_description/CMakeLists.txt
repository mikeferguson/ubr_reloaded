cmake_minimum_required(VERSION 2.8.3)
project(ubr1_description)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY robots
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY meshes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
