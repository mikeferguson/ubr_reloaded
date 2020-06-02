cmake_minimum_required(VERSION 3.0.2)
project(ubr_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
)

add_message_files(
  FILES
    BoardInfo.msg
    BreakerState.msg
    BreakerInfo.msg
    JointInfo.msg
    MotorInfo.msg
    RobotState.msg
    DebugInfo.msg
    ChargerInfo.msg
)

add_service_files(
  FILES
    BreakerCommand.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
