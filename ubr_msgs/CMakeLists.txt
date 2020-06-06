cmake_minimum_required(VERSION 3.5)
project(ubr_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoardInfo.msg"
  "msg/BreakerState.msg"
  "msg/BreakerInfo.msg"
  "msg/JointInfo.msg"
  "msg/MotorInfo.msg"
  "msg/RobotState.msg"
  "msg/DebugInfo.msg"
  "msg/ChargerInfo.msg"
  "srv/BreakerCommand.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
