cmake_minimum_required(VERSION 3.8)
project(vica_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VicaIntent.msg"
  "msg/RobotState.msg"
  "msg/EmergencyEvent.msg"
  "msg/TurnGuide.msg"
  "msg/SmartHandleState.msg"
  "msg/RobotFault.msg"
  "msg/RobotEvent.msg"
  "msg/RobotHealth.msg"
  "msg/PersonDetection.msg"
  "srv/RequestDestination.srv"
  "srv/MissionCommand.srv"
  "srv/DeleteMap.srv"
  "srv/SaveMap.srv"
  "srv/PoseCheck.srv"
  "srv/PoseCommit.srv"
  "srv/RequestApproach.srv"
  "srv/SaveHome.srv"
  "srv/GetHome.srv"
  "srv/DeleteHome.srv"
  "srv/SaveKeepout.srv"
  "srv/GetKeepout.srv"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs
)

ament_package()
