cmake_minimum_required(VERSION 3.8)
project(vica_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Header·Time 필드를 쓰는 msg용. 기존 msg 3종과 srv는 내장 타입만 써서 이 선언이 없었다.
# 현재 대상: TurnGuide, SmartHandleState, RobotEvent, RobotHealth, RobotFault, PersonDetection.
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Pose 필드를 쓰는 msg용. 현재 대상: PersonDetection.
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
  "srv/RequestApproach.srv"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs
)

ament_package()
