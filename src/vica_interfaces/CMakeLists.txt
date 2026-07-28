cmake_minimum_required(VERSION 3.8)
project(vica_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# TurnGuide가 std_msgs/Header와 builtin_interfaces/Time을 쓰므로 필요하다.
# 기존 msg 3종과 srv는 내장 타입만 써서 이 선언이 없었다.
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VicaIntent.msg"
  "msg/RobotState.msg"
  "msg/EmergencyEvent.msg"
  "msg/TurnGuide.msg"
  "msg/SmartHandleState.msg"
  "srv/RequestDestination.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_package()
