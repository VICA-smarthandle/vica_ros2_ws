cmake_minimum_required(VERSION 3.8)
project(vica_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VicaIntent.msg"
  "msg/RobotState.msg"
  "msg/EmergencyEvent.msg"
)

ament_package()
