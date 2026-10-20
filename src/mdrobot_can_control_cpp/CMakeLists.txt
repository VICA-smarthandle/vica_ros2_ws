cmake_minimum_required(VERSION 3.8)
project(mdrobot_can_control_cpp)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

add_library(mdrobot_socketcan STATIC src/socketcan.cpp)
target_include_directories(mdrobot_socketcan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(mdrobot_socketcan PUBLIC cxx_std_17)

add_executable(keyboard_knob src/keyboard_knob_node.cpp)
target_link_libraries(keyboard_knob mdrobot_socketcan)
ament_target_dependencies(keyboard_knob
  rclcpp geometry_msgs std_msgs diagnostic_msgs diagnostic_updater)

install(TARGETS keyboard_knob DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_motor_logic test/test_motor_logic.cpp)
  target_link_libraries(test_motor_logic mdrobot_socketcan)
  ament_add_gtest(test_motor_core test/test_motor_core.cpp)
  target_link_libraries(test_motor_core mdrobot_socketcan)
  ament_add_gtest(test_can_preflight test/test_can_preflight.cpp)
  target_link_libraries(test_can_preflight mdrobot_socketcan)

  add_executable(law_cpp test/parity/law_cpp.cpp)
  target_link_libraries(law_cpp mdrobot_socketcan)
  add_executable(preflight_cpp test/parity/preflight_cpp.cpp)
  target_link_libraries(preflight_cpp mdrobot_socketcan)
  ament_add_pytest_test(test_parity test/parity/test_parity.py
    ENV
      LAW_CPP=$<TARGET_FILE:law_cpp>
      PREFLIGHT_CPP=$<TARGET_FILE:preflight_cpp>
      MOTOR_PY_SRC=${CMAKE_CURRENT_SOURCE_DIR}/../mdrobot_can_control
    TIMEOUT 600)
endif()

ament_package()
