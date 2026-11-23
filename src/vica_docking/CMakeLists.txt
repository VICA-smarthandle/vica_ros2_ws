cmake_minimum_required(VERSION 3.8)
project(vica_docking)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vica_interfaces REQUIRED)

add_library(vica_docking_core STATIC
  src/v_detector.cpp
  src/scan_zones.cpp
  src/approach.cpp
  src/undock.cpp
  src/dock_spot.cpp)
target_include_directories(vica_docking_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(vica_docking_core PUBLIC cxx_std_17)
set_target_properties(vica_docking_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(vica_docking_node src/docking_node.cpp)
target_link_libraries(vica_docking_node vica_docking_core)
ament_target_dependencies(vica_docking_node
  rclcpp std_msgs std_srvs sensor_msgs geometry_msgs vica_interfaces)

add_executable(dock_replay test/parity/dock_replay.cpp)
target_link_libraries(dock_replay vica_docking_core)

install(TARGETS vica_docking_node dock_replay DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_docking_core test/test_docking_core.cpp)
  target_link_libraries(test_docking_core vica_docking_core)
  ament_add_pytest_test(test_parity test/parity/test_parity.py
    ENV DOCK_REPLAY=$<TARGET_FILE:dock_replay>
    TIMEOUT 300)
endif()

ament_package()
