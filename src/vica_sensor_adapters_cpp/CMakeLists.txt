cmake_minimum_required(VERSION 3.8)
project(vica_sensor_adapters_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 순수 로직은 ROS 에 의존하지 않는다 — 시험이 노드 없이 돈다.
add_library(gyro_bias src/gyro_bias.cpp)
target_include_directories(gyro_bias PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(gyro_bias PUBLIC cxx_std_17)

# 넘어짐 판정(2026-10-15)도 같은 이유로 노드 밖에 둔다.
add_library(tilt_detector src/tilt_detector.cpp)
target_include_directories(tilt_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tilt_detector PUBLIC cxx_std_17)

add_executable(imu_base_link_adapter src/imu_base_link_adapter.cpp)
target_link_libraries(imu_base_link_adapter gyro_bias tilt_detector)
ament_target_dependencies(imu_base_link_adapter
  rclcpp sensor_msgs std_msgs geometry_msgs tf2 tf2_ros)

install(TARGETS gyro_bias tilt_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS imu_base_link_adapter DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gyro_bias test/test_gyro_bias.cpp)
  target_link_libraries(test_gyro_bias gyro_bias)
  ament_add_gtest(test_tilt_detector test/test_tilt_detector.cpp)
  target_link_libraries(test_tilt_detector tilt_detector)
endif()

ament_package()
