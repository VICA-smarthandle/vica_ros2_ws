cmake_minimum_required(VERSION 3.8)
project(vica_sensor_adapters_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(gyro_bias src/gyro_bias.cpp)
target_include_directories(gyro_bias PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(gyro_bias PUBLIC cxx_std_17)

add_executable(imu_base_link_adapter src/imu_base_link_adapter.cpp)
target_link_libraries(imu_base_link_adapter gyro_bias)
ament_target_dependencies(imu_base_link_adapter
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros)

install(TARGETS gyro_bias
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS imu_base_link_adapter DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gyro_bias test/test_gyro_bias.cpp)
  target_link_libraries(test_gyro_bias gyro_bias)
endif()

ament_package()
