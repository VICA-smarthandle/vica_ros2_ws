cmake_minimum_required(VERSION 3.8)
project(vica_vcc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 순수 계산은 ROS 에 의존하지 않는다 — 시험이 노드·TF 없이 돈다(vica_nav2_bt_plugins 와 같은 모양).
add_library(vcc_core STATIC
  src/core/geometry.cpp
  src/core/clearance.cpp
  src/core/pure_pursuit.cpp
  src/core/speed_profile.cpp
)
target_include_directories(vcc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(vcc_core PUBLIC cxx_std_17)
set_target_properties(vcc_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS vcc_core ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_link_libraries(test_geometry vcc_core)

  ament_add_gtest(test_clearance test/test_clearance.cpp)
  target_link_libraries(test_clearance vcc_core)

  ament_add_gtest(test_pure_pursuit test/test_pure_pursuit.cpp)
  target_link_libraries(test_pure_pursuit vcc_core)

  ament_add_gtest(test_speed_profile test/test_speed_profile.cpp)
  target_link_libraries(test_speed_profile vcc_core)
endif()

ament_package()
