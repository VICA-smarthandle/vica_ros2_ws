cmake_minimum_required(VERSION 3.8)
project(vica_vcc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 순수 계산은 ROS 에 의존하지 않는다 — 시험이 노드·TF 없이 돈다(vica_nav2_bt_plugins 와 같은 모양).
add_library(vcc_core STATIC
  src/core/geometry.cpp
  src/core/clearance.cpp
  src/core/pure_pursuit.cpp
  src/core/speed_profile.cpp
  src/core/output_stage.cpp
  src/core/ultrasonic.cpp
  src/core/lanes.cpp
  src/core/turn_planner.cpp
  src/core/align_planner.cpp
  src/core/state_machine.cpp
  src/core/vcc_core.cpp
  src/core/path_window.cpp
  src/core/arrival_latch.cpp
)
target_include_directories(vcc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(vcc_core PUBLIC cxx_std_17)
set_target_properties(vcc_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# controller_server 가 pluginlib 로 여는 .so. 이름이 vcc_plugins.xml 의 library path 다.
# 도착 판정 플러그인(LatchedGoalChecker, 2026-09-30)도 같은 .so 에 둔다 — vcc_plugins.xml 한 곳에서 등록.
add_library(vica_vcc_controller SHARED src/vcc_controller.cpp src/latched_goal_checker.cpp)
target_link_libraries(vica_vcc_controller vcc_core)
ament_target_dependencies(vica_vcc_controller
  rclcpp rclcpp_lifecycle pluginlib nav2_core nav2_costmap_2d nav2_util nav_msgs
  geometry_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)
target_compile_definitions(vica_vcc_controller PRIVATE PLUGINLIB__DISABLE_BOOST_FUNCTIONS)
pluginlib_export_plugin_description_file(nav2_core vcc_plugins.xml)

install(TARGETS vcc_core vica_vcc_controller
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES vcc_plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t geometry clearance pure_pursuit speed_profile output_stage ultrasonic
      lanes turn_planner align_planner state_machine vcc_core path_window arrival_latch)
    ament_add_gtest(test_${t} test/test_${t}.cpp)
    target_link_libraries(test_${t} vcc_core)
  endforeach()
  ament_add_gtest(test_vcc_plugin test/test_vcc_plugin.cpp)
  target_link_libraries(test_vcc_plugin vica_vcc_controller)
  ament_target_dependencies(test_vcc_plugin rclcpp rclcpp_lifecycle nav2_costmap_2d nav2_core tf2_ros)
  # 멈추면 빨리 실패로 끝낸다(병렬 부하에서 costmap 소멸 멈춤을 겪었다).
  set_tests_properties(test_vcc_plugin PROPERTIES TIMEOUT 120)

  add_executable(bench_vcc_core test/bench_vcc_core.cpp)
  target_link_libraries(bench_vcc_core vcc_core)
  install(TARGETS bench_vcc_core RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(vica_vcc_controller)
ament_export_dependencies(rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util pluginlib)
ament_package()
