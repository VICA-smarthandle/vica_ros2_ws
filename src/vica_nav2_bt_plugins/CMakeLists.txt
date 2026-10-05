cmake_minimum_required(VERSION 3.8)
project(vica_nav2_bt_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)

# 순수 판정은 ROS 에 의존하지 않는다 — 시험이 노드·TF 없이 돈다.
add_library(route_path_check src/route_path_check.cpp src/path_end_align.cpp src/rail_clear_check.cpp)
target_include_directories(route_path_check PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(route_path_check PUBLIC cxx_std_17)
# 공유 플러그인에 정적으로 묶인다. std::regex(rail_clear_check) 처럼 외부 기호를 쓰면 PIC 가 필요하다(aarch64).
set_target_properties(route_path_check PROPERTIES POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(route_path_check nav_msgs geometry_msgs)

# BT 플러그인. 이 타깃 이름이 그대로 nav2_params.yaml 의 plugin_lib_names 항목이다.
# (bt_navigator 는 lib<이름>.so 를 연다. 이름이 어긋나면 configure 에서 멈춘다 —
#  test_bt_plugin_libraries_exist 가 파일 존재를 확인한다.)
add_library(vica_is_route_path_usable_condition_bt_node SHARED
  src/is_route_path_usable_condition.cpp)
target_link_libraries(vica_is_route_path_usable_condition_bt_node route_path_check)
ament_target_dependencies(vica_is_route_path_usable_condition_bt_node
  rclcpp behaviortree_cpp_v3 nav2_behavior_tree nav2_util nav_msgs geometry_msgs tf2_ros)
target_compile_definitions(vica_is_route_path_usable_condition_bt_node PRIVATE BT_PLUGIN_EXPORT)

# 2026-09-30: 목적지 경로 끝점 방향 맞춤(run49 409호 도착 교착). 이름 규칙은 위와 같다.
add_library(vica_align_path_end_to_goal_action_bt_node SHARED
  src/align_path_end_to_goal_action.cpp)
target_link_libraries(vica_align_path_end_to_goal_action_bt_node route_path_check)
ament_target_dependencies(vica_align_path_end_to_goal_action_bt_node
  rclcpp behaviortree_cpp_v3 nav_msgs geometry_msgs)
target_compile_definitions(vica_align_path_end_to_goal_action_bt_node PRIVATE BT_PLUGIN_EXPORT)

# 2026-10-05 해결안 B: 레일 앞 조각 막힘 검사(IsPathValid 대신). 이름 규칙은 위와 같다.
add_library(vica_is_rail_ahead_clear_condition_bt_node SHARED
  src/is_rail_ahead_clear_condition.cpp)
target_link_libraries(vica_is_rail_ahead_clear_condition_bt_node route_path_check)
ament_target_dependencies(vica_is_rail_ahead_clear_condition_bt_node
  rclcpp behaviortree_cpp_v3 nav_msgs nav2_msgs)
target_compile_definitions(vica_is_rail_ahead_clear_condition_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS route_path_check vica_is_route_path_usable_condition_bt_node
  vica_align_path_end_to_goal_action_bt_node vica_is_rail_ahead_clear_condition_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_route_path_check test/test_route_path_check.cpp)
  target_link_libraries(test_route_path_check route_path_check)
  ament_target_dependencies(test_route_path_check nav_msgs)
  ament_add_gtest(test_path_end_align test/test_path_end_align.cpp)
  target_link_libraries(test_path_end_align route_path_check)
  ament_target_dependencies(test_path_end_align nav_msgs geometry_msgs)
  ament_add_gtest(test_rail_clear_check test/test_rail_clear_check.cpp)
  target_link_libraries(test_rail_clear_check route_path_check)
  ament_target_dependencies(test_rail_clear_check nav_msgs geometry_msgs)
endif()

ament_package()
