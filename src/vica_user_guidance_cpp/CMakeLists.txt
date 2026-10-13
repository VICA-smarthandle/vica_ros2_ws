cmake_minimum_required(VERSION 3.8)
project(vica_user_guidance_cpp)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# 파이썬 판과 끝자리까지 같게: 곱·더하기를 한 명령(FMA)으로 합치지 않는다(모터 C++ 판과 같은 이유).
add_compile_options(-ffp-contract=off)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vica_interfaces REQUIRED)

# 판정·순서·시리얼. ROS 에 기대지 않는다 — 시험과 대조 시험이 노드 없이 돈다.
add_library(vica_guidance_core STATIC
  src/pyfmt.cpp
  src/pyjson.cpp
  src/frames.cpp
  src/guidance.cpp
  src/serial_link.cpp
  src/posix_serial.cpp
  src/custom_baud.cpp
  src/driver_core.cpp
  src/rclpy_compat.cpp
  src/pymath.cpp
  src/turn_detector.cpp
  src/rail_turn_forecast.cpp
  src/turn_core.cpp)
target_include_directories(vica_guidance_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(vica_guidance_core PUBLIC cxx_std_17)
# __int128 는 GCC 확장이라 -Wpedantic 이 경고한다. 파이썬 int 의 크기를 담으려고 일부러 쓴다.
target_compile_options(vica_guidance_core PRIVATE -Wno-pedantic)

# 실행 파일 이름은 파이썬 판과 같아야 한다 — 터미네이터 guard(앞 15자 user_guidance_d)가 이 이름을 본다.
add_executable(user_guidance_driver_node src/user_guidance_driver_node.cpp)
target_link_libraries(user_guidance_driver_node vica_guidance_core)
target_compile_options(user_guidance_driver_node PRIVATE -Wno-pedantic)
ament_target_dependencies(user_guidance_driver_node
  rclcpp std_msgs sensor_msgs builtin_interfaces tf2 tf2_ros vica_interfaces)

# 방향지시 노드(2026-10-13). 실행 파일 이름도 파이썬 판과 같다(turn_guide_node).
add_executable(turn_guide_node src/turn_guide_node.cpp)
target_link_libraries(turn_guide_node vica_guidance_core)
target_compile_options(turn_guide_node PRIVATE -Wno-pedantic)
ament_target_dependencies(turn_guide_node rclcpp nav_msgs tf2 tf2_ros vica_interfaces)

install(TARGETS user_guidance_driver_node turn_guide_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_guidance_core test/test_guidance_core.cpp)
  target_link_libraries(test_guidance_core vica_guidance_core util)   # util = openpty
  target_compile_options(test_guidance_core PRIVATE -Wno-pedantic)

  # 파이썬 원본(같은 저장소 vica_user_guidance)을 정답지로 쓰는 대조 시험.
  add_executable(law_cpp test/parity/law_cpp.cpp)
  target_link_libraries(law_cpp vica_guidance_core)
  target_compile_options(law_cpp PRIVATE -Wno-pedantic)
  ament_add_pytest_test(test_parity test/parity/test_parity.py
    ENV
      LAW_CPP=$<TARGET_FILE:law_cpp>
      GUIDANCE_PY_SRC=${CMAKE_CURRENT_SOURCE_DIR}/../vica_user_guidance
    TIMEOUT 1200)
  # 방향지시(turn_guide_node) 대조 — TF 는 진짜 tf2::BufferCore.
  find_package(geometry_msgs REQUIRED)
  add_executable(turn_law_cpp test/parity/turn_law_cpp.cpp)
  target_link_libraries(turn_law_cpp vica_guidance_core)
  target_compile_options(turn_law_cpp PRIVATE -Wno-pedantic)
  ament_target_dependencies(turn_law_cpp tf2 geometry_msgs)
  ament_add_pytest_test(test_turn_parity test/parity/test_turn_parity.py
    ENV
      TURN_LAW_CPP=$<TARGET_FILE:turn_law_cpp>
      GUIDANCE_PY_SRC=${CMAKE_CURRENT_SOURCE_DIR}/../vica_user_guidance
    TIMEOUT 1800)
  ament_add_pytest_test(test_source_contract test/test_source_contract.py TIMEOUT 60)
endif()

ament_package()
