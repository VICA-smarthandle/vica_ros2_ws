-- VICA Cartographer 2D SLAM
--
-- TF ownership:
--   Cartographer: map -> odom
--   EKF:           odom -> base_footprint
--   URDF/RSP:      base_footprint -> base_link -> laser_frame/camera_link
--
-- tf_vica static publishers are not used with this setup.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",

  -- External odom/EKF provides odom -> base_footprint.
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,

  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 10e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- VICA currently maps with 2D LiDAR + wheel odometry, without IMU input.
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- RPLIDAR(/dev/rplidar, scan_mode Express) 실측 0.15 ~ 12.00 m.
-- 2026-08-15 이전 주석은 "YDLIDAR G2" 였는데 장비가 다르다.
--
-- min_range 0.15 -> 0.25 (2026-08-15)
--   0.15 로 두면 차체 프레임이 장애물로 들어온다. /scan 30장 정밀 측정에서
--   후방 좌우 대칭으로 근접 반사가 잡혔다.
--     -147 ~ -138도  0.221~0.239 m  변동 0.023 m  검출률 89%
--     +138 ~ +150도  0.220~0.273 m                검출률 98%
--   손잡이를 분리한 상태와 부착한 상태에서 값이 거의 같아(차이 0~17 mm)
--   차체 고정 구조물로 확정했다. 라이다가 로봇 중심보다 18.5 cm 앞에 있어
--   중앙의 구조물이 뒤쪽으로 보이는 것이다. base_footprint 로 옮기면
--   x +0.003 · y ±0.136 — 로봇 정중앙, 좌우 13~14 cm 다.
--
--   Nav2 에서는 문제가 안 된다. costmap 은 footprint_clearing_enabled 로
--   매 주기 footprint 내부를 FREE_SPACE 로 덮으므로 이 반사가 지워진다
--   (run12 bag 실측: local 509장 global 112장 모두 LETHAL 0건).
--   **Cartographer 에는 그런 기능이 없다.** 걸러지지도 지워지지도 않고
--   스캔당 약 46점(23도 ÷ 0.499도)이 그대로 들어간다.
--
--   0.25 인 이유: 구조물 최대 0.239 m 보다 크고, 로봇 내접반경 0.275 m
--   보다 작다. 잃는 것은 라이다에서 0.15~0.25 m 구간인데 옆·뒤로는 로봇
--   몸이고, 앞으로는 로봇 중심에서 0.335~0.435 m 다. 그 거리의 벽은 이미
--   8 m 밖에서부터 관측된 뒤다.
--
--   근거와 실측: devlog/2026-08-15-복구예산-collision-monitor-자동재시도.md §9
TRAJECTORY_BUILDER_2D.min_range = 0.25

-- [CM-1] max_range 8.0 -> 11.0, missing_data_ray_length 8.5 -> 11.5 (2026-08-21)
--
-- 라이다는 12 m 를 보는데 8 m 에서 자르고 있었다. RPLIDAR 드라이버 신고값이
-- 0.15~12.00 m 이고 실내 실측 최대 반사가 9.04 m 였다.
--
-- 왜 중요한가 — 복도에서 앞뒤 위치를 알려 주는 것은 **멀리 있는 것들**이다.
-- 복도 끝, 문, 교차로. 8 m 에서 자르면 그걸 버리고 양옆 벽만 남는데, 양옆 벽은
-- 앞뒤에 대해 아무 말도 하지 않는다. 2026-08-12 에 44 m 복도에서 최대 7.01 m 가
-- 어긋난 것이 바로 그 앞뒤 미끄러짐이다(회전은 0.031 로 멀쩡했다).
--
-- 12.0 이 아니라 11.0 인 이유: 12.00 은 드라이버가 신고한 **상한**이고 실제로
-- 반사가 돌아오는 최대는 9 m 근처다. 12.0 으로 두면 아무것도 안 맞은 빔이 유효한
-- 것처럼 섞인다.
--
-- 대가: 계산량이 는다. 원거리 점은 각해상도가 성겨(0.499도 -> 11 m 에서 9.6 cm
-- 간격) 잡음이 섞인다.
--
-- 판정: cartographer_node 로그의 `differs by translation` 최대값과 1 m 초과 건수.
-- 합격선은 최대 1 m 미만 · 1 m 초과 0건이다.
-- 되돌릴 때는 두 값을 함께 8.0 / 8.5 로 내린다.
-- 근거: docs/cartographer_corridor_mapping.md CM-1
TRAJECTORY_BUILDER_2D.max_range = 11.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 11.5

-- Keep online scan matching enabled to compensate small wheel odom errors.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.1

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20.0

-- Low-speed indoor mapping update thresholds.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- [CM-7] loop closure 검색창 7.0(기본) -> 3.0 (2026-09-03)
--
-- 9/3 복도 매핑 2회에서 translation 오차 최대가 매번 7 m 근처(7.27 / 7.85)
-- = 이 창의 기본값. 8/12 실패도 7.01 이었다. 복도는 ±7 m 어디나 비슷해서
-- 창 끝까지 미끄러진 가짜 매칭이 62~68 % 점수로 min_score(0.62)를 통과해
-- 지도를 찢었다(1 m 초과 48~59 %). 창을 3 m 로 줄이면 가짜 매칭이 미끄러질
-- 수 있는 최대 거리가 3 m 로 준다.
--
-- 2.0 이 아닌 이유: 진짜 보정(오도메트리 드리프트)이 창보다 크면 loop
-- closure 를 아예 못 찾는다. 2.0 과 min_score 0.70(CM-3)은 이번 회차의
-- bag 을 cartographer_offline_node 로 재생해 재주행 없이 비교한다.
--
-- 판정: docs/cartographer_corridor_mapping.md §6 — 최대 1 m 미만 · 1 m 초과
-- 0건 · 구속조건 수 급감 없음. 되돌릴 때는 이 한 줄을 지운다(기본 7.0).
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.0

-- [CM-3] loop closure 합격 점수 0.62 -> 0.70 (2026-09-03, CM-7 2회차)
--
-- CM-7(창 3 m) 1회차: 1 m 초과 48~59 % -> 12.2 %, 최대 7.85 -> 3.15 m 로 번짐은
-- 크게 줄었지만, 가짜 매칭 532건 중 309건이 창 끝(2~3 m)에 몰렸다. 창은 "얼마나
-- 멀리 틀리나"만 줄이고 틀린 매칭을 받는 버릇은 못 고친다. 그 회차 점수를
-- 진짜(<=1 m, 중앙 71.0)와 가짜(>1 m, 중앙 67.0)로 가르면 0.70 에서
-- 가짜 73 % 탈락 · 진짜 56 %(2168건) 유지다.
--
-- 위험: 정상 회차의 진짜 꿰매기도 같이 잘려 지도가 덜 닫힐 수 있다. 판정은
-- 1 m 초과 건수가 줄면서 **구속조건 총수가 급감하지 않는 것** — 절반 밑으로
-- 떨어지면 0.68 로 물러선다. 이 회차는 bag 필수(값별 offline 비교용).
-- 근거: docs/cartographer_corridor_mapping.md CM-3 · CM-7 1회차 결과.
POSE_GRAPH.constraint_builder.min_score = 0.70
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35

POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e3

-- If maps smear, verify in this order:
--   /scan hz, /odom hz, odom -> base_footprint,
--   base_footprint -> base_link, base_link -> laser_frame,
--   wheel_radius_m, wheel_base_m, motor direction, timestamps.

return options
