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
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.5

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

-- Conservative loop-closure values for repeated indoor corridors.
POSE_GRAPH.constraint_builder.min_score = 0.62
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
