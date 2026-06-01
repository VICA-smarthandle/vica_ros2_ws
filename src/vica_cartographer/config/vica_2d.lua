-- ============================================================
-- VICA Cartographer 2D SLAM 설정 파일
-- 목적:
--   YDLIDAR G2 2D LiDAR + 외부 바퀴 오도메트리(/odom)를 이용해
--   Cartographer 2D 지도를 생성한다.
--
-- 현재 설정 방향:
--   - 외부 오도메트리 사용: use_odometry = true
--   - 외부 odom -> base_link TF 사용: provide_odom_frame = false
--   - Cartographer는 map -> odom 보정 TF를 발행
--
-- 반드시 전제되어야 하는 TF 구조:
--   map -> odom -> base_link -> laser_frame
--
-- 즉, 모터/오도메트리 노드가 odom -> base_link를 발행하고,
-- Cartographer는 map -> odom만 발행하는 구조여야 한다.
--
-- 주의:
--   odom -> base_link를 Cartographer와 다른 노드가 동시에 발행하면
--   TF 충돌이 발생하여 맵이 번지거나 흔들릴 수 있다.
-- ============================================================

include "map_builder.lua"
include "trajectory_builder.lua"


options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- ==========================================================
  -- [1] 프레임 설정
  -- ==========================================================

  -- 전체 지도의 기준 프레임.
  -- RViz에서 최종 2D Map은 보통 map 프레임 기준으로 표시된다.
  map_frame = "map",

  -- Cartographer가 로봇의 위치를 추적할 기준 프레임.
  -- IMU를 쓰지 않는 2D LiDAR 기반 로봇에서는 보통 base_link를 사용한다.
  --
  -- IMU를 사용한다면 tracking_frame을 imu_link로 두는 경우도 있지만,
  -- 현재 VICA 설정은 IMU 미사용이므로 base_link가 적절하다.
  tracking_frame = "base_link",

  -- Cartographer가 pose를 발행할 child frame.
  --
  -- 외부 odom을 사용하는 경우:
  --   published_frame = "odom"
  --
  -- 의미:
  --   외부 오도메트리 노드가 odom -> base_link를 담당하고,
  --   Cartographer는 map -> odom 보정만 수행한다.
  --
  -- 기존에 published_frame = "base_link"로 두면
  -- 외부 odom 구조와 섞이면서 TF 해석이 애매해질 수 있다.
  published_frame = "odom",

  -- 외부 바퀴 오도메트리의 기준 프레임.
  -- 일반적인 차동구동 로봇에서는 odom 프레임을 사용한다.
  odom_frame = "odom",


  -- ==========================================================
  -- [2] odom 프레임 제공 방식
  -- ==========================================================

  -- false:
  --   Cartographer가 odom 프레임을 새로 만들지 않는다.
  --   대신 외부 모터/오도메트리 노드가 odom -> base_link를 제공해야 한다.
  --
  -- true:
  --   Cartographer가 자체적으로 odom 프레임을 제공한다.
  --   이 경우 보통 use_odometry = false와 함께 LiDAR-only 테스트할 때 사용한다.
  --
  -- 현재는 외부 바퀴 오도메트리를 사용하므로 false가 맞다.
  provide_odom_frame = false,

  -- 2D SLAM에서 roll/pitch 성분을 제거하고 평면으로 투영해 발행한다.
  -- 바닥이 평평한 실내 주행 로봇에서는 true 권장.
  publish_frame_projected_to_2d = true,


  -- ==========================================================
  -- [3] 입력 센서 사용 여부
  -- ==========================================================

  -- true:
  --   /odom 토픽을 Cartographer 입력으로 사용한다.
  --
  -- 장점:
  --   긴 복도, 특징점이 적은 공간, 순간적인 LiDAR 매칭 실패 상황에서
  --   바퀴 오도메트리가 보조 역할을 한다.
  --
  -- 위험:
  --   wheel_radius, wheel_base, 좌우 모터 방향, encoder 환산값이 틀리면
  --   잘못된 odom을 Cartographer가 믿어서 맵이 더 번질 수 있다.
  --
  -- 맵이 계속 번지면 가장 먼저 use_odometry = false로 바꿔서
  --   LiDAR-only 기준 테스트를 해봐야 한다.
  use_odometry = false,

  -- 실내 로봇이므로 GPS는 사용하지 않는다.
  use_nav_sat = false,

  -- AprilTag, QR, 별도 landmark를 쓰지 않으므로 false.
  use_landmarks = false,


  -- ==========================================================
  -- [4] LiDAR 입력 설정
  -- ==========================================================

  -- 2D 단선 LiDAR 1개 사용.
  -- YDLIDAR G2가 여기에 해당한다.
  num_laser_scans = 1,

  -- Multi-echo LiDAR가 아니므로 0.
  num_multi_echo_laser_scans = 0,

  -- 하나의 LaserScan 메시지를 몇 조각으로 쪼개 처리할지 결정.
  -- 일반 2D LiDAR는 1부터 시작하는 것이 안정적이다.
  num_subdivisions_per_laser_scan = 1,

  -- 3D LiDAR PointCloud를 쓰지 않으므로 0.
  num_point_clouds = 0,


  -- ==========================================================
  -- [5] TF 대기 시간 및 발행 주기
  -- ==========================================================

  -- Cartographer가 필요한 TF를 기다리는 최대 시간.
  --
  -- 0.2초:
  --   실시간성은 좋지만, TF 발행이 늦으면 경고가 날 수 있다.
  --
  -- 1.0초:
  --   TF 지연에는 관대하지만, 문제가 늦게 드러나고 반응성이 떨어질 수 있다.
  --
  -- 현재는 실시간 맵핑 기준으로 0.2초를 사용한다.
  lookup_transform_timeout_sec = 0.2,

  -- Submap을 RViz 등에 발행하는 주기.
  -- 너무 빠르면 부하가 늘고, 너무 느리면 RViz에서 갱신이 답답하다.
  -- 0.3초는 Cartographer 예제에서 자주 쓰이는 안정적인 값이다.
  submap_publish_period_sec = 0.3,

  -- Pose 발행 주기.
  --
  -- 10e-3 = 0.01초 = 100Hz
  --
  -- 이전 5e-3은 200Hz로 매우 촘촘하지만,
  -- 맵 품질 자체보다는 TF/시각화 부하에 더 영향을 줄 수 있다.
  --
  -- Jetson Orin NX에서는 100Hz 정도부터 시작하는 것이 안전하다.
  pose_publish_period_sec = 10e-3,

  -- Trajectory 발행 주기.
  -- 30e-3 = 0.03초 = 약 33Hz
  trajectory_publish_period_sec = 30e-3,


  -- ==========================================================
  -- [6] 센서 샘플링 비율
  -- ==========================================================

  -- 1.0은 데이터를 100% 사용한다는 뜻.
  -- 성능이 부족할 때만 0.5 등으로 낮춘다.
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,

  -- 현재 사용하지 않는 입력들이지만 기본값 1.0 유지.
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}


-- ============================================================
-- [7] 2D Trajectory Builder 활성화
-- ============================================================

-- 2D SLAM을 사용한다.
-- 3D LiDAR나 3D SLAM이 아니므로 true.
MAP_BUILDER.use_trajectory_builder_2d = true


-- ============================================================
-- [8] 로컬 SLAM: IMU 사용 여부
-- ============================================================

-- 현재 IMU를 사용하지 않는다.
-- LiDAR + 외부 바퀴 오도메트리 조합으로 SLAM을 수행한다.
--
-- IMU를 나중에 추가할 경우:
--   1. tracking_frame을 imu_link로 바꿀지 검토
--   2. base_link -> imu_link TF 정확히 설정
--   3. IMU orientation, gravity alignment 확인
TRAJECTORY_BUILDER_2D.use_imu_data = false


-- ============================================================
-- [9] 로컬 SLAM: LiDAR 유효 거리
-- ============================================================

-- 최소 반영 거리.
--
-- 0.15m 이내의 점은 지도 생성에서 제외한다.
-- 이유:
--   로봇 프레임, 케이블, 바퀴, 본체 일부가 LiDAR에 찍히는 것을 방지.
--
-- 만약 로봇 본체가 계속 지도에 찍히면 0.20~0.25로 올려본다.
-- 단, 너무 올리면 가까운 장애물 인식이 약해질 수 있다.
TRAJECTORY_BUILDER_2D.min_range = 0.15

-- 최대 반영 거리.
--
-- YDLIDAR G2의 스펙상 최대 거리보다,
-- 실내에서 반복 측정이 안정적인 거리를 우선 사용한다.
--
-- 8.0m:
--   맵 번짐 원인 분리용으로 안전한 시작값.
--
-- 10.0m:
--   넓은 복도나 강당에서 더 멀리 보고 싶을 때 2차 테스트값.
--
-- 현재는 안정성을 우선해 8.0m로 둔다.
TRAJECTORY_BUILDER_2D.max_range = 8.0

-- LiDAR가 최대거리까지 아무것도 못 봤을 때,
-- 빈 공간으로 처리할 ray 길이.
--
-- 일반적으로 max_range보다 약간 크게 둔다.
--
-- max_range = 8.0이면 missing_data_ray_length = 8.5
-- max_range = 10.0이면 missing_data_ray_length = 10.5
--
-- 이 값이 너무 짧으면 예전 장애물 잔상이 잘 안 지워질 수 있고,
-- 너무 길면 빈 공간이 과하게 뚫릴 수 있다.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.5


-- ============================================================
-- [10] 로컬 SLAM: Real-time Correlative Scan Matching
-- ============================================================

-- 실시간 상관관계 스캔 매칭 활성화.
--
-- true:
--   외부 odom이 조금 틀려도 LiDAR 스캔 모양을 기준으로 보정하려고 한다.
--
-- 장점:
--   코너링, 미끄러짐, odom 오차에 대한 방어력이 생긴다.
--
-- 단점:
--   CPU 사용량이 증가한다.
--   너무 넓은 탐색창을 주면 잘못된 위치에 맞을 가능성도 있다.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- 스캔 매칭 때 선형 방향으로 탐색할 범위.
--
-- 0.1 = ±10cm 범위에서 후보를 찾는다.
--
-- odom이 대체로 맞는다면 0.1이면 충분하다.
-- odom이 많이 흔들리면 0.15~0.2까지 늘릴 수 있지만,
-- 계산량과 오매칭 위험이 증가한다.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1

-- 이동량이 커지는 후보에 부과하는 비용 가중치.
--
-- 값이 크면 odom 예측 위치에서 많이 벗어나는 해를 덜 선호한다.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0

-- 회전량이 커지는 후보에 부과하는 비용 가중치.
--
-- 0.1은 회전 보정에 비교적 관대한 값이다.
--
-- 회전 중 벽이 두꺼워지거나 밀리면 이 값과 ceres rotation_weight를 함께 봐야 한다.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.1


-- ============================================================
-- [11] 로컬 SLAM: Ceres Scan Matcher
-- ============================================================

-- LiDAR 스캔이 점유 격자 지도와 얼마나 잘 맞는지를 보는 가중치.
--
-- 높을수록 실제 LiDAR 벽면 형태에 강하게 맞추려 한다.
-- 너무 높으면 노이즈에도 과민해질 수 있다.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.0

-- 이전 pose 예측값에서 선형 이동이 크게 벗어나는 것을 얼마나 억제할지 결정.
--
-- 10.0:
--   보수적인 시작값.
--
-- 1.0:
--   odom을 덜 믿고 scan에 더 맡기는 공격형 값.
--   하지만 복도처럼 특징이 부족한 곳에서는 흔들릴 수 있다.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0

-- 이전 pose 예측값에서 회전이 크게 벗어나는 것을 얼마나 억제할지 결정.
--
-- 20.0:
--   회전 추정을 너무 자유롭게 풀어주지 않는 안정형 값.
--
-- 맵이 안정된 뒤 loop closure 반응이 느리면 20으로 줄여 테스트한다.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20.0


-- ============================================================
-- [12] 로컬 SLAM: Motion Filter
-- ============================================================

-- Motion filter는 모든 scan을 무조건 submap에 넣지 않고,
-- 일정 시간/거리/각도 변화가 있을 때만 노드로 추가한다.
--
-- 너무 촘촘하면:
--   계산량 증가, 중복 scan 증가
--
-- 너무 듬성듬성하면:
--   코너링에서 맵 갱신이 늦고 벽이 번질 수 있다.

-- 로봇이 거의 움직이지 않아도 0.2초마다 scan을 검토한다.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.2

-- 5cm 이동하면 새 노드를 추가한다.
--
-- 맵이 거칠면 0.03으로 줄일 수 있다.
-- CPU 부하가 크면 0.07~0.10으로 늘릴 수 있다.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05

-- 0.5도 회전하면 새 노드를 추가한다.
--
-- 기존 2.0도는 코너링에서 반응이 늦을 수 있다.
-- 0.5도는 실내 저속 맵핑에서 비교적 정교한 값이다.
--
-- 회전 중 맵이 여전히 번지면 0.3도까지 낮춰볼 수 있다.
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)


-- LiDAR scan을 몇 개 모아서 하나의 range data로 만들지 결정.
--
-- 1:
--   들어오는 scan을 바로 사용한다.
--   2D LiDAR에서는 보통 1이 안정적이다.
--
-- 2 이상:
--   scan을 누적하므로 노이즈 평균 효과가 있을 수 있지만,
--   로봇이 움직이는 동안 누적하면 오히려 벽이 번질 수 있다.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1


-- ============================================================
-- [13] 글로벌 SLAM: Loop Closure / Constraint Builder
-- ============================================================

-- 로컬 submap과 과거 위치가 같은 장소인지 판단하는 최소 점수.
--
-- 높을수록:
--   잘못된 loop closure가 줄어든다.
--   하지만 loop closure를 덜 잡을 수 있다.
--
-- 낮을수록:
--   loop closure를 더 잘 잡는다.
--   하지만 비슷하게 생긴 복도/반복 구조의
--   장소를 같은 곳으로 착각할 수 있다.
--
-- 0.62:
--   반복 구조가 많은 실내 환경에서 보수적인 시작값.
--
-- 루프 클로저가 너무 안 잡히면 0.60으로 낮춰본다.
-- 지도가 접히거나 갑자기 휘면 0.65로 올린다.
POSE_GRAPH.constraint_builder.min_score = 0.62

-- 전역 재위치 추정의 최소 점수.
--
-- min_score보다 조금 높게 잡아 잘못된 global localization을 줄인다.
--
-- 반복 구조가 많은 실내에서는 너무 낮게 두지 않는 것이 안전하다.
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

-- 큰 오차값(outlier)의 영향을 완화하는 Huber loss scale.
-- 일반적으로 예제에서 많이 쓰는 1e2를 유지한다.
POSE_GRAPH.optimization_problem.huber_scale = 1e2


-- ============================================================
-- [14] 글로벌 SLAM: Pose Graph 최적화 주기
-- ============================================================

-- 노드가 몇 개 쌓일 때마다 전체 pose graph를 최적화할지 결정.
--
-- 35:
--   보수적이고 안정적인 기본 시작값.
--
-- 20:
--   더 자주 최적화해서 누적 오차를 빨리 펴줄 수 있지만,
--   CPU 부하가 늘고 맵이 자주 흔들리는 것처럼 보인다.
--
-- 맵이 안정된 뒤 loop closure 반응이 느리면 20으로 줄여 테스트한다.
POSE_GRAPH.optimize_every_n_nodes = 35


-- ============================================================
-- [15] 글로벌 SLAM: Pose Graph 최적화 가중치
-- ============================================================

-- local SLAM pose의 선형 이동 결과를 얼마나 신뢰할지 결정.
-- 값이 높을수록 로컬 SLAM 결과를 강하게 유지한다.
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5

-- local SLAM pose의 회전 결과를 얼마나 신뢰할지 결정.
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5

-- odometry의 선형 이동 정보를 얼마나 신뢰할지 결정.
--
-- 1e4:
--   odom을 참고하되 local SLAM보다 낮은 가중치로 둔다.
--
-- 바퀴 반지름이나 encoder 환산이 정확하다면 이 값은 도움이 된다.
-- odom 직진 거리가 틀리면 맵이 늘어나거나 줄어들 수 있다.
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4

-- odometry의 회전 정보를 얼마나 신뢰할지 결정.
--
-- 1e3:
--   odom 회전을 완전히 버리지는 않되,
--   local SLAM 회전 결과보다 낮게 둔다.
--
-- 1e2:
--   바퀴 미끄러짐이 심하거나 제자리 회전 odom이 많이 틀릴 때 테스트.
--
-- 1e4 이상:
--   odom 회전이 매우 정확할 때만 고려.
--
-- 회전할 때 맵이 번지면 이 값만 바로 낮추기 전에
--   wheel_base_m, 좌우 모터 방향, base_link -> laser_frame TF를 먼저 확인한다.
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e3


-- ============================================================
-- [16] 튜닝 순서 요약
-- ============================================================
--
-- 1. 이 설정으로 저속 맵핑 테스트
--    직진 0.10~0.20 m/s
--    회전 0.20~0.40 rad/s
--
-- 2. 맵이 직진 중에도 두꺼워지면
--    - max_range 8.0 유지
--    - min_range 0.15 -> 0.20 테스트
--    - laser_frame TF 확인
--
-- 3. 회전할 때만 벽이 번지면
--    - wheel_base_m 확인
--    - 좌우 모터 방향 확인
--    - odom angular.z 확인
--    - ceres rotation_weight 20 -> 10 테스트
--    - odometry_rotation_weight 1e3 -> 1e2 테스트
--
-- 4. 루프를 돌고 나서 지도가 접히면
--    - min_score 0.62 -> 0.65
--    - global_localization_min_score 0.66 -> 0.70
--
-- 5. 루프 클로저가 너무 안 잡히면
--    - min_score 0.62 -> 0.60
--    - optimize_every_n_nodes 35 -> 20
--
-- 6. odom을 켰을 때만 맵이 나빠지면
--    - use_odometry = false로 LiDAR-only 테스트
--    - odom 계산 노드부터 수정
--
-- ============================================================

return options
