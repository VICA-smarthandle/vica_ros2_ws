# VICA Nav2 Bringup Context Summary

이 문서는 VICA ROS2 워크스페이스에서 Nav2 패키지 구성 직전부터 Nav2 주행 테스트 및 장애물 인식 문제까지의 진행 흐름을 다른 GPT/에이전트에게 전달하기 위한 요약본이다.

## 1. Nav2 진행 직전 상태

### 로봇 기본 센서/TF 상태

- Host ROS2에서 YDLIDAR, encoder_feedback, tf_vica, MDROBOT CAN motor node를 실행하는 구조로 진행했다.
- 기준 TF 정책은 다음과 같다.

```text
map -> odom -> base_link -> laser_frame
                         -> camera_link
```

- `base_link -> laser_frame`, `base_link -> camera_link`는 `tf_vica`에서 static TF로 발행하도록 정리했다.
- `odom -> base_link`는 당시 encoder_feedback이 발행했다.
- Docker/Isaac ROS 쪽 RealSense/VSLAM은 `map -> odom`, `odom -> base_link`, `base_link -> camera_link`, `base_link -> laser_frame`을 발행하지 않도록 하는 방향을 유지했다.

### Cartographer 지도 작성

- Cartographer 2D를 사용해 지도를 작성했다.
- `vica_cartographer/config/vica_2d.lua`에서 `use_odometry = true`로 설정 후 지도 작성이 잘 되는 것을 확인했다.
- 지도 작성 단계에서는 SLAM과 Nav2를 동시에 장기 운용하기보다, 지도 작성 후 저장된 map을 `map_server + AMCL + Nav2`로 사용하는 방향이 맞다고 판단했다.

### encoder odom 상태

- encoder 기반 `/odom`은 Nav2 전에 물리 이동거리와 비교하며 튜닝했다.
- 직진 거리 환산은 `ticks_per_rev`를 조정해 실제 이동거리와 `/odom` 이동거리를 맞췄다.
- 이후 최신 흐름에서는 `ticks_per_rev = 61.2`로 안정화했다.
- `/odom` covariance도 EKF를 위해 추가했다.

## 2. Nav2 방향 결정

대화 중 사용자는 다음 방향에 동의했다.

```text
Cartographer로 지도 작성
-> map 저장
-> 이후 자율주행은 SLAM이 아니라 map_server + AMCL + Nav2 구성으로 진행
```

따라서 Nav2 테스트 목표는 다음과 같이 잡았다.

- 저장된 Cartographer map yaml을 Nav2 launch에 인자로 전달한다.
- `map_server`가 `/map`을 발행한다.
- `amcl`이 `/scan`, `/odom`, TF를 기반으로 `map -> odom`을 발행한다.
- Nav2 controller/planner/BT navigator가 goal을 받아 `/cmd_vel`을 생성한다.
- `/cmd_vel`은 당시 MDROBOT motor node가 구독하는 구조였다.

주의:

- AGENTS.md의 장기 목표 command flow는 다음과 같다.

```text
/cmd_vel_requested
-> safety_supervisor_node
-> /cmd_vel_safe
-> velocity_smoother_node
-> mdrobot_can_motor_node
```

- 하지만 당시 테스트 단계에서는 safety node 구성 전이었고, 기존 motor node가 `/cmd_vel`을 직접 구독했다.

## 3. Nav2 설치 상태 확인

처음에는 레포에 Nav2 패키지가 없다고 판단했지만, 시스템에는 Nav2 binary 패키지가 이미 설치되어 있었다.

사용자가 확인한 설치 패키지 예:

```text
nav2_amcl
nav2_behavior_tree
nav2_behaviors
nav2_bringup
nav2_bt_navigator
nav2_controller
nav2_costmap_2d
nav2_dwb_controller
nav2_lifecycle_manager
nav2_map_server
nav2_msgs
nav2_navfn_planner
nav2_planner
nav2_velocity_smoother
...
```

당시 확인된 주요 토픽:

```text
/cmd_vel
/odom
/scan
/tf
/tf_static
```

## 4. vica_nav2 패키지 생성

VICA 전용 Nav2 bringup/config를 관리하기 위해 `src/vica_nav2` 패키지를 생성했다.

현재 관련 파일:

```text
src/vica_nav2/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/vica_nav2
├── vica_nav2/__init__.py
├── launch/nav2_map_test.launch.py
└── config/
    ├── nav2_params.yaml
    └── nav2_params_backup.yaml
```

### package.xml 의존성

`src/vica_nav2/package.xml`에는 다음 runtime dependency가 있다.

```xml
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
<exec_depend>nav2_bringup</exec_depend>
<exec_depend>nav2_common</exec_depend>
```

### setup.py 구성

`setup.py`에서 launch/config yaml을 install share 경로로 포함하도록 구성했다.

```python
(os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
(os.path.join("share", package_name, "config"), glob("config/*.yaml")),
```

## 5. nav2_map_test.launch.py 구성

`src/vica_nav2/launch/nav2_map_test.launch.py`는 `nav2_bringup`의 `bringup_launch.py`를 include한다.

핵심 launch argument:

```python
DeclareLaunchArgument("map")
DeclareLaunchArgument("params_file", default_value=default_params)
DeclareLaunchArgument("use_sim_time", default_value="false")
DeclareLaunchArgument("autostart", default_value="true")
DeclareLaunchArgument("use_composition", default_value="False")
```

Nav2 bringup include 시 설정:

```python
launch_arguments={
    "slam": "False",
    "map": map_yaml,
    "params_file": params_file,
    "use_sim_time": use_sim_time,
    "autostart": autostart,
    "use_composition": use_composition,
    "use_respawn": "False",
}.items()
```

즉 이 launch는 SLAM 모드가 아니라 저장된 map yaml을 입력으로 받는 localization/navigation 모드이다.

실행 예:

```bash
ros2 launch vica_nav2 nav2_map_test.launch.py map:=/home/ji_w/ros2_ws/maps/vica_cartographer_map_20260602_160101.yaml
```

## 6. Nav2 초기 실행 문제와 해결

### map frame 없음 / map->odom 없음

초기 실행 시 다음 오류가 나타났다.

```text
Timed out waiting for transform from base_link to map to become available
tf error: Invalid frame ID "map" passed to canTransform argument target_frame - frame does not exist
```

확인 명령:

```bash
ros2 node list | grep -E 'map_server|amcl'
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map odom
```

이후 확인 결과:

```text
/map_server active
/amcl active
/map 발행 정상
```

하지만 `map -> odom`은 RViz2에서 `2D Pose Estimate`를 찍기 전에는 나오지 않았다.

해석:

- `map_server`는 `/map`을 발행한다.
- `amcl`은 초기 pose를 받기 전에는 안정적인 `map -> odom`을 발행하지 못한다.
- RViz2에서 `2D Pose Estimate`로 현재 로봇 위치를 지정하면 `map -> odom`이 생성된다.

### RViz2 Map 표시 문제

`/map`은 발행되고 있었으나 RViz2 Map display가 `No map received` 상태가 되었다.

확인:

```bash
ros2 topic info /map -v
```

확인된 상태:

- `map_server` publisher QoS:

```text
Reliability: RELIABLE
Durability: TRANSIENT_LOCAL
```

- RViz subscriber가 처음에는:

```text
Reliability: BEST_EFFORT
Durability: VOLATILE
```

해결:

- RViz2 Map display의 QoS를 `Reliable`, `Transient Local`로 변경했다.
- 이후 map이 RViz2에 정상 표시되었고, `2D Pose Estimate`도 정상 반영되었다.

## 7. Nav2 최초 주행 테스트 결과

RViz2에서 `2D Goal Pose`를 찍어 실제 주행을 시도했다.

확인된 긍정 결과:

- 로봇이 목표 위치까지 이동하는 것 자체는 가능했다.
- global path가 표시되었다.
- AMCL localization과 Nav2 기본 goal flow가 동작했다.

확인된 문제:

```text
1. 장애물이 없는데도 직진하지 않고 좌우로 비틀거리며 주행
2. 기본 속도가 느림
3. 지도상 장애물에 막혔을 때 적절히 회피하지 못함
4. 장애물에 닿거나 가까워져도 멈추지 않고 모터가 계속 돌 수 있음
5. RViz/콘솔에서 Message Filter dropping message: frame 'laser_frame' queue full 경고 발생
```

## 8. 속도/주행 안정성 튜닝

Nav2 속도와 motor node 가변저항 값을 맞추는 논의가 있었다.

당시 motor node는 `/cmd_vel`을 구독하고 있었고, Nav2의 `/cmd_vel` 출력이 motor node로 들어가는 구조였다.

확인한 내용:

```bash
ros2 topic info /cmd_vel -v
```

예시:

```text
Publisher count: 0
Subscription count: 1
Node name: mdrobot_can_keyboard_knob_node
```

이후 Nav2 주행 안정화를 위해 `nav2_params.yaml`의 `controller_server`, `velocity_smoother` 값을 조정했다.

현재 `nav2_params.yaml`에서 DWB 주요 값 예:

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.26
      max_speed_xy: 0.26
      max_vel_theta: 1.0
      acc_lim_x: 2.5
      decel_lim_x: -2.5
      sim_time: 1.7
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
```

튜닝 후:

```text
비틀거림은 거의 줄고 주행은 안정적으로 개선됨.
```

하지만 장애물에 대한 감속/정지는 여전히 해결되지 않았다.

## 9. local_costmap / obstacle 인식 문제

사용자는 RViz2에서 LaserScan 점들은 보이는데 local costmap에는 장애물 cell이 제대로 생기지 않는다고 판단했다.

관찰:

- RViz2에서 LaserScan 초록 점은 보임.
- local costmap 영역은 보이지만 장애물이 cost로 표시되지 않거나 회피에 반영되지 않음.
- `/local_costmap/costmap` 또는 `/local_costmap/costmap_raw` 확인을 진행했다.

관련 확인 명령:

```bash
ros2 param get /local_costmap/local_costmap transform_tolerance
ros2 param list /local_costmap/local_costmap | grep -E 'transform|observation|inf|scan'
ros2 topic echo /local_costmap/costmap_raw --once
ros2 topic echo /local_costmap/costmap_raw --once | grep -v '^- 0$' | head -50
```

당시 local costmap 관련 수정/검토 후보:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      transform_tolerance: 0.5
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 4
      height: 4
      resolution: 0.05
      footprint: "[[0.15, 0.1875], [0.15, -0.1875], [-0.60, -0.1875], [-0.60, 0.1875]]"
      footprint_padding: 0.03
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        tf_filter_tolerance: 0.3
        scan:
          topic: /scan
          data_type: "LaserScan"
          marking: true
          clearing: true
          obstacle_min_range: 0.05
          obstacle_max_range: 3.0
          raytrace_min_range: 0.05
          raytrace_max_range: 3.5
          observation_persistence: 0.2
          expected_update_rate: 0.0
          inf_is_valid: true
```

나중에 확인한 결과:

```text
scan timestamp 기준 TF를 못 찾아 obstacle marking을 못 하는 문제라고 단정하긴 어려움.
/local_costmap/costmap_raw 자체는 발행되고 있었음.
하지만 장애물 cell이 기대한 방식으로 costmap에 생성/반영되지 않았다.
```

## 10. 현재 nav2_params.yaml의 costmap 구조

현재 `src/vica_nav2/config/nav2_params.yaml` 기준 local costmap은 `VoxelLayer`를 사용한다.

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      footprint: "[[0.15, 0.1875], [0.15, -0.1875], [-0.60, -0.1875], [-0.60, 0.1875]]"
      footprint_padding: 0.03
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        observation_sources: scan
        scan:
          topic: /scan
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 9.0
          obstacle_max_range: 8.0
```

global costmap은 `StaticLayer + ObstacleLayer + InflationLayer` 구성이다.

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_link
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

## 11. 장애물 회피 문제의 당시 결론

당시 결론:

```text
1. Nav2 기본 goal 주행은 가능함.
2. 주행 비틀거림은 controller/velocity 설정 조정으로 상당히 개선됨.
3. 장애물 회피/정지는 아직 신뢰할 수 없음.
4. LaserScan은 RViz에서 보이지만 local_costmap obstacle marking이 정상인지 확실하지 않음.
5. safety_node 또는 collision layer 없이 motor node가 /cmd_vel을 직접 받는 구조라 실제 장애물 충돌 시 정지 보장이 약함.
```

사용자는 이후 장애물 인식은 safety_node 구성 시 다시 설정하겠다고 했고, 먼저 주행 안정성과 VSLAM/EKF 쪽을 진행했다.

## 12. 다음에 이어서 볼 우선순위

Nav2 쪽으로 다시 돌아올 때 추천 우선순위:

```text
1. encoder/EKF 기반 odom->base_link TF owner 정책 확정
2. Nav2 실행 전 TF tree 확인
   - map -> odom
   - odom -> base_link
   - base_link -> laser_frame
3. /scan 자체 품질 확인
   - frame_id = laser_frame
   - range_min/range_max
   - 0.0 range 처리
   - hz 안정성
4. local_costmap observation source 확인
   - /local_costmap/costmap_raw
   - /local_costmap/voxel_grid 또는 voxel 관련 debug
   - marking/clearing 반영 여부
5. RViz에서 LaserScan, Local Costmap, Global Costmap을 동시에 확인
6. 장애물 cell이 local_costmap에 생기는지 확인
7. cell은 생기는데 회피하지 않으면 DWB critic / inflation / footprint 튜닝
8. cell 자체가 안 생기면 costmap layer / scan / TF / range 설정 수정
9. safety_supervisor 또는 collision monitor 도입 전까지 고속 주행 금지
```

## 13. GPT에게 전달할 핵심 요약

```text
VICA는 Cartographer로 만든 저장 map을 map_server + AMCL + Nav2로 사용하는 방향을 선택했다.
이를 위해 vica_nav2 패키지를 만들고 nav2_bringup의 bringup_launch.py를 include하는 nav2_map_test.launch.py를 구성했다.
초기 문제는 map이 RViz에 안 뜨는 것과 map->odom TF가 없는 것이었고, Map QoS를 Reliable/Transient Local로 맞추고 2D Pose Estimate를 찍어서 해결했다.
Nav2 goal 주행 자체는 성공했지만, 장애물 회피/정지가 신뢰되지 않았다.
주행 비틀거림은 controller_server / velocity_smoother 계열 튜닝으로 개선되었다.
그러나 LaserScan은 보이는데 local_costmap obstacle marking 또는 회피 반영이 불안정했다.
현재 Nav2 쪽 미해결 핵심은 local_costmap obstacle layer/voxel layer가 /scan을 cost로 제대로 만들고 있는지, 그리고 DWB가 그 cost를 회피 행동으로 반영하는지 확인하는 것이다.
```
