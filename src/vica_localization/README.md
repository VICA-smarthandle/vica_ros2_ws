# VICA Localization

이 패키지는 MDROBOT 휠 오도메트리와 IMU 각속도를 `robot_localization` EKF로 융합하여 VICA의 표준 `/odom`과 `odom -> base_footprint` TF를 제공한다.

## 오도메트리 계약

```text
MDROBOT encoder
  -> encoder_feedback
  -> /wheel/odom
  -> robot_localization/ekf_node
  -> /odom
  -> odom -> base_footprint TF

D455 IMU
  -> IMU frame adapter
  -> /imu/base_link
  -> robot_localization/ekf_node
```

- `/wheel/odom`: EKF 처리 전 원시 휠 오도메트리
- `/imu/base_link`: 로봇 기준 좌표계로 변환된 IMU 데이터
- `/odom`: EKF 융합 결과이며 Cartographer, Nav2와 앱이 사용하는 표준 오도메트리
- `odom -> base_footprint`: EKF가 단독 게시한다.
- `encoder_feedback`는 TF를 게시하지 않는다.

## EKF 설정 유지 원칙

기준 설정은 `config/ekf.yaml`이다. 기존 `ekf.yaml`의 활성 설정 형식과 주석 처리된 VSLAM 대안 설정을 유지했으며, 원시 휠 입력 토픽만 `/odom`에서 `/wheel/odom`으로 변경했다.

현재 활성 계약은 다음과 같다.

```yaml
odom0: /wheel/odom
imu0: /imu/base_link
base_link_frame: base_footprint
world_frame: odom
publish_tf: true
```

EKF 출력 `odometry/filtered`는 `launch/wheel_ekf.launch.py`에서 `/odom`으로 remap한다. 설정을 변경할 때 주석 처리된 대안 블록을 임의로 삭제하거나 YAML 구조를 재작성하지 않는다.

## 설치 의존성

새 환경에서는 다음 런타임 패키지가 필요하다.

```bash
sudo apt install ros-humble-robot-localization python3-can
```

- `ros-humble-robot-localization`: EKF 실행 파일 제공
- `python3-can`: `encoder_feedback`의 SocketCAN 통신에 필요

저장소 빌드만 성공하더라도 시스템에 `robot_localization`이 설치되지 않았다면 실제 launch는 실행할 수 없다.

## 빌드 및 실행

워크스페이스 루트 `vica_ros2_ws/`에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to vica_localization
source install/setup.bash
ros2 launch vica_localization wheel_ekf.launch.py
```

CAN 장치와 모터를 사용하지 않고 EKF 설정만 확인할 때는 encoder 노드를 비활성화한다.

```bash
ros2 launch vica_localization wheel_ekf.launch.py start_encoder:=false
```

## 2026-07-22 로컬 검증 결과

- `encoder_feedback`, `vica_localization`, `vica_cartographer`, `vica_nav2` 빌드 성공
- 소스의 `config/ekf.yaml`과 설치된 `ekf.yaml`이 일치함을 확인
- `vica_localization` EKF 계약 테스트 2건 통과
- `robot_localization/ekf_node` 기동 확인
- 실행 중인 노드에서 다음 파라미터가 실제로 로드됨을 확인
  - `odom0 = /wheel/odom`
  - `imu0 = /imu/base_link`
  - `base_link_frame = base_footprint`
  - `world_frame = odom`
  - `publish_tf = true`
- `/odom`에 `nav_msgs/msg/Odometry` Publisher가 한 개 생성됨을 확인
- 검증 종료 시 EKF 프로세스가 정상 종료됨을 확인

검증 당시 개발 PC의 시스템 ROS 경로에는 `robot_localization`이 설치되어 있지 않아 임시
디렉터리에 추출한 공식 Ubuntu 패키지로 런타임만 확인했다. 팀원 환경에서는 위 설치
의존성을 먼저 설치해야 한다.

## 아직 완료되지 않은 실기 검증

다음 항목은 설정 및 합성 입력 검증과 별도로 실제 로봇에서 확인해야 한다.

- `can1`에서 MDROBOT C5 엔코더 응답이 지속해서 수신되는지 확인
- `/wheel/odom`의 선속도와 각속도 부호 및 스케일 확인
- D455 실행 환경에서 실제 IMU 토픽이 `/imu/base_link`까지 전달되는지 확인
- 정지 상태에서 IMU 각속도 편향과 `/odom` 드리프트 확인
- 직진, 제자리 회전, 사각 경로 주행 후 위치 및 yaw 오차 확인
- `/odom`과 `odom -> base_footprint`의 Publisher가 각각 EKF 하나뿐인지 확인
- 원시 오도메트리 또는 IMU 입력이 끊겼을 때 진단 및 안전 정지 동작 확인

실기 검증 전에는 이 구성을 실제 로봇 검증 완료 상태로 표시하지 않는다. 모터 구동 시험은 E-stop 체인과 안전 담당자의 확인 후 진행한다.

공식 자료는 저장소의 `guideline/official_reference_urls.md`를 우선 참고한다.
