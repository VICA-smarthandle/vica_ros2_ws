# VICA ROS2 Workspace

VICA는 Jetson Orin NX 16GB 기반 ROS2 Humble 실내 안내 AMR 프로젝트입니다.

이 프로젝트는 시각장애인 또는 고령자의 실내 이동을 보조하기 위해 2D SLAM, Visual SLAM, Nav2, CAN 모터 제어, 음성 인터페이스, LLM을 통합하는 것을 목표로 합니다.

---

## 1. Project Overview

VICA의 주요 기능은 다음과 같습니다.

* YDLIDAR G2 기반 2D LiDAR SLAM
* Cartographer 2D 기반 지도 작성
* Intel RealSense D455 기반 Visual SLAM
* Nav2 기반 목적지 자율주행
* MDROBOT CAN 기반 BLDC 모터 제어
* Smart Handle 기반 사용자 입력 및 안전 제어
* Flutter 관리자 앱을 통한 지도 및 장소 좌표 관리
* Gemma 경량화 모델 기반 자연어 의도 해석 후 목적지 추론

---

## 2. Hardware Environment

| Component     | Specification                         |
| ------------- | ------------------------------------- |
| Edge Computer | NVIDIA Jetson Orin NX 16GB            |
| Carrier Board | Seeed Studio reComputer Robotics J401 |
| OS            | Ubuntu 22.04.x                        |
| JetPack       | R36 / JetPack 6.x                     |
| ROS2          | Humble                                |
| Camera        | Intel RealSense D455                  |
| LiDAR         | YDLIDAR G2                            |
| Motor Driver  | MDROBOT BLDC Driver, CAN              |
| CAN Interface | can1                                  |

---

## 3. Repository Policy

이 레포는 VICA에서 직접 작성하거나 수정한 코드와 설정을 관리합니다.

외부 ROS2 드라이버 패키지는 레포에 직접 포함하지 않고 `vica.repos`로 관리합니다.

외부 의존성:

* `realsense-ros`
* `ydlidar_ros2_driver`
* `robot_localization`
* `python3-can`

외부 드라이버에 임시로 적용했던 로컬 수정사항은 `docs/patches/`에 patch 파일로 보관합니다.

---

## 4. Branch Policy

이 레포는 단순한 2-브랜치 전략을 사용합니다.

| Branch | Role      |
| ------ | --------- |
| main   | 안정/제출 버전  |
| dev    | 개발/테스트 버전 |

운영 원칙:

* 일반 개발 작업은 `dev`에서 진행합니다.
* 실제 로봇에서 검증된 변경사항만 `main`에 병합합니다.
* `main`에서는 직접 수정하지 않습니다.

---

## 5. External Dependencies

외부 ROS2 패키지는 `vica.repos`로 복원합니다.

```bash
sudo apt update
sudo apt install -y python3-vcstool ros-humble-robot-localization python3-can

# vica_ros2_ws/ 루트에서 실행
vcs import < vica.repos
```

필요 시 외부 드라이버 로컬 수정 patch를 적용합니다.

```bash
# vica_ros2_ws/ 루트에서 실행
git -C src/realsense-ros apply ../../docs/patches/realsense_ros_local_changes.patch
git -C src/ydlidar_ros2_driver apply ../../docs/patches/ydlidar_ros2_driver_local_changes.patch
```

장기적으로는 외부 드라이버를 직접 수정하지 않고, VICA 전용 launch/config를 별도 패키지에서 관리하는 것을 목표로 합니다.

---

## 6. Build

```bash
# vica_ros2_ws/ 루트에서 실행
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 7. Basic Check Commands

ROS2 환경 확인:

```bash
echo $ROS_DOMAIN_ID
echo $RMW_IMPLEMENTATION
ros2 topic list
```

LiDAR 확인:

```bash
ros2 topic hz /scan
```

Odometry 확인:

```bash
ros2 topic hz /wheel/odom
ros2 topic hz /imu/base_link
ros2 topic hz /odom
ros2 topic echo /odom --once
```

TF 확인:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_link laser_frame
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_tools view_frames
```

CAN 확인:

```bash
ip -details link show can1
candump can1
```

---

## 8. Current SLAM / Navigation Policy

* 2D SLAM은 Cartographer 2D 중심으로 구성합니다.
* Visual SLAM과 nvblox는 Isaac ROS Docker에서 분리 운영합니다.
* Nav2는 지도 기반 목적지 자율주행에 사용합니다.
* wheel odometry와 IMU의 EKF 설정·bringup 정본은 `src/vica_localization/`입니다.
* 표준 계약은 `/wheel/odom + /imu/base_link → EKF → /odom`입니다.
* D455 launch는 이 저장소가 아닌 별도 Docker/Isaac ROS 환경에서 관리합니다.

---

## 9. Safety Policy

VICA는 실제 바퀴가 움직이는 AMR이므로 안전 구조를 우선합니다.

현재 코드와 목표 안전 연결을 함께 표시한 주행 명령 흐름은 다음과 같습니다.

```text
Nav2 controller /cmd_vel_nav
→ velocity_smoother_node
→ /cmd_vel
→ [GAP: /cmd_vel_req 연결 필요]
→ safety_supervisor_node
→ /cmd_vel_safe
→ mdrobot_can_keyboard_knob_node
→ CAN frame
→ MDROBOT driver
```

안전 원칙:

* `/cmd_vel`을 모터 CAN 명령으로 직접 연결하지 않습니다.
* LLM이 직접 주행 명령을 내리지 않습니다.
* 실행 중인 `can1`을 임의로 down/up 하지 않습니다.
* 비상정지와 명령 timeout은 Safety Supervisor가 정지로 처리합니다.
* 장애물 회피·감속은 Nav2 계층의 책임이며 Safety Supervisor나 물리 E-stop을 대체하지 않습니다.
* 중앙 E-stop 래치와 관리자 앱 단일 reset은 아직 구현·종단 검증이 필요한 `[TARGET]`입니다.
