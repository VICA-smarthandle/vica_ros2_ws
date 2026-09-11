# vica_nvblox_bringup 운영 매뉴얼

RealSense D455 + nvblox(3D 재구성)로 만든 장애물을 Nav2 local costmap의
`nvblox_layer`로 넣기 위한 nvblox 실행 패키지다. 기존의 긴 수동 명령
`run_nvblox.sh`를 대체한다.

---

## 0. 구조 한눈에 (왜 복사가 필요한가)

- **Host(내 방)** 와 **Isaac ROS Docker 컨테이너(옆방)** 는 분리돼 있고, 공용으로
  보이는 건 **`/workspaces/isaac_ros-dev` 폴더(공용 선반)** 하나뿐이다.
- `nvblox`는 **컨테이너에만** 있고, Nav2·EKF·LiDAR는 **Host에만** 있다.
- 이 패키지의 **정본**은 VICA repo(`vica_ros2_ws/src/vica_nvblox_bringup`)에 있지만,
  컨테이너는 VICA repo를 못 본다. 그래서 **컨테이너용 복사본**을
  `~/workspaces/isaac_ros-dev/src/`(= 공용 선반)에 둔다.

| | Host ROS 2 | Isaac ROS Docker |
| --- | --- | --- |
| 담당 | encoder, EKF, TF, LiDAR, **Nav2**, motor/safety | RealSense D455, **nvblox** |
| 이 패키지 위치 | 정본(git) | 실행용 복사본 |

> ⚠️ **정본을 고치면 복사본은 자동으로 안 바뀐다.** 아래 8절 "동기화"를 다시 해야 한다.

TF 정책(반드시 유지):
```
map └ odom └ base_footprint └ base_link ├ laser_frame └ camera_link
```
- Host: EKF가 `odom -> base_footprint`, robot_state_publisher가 그 아래.
- Docker: **camera 내부 frame만** 발행. `odom->base_*`, `base_link->camera_link`는 발행 금지.

---

## 1. 사전 조건

- Host 기본 주행 stack이 정상(단독 LiDAR Nav2 주행 확인됨).
- Host에 nvblox 플러그인 빌드됨(`nvblox_msgs`, `nvblox_nav2`). 소스 symlink가
  `vica_ros2_ws/src/`에 있어야 한다(7절 참고).
- 컨테이너에서 `run_d455.sh`로 D455 depth/color 발행 가능.

---

## 2. 최초 1회: 컨테이너에 패키지 설치

### 2-1. 동기화 (Host에서 — 정본을 공용 선반으로 복사)
Host 터미널:
```bash
rm -rf ~/workspaces/isaac_ros-dev/src/vica_nvblox_bringup
cp -r ~/VICA-smarthandle/vica_ros2_ws/src/vica_nvblox_bringup ~/workspaces/isaac_ros-dev/src/
```

### 2-2. 빌드 (컨테이너 안에서 — 한 줄씩)
```bash
cd /workspaces/isaac_ros-dev
```
```bash
source /opt/ros/humble/setup.bash && source /workspaces/isaac_ros-dev/install/setup.bash
```
```bash
colcon build --symlink-install --packages-select vica_nvblox_bringup
```
```bash
source /workspaces/isaac_ros-dev/install/setup.bash
```
```bash
ros2 pkg prefix vica_nvblox_bringup
```
→ `/workspaces/isaac_ros-dev/install/vica_nvblox_bringup` 가 나오면 설치 성공.

> 💡 여러 줄을 `\`로 이어 붙여 붙여넣으면 터미널에서 깨진다. **한 줄씩** 복사할 것.

---

## 3. 매번 실행 순서

### 3-1. 컨테이너 (순서 중요: 카메라 먼저)
```bash
# 터미널 A — 카메라
./run_d455.sh
```
```bash
# 터미널 B — nvblox (run_nvblox.sh 대신 이것을 쓴다)
source /opt/ros/humble/setup.bash && source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch vica_nvblox_bringup vica_nvblox.launch.py
```

### 3-2. Host
- 기본 stack(encoder/EKF/TF/LiDAR/motor·safety) 기동.
- **Nav2 재시작.** YAML은 실행 중 자동 반영 안 되므로, 돌던 Nav2는 반드시 종료 후 재시작.

---

## 4. 검증 (이 순서로)

```bash
ros2 lifecycle get /local_costmap/local_costmap   # active [3] 기대
ros2 topic hz  /nvblox_node/static_map_slice        # Hz 찍히면 slice 발행 OK
ros2 topic echo --once /nvblox_node/static_map_slice # header.frame_id == odom 확인
ros2 topic echo --once /local_costmap/costmap        # 로봇 주변이 통째로 막혔는지(바닥 블랭킷) 확인
ros2 topic echo /cmd_vel_req                         # 2D Goal 시 발행되면 주행 성공
```

**성공 기준:** `local_costmap` = active, 2D Goal 입력 시 path 생성 + `/cmd_vel_req` 발행.

---

## 5. 튜닝: 바닥이 장애물로 잡힐 때

Goal은 수락되는데 안 움직이고 `/local_costmap/costmap` 전방이 통째로 LETHAL이면,
nvblox가 **바닥을 장애물로** 투영한 것이다. slice 하한 높이를 올려 바닥을 제외한다.

```bash
ros2 launch vica_nvblox_bringup vica_nvblox.launch.py esdf_slice_min_height:=0.08
```
- 기본값 `0.05`. 바닥이 계속 잡히면 `0.02`씩 올린다(대략 0.03~0.15).
- 반대로 낮은 장애물(의자 다리 등)이 안 잡히면 내린다.
- 로봇 주변이 FREE가 되고 실제 장애물만 남을 때까지 반복.

관련 파일: `config/vica_nvblox_overrides.yaml`(max/height 등 고정값).

---

## 6. launch 인자

| 인자 | 기본값 | 의미 |
| --- | --- | --- |
| `global_frame` | `odom` | nvblox 좌표계. Nav2 local costmap `global_frame`·`nav2_costmap_global_frame`과 반드시 일치 |
| `map_clearing_frame_id` | `camera_link` | map clearing/slice bounds 기준 frame |
| `esdf_slice_min_height` | `0.05` | 바닥 제외 높이(1차 튜닝 노브) |

카메라 remap은 launch에 고정: `camera_0/{depth,color}/*` → `/camera/camera/{depth,color}/*`.

---

## 7. 문제 해결

**`package 'vica_nvblox_bringup' not found`**
→ 컨테이너에 아직 빌드·source 안 됨. 2절을 순서대로. 특히 빌드 후
`source .../install/setup.bash`를 **다시** 실행해야 함.

**Nav2가 Goal을 거부 / `local_costmap`이 `unconfigured`**
→ Host의 nvblox 플러그인 리소스 깨짐. `vica_ros2_ws/src/`에 아래 symlink가 있어야 하고,
없으면 install의 `nvblox_costmap_layer.xml`이 빈 파일이 되어 플러그인 로드가 실패한다.
```bash
# Host에서
cd ~/VICA-smarthandle/vica_ros2_ws/src
ln -s ~/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_msgs   nvblox_msgs
ln -s ~/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_nav2   nvblox_nav2
ln -s ~/workspaces/isaac_ros-dev/src/isaac_ros_common               isaac_ros_common
cd .. && colcon build --symlink-install --packages-select nvblox_msgs nvblox_nav2
```
`vica_nav2`의 `test_nvblox_dependency_contract`가 이 상태를 감시한다.

**Goal 수락되는데 안 움직임**
→ 5절 튜닝(바닥 블랭킷). `/local_costmap/costmap`로 확인.

**장애물이 로봇 위/뒤에 얹힘 (TF 오배치)**
```bash
ros2 run tf2_tools view_frames          # base_link 부모가 base_footprint 하나뿐인지(중복 발행 X)
ros2 run tf2_ros tf2_echo odom camera_depth_optical_frame
```

**슬라이스가 아예 안 나옴(`topic hz` 무응답)**
→ 카메라(`run_d455.sh`) 먼저 확인: `ros2 topic hz /camera/camera/depth/image_rect_raw`.

---

## 8. 동기화 (정본 수정 후)

launch/config를 VICA repo에서 고쳤다면 컨테이너용 복사본을 다시 만든다(Host):
```bash
rm -rf ~/workspaces/isaac_ros-dev/src/vica_nvblox_bringup
cp -r ~/VICA-smarthandle/vica_ros2_ws/src/vica_nvblox_bringup ~/workspaces/isaac_ros-dev/src/
```
그다음 컨테이너에서 2-2 빌드를 다시.

> 장기적으로는 컨테이너 실행 스크립트에 VICA repo를 마운트하면 복사 없이 symlink로 끝난다(별도 과제).

---

## 참고

- 배경/원인 분석: `~/.claude/plans/nvblox-layer-temporal-lantern.md`
- nvblox 단독/설치 이력: Notion `0627_NVBLOX`, `0630_nvblox nav2 plugin/EKF 재구성`
