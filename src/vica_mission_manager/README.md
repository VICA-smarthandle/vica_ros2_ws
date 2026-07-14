# vica_mission_manager

VICA 음성→주행 통합의 유일한 신규 노드 (통합 진행순서 ②).
`/vica/intent`(LLM의 '제안')를 게이트로 심사해, **통과한 경우에만**
`nav2_simple_commander`로 NavigateToPose를 보낸다.

## 안전 원칙 (불변)

- LLM/음성 파트는 `/cmd_vel`·Nav2 goal을 직접 발행하지 않는다 — 이 노드가 유일한 관문.
- 모터 정지의 권위는 `/emergency_stop` 래치 체인. 이 노드의 goal 취소는 심층 방어 보조 경로.
- estopped 해제 후 이전 goal 자동 재개 금지 — 사용자가 다시 요청해야 한다.

## 게이트 5조건 (전부 AND)

1. `intent == "navigate"`
2. `matched_destination_id != ""`
3. `need_confirm == false`
4. `safety_flag == "normal"`
5. pose 유효: calibrated + (0,0) 아님 + `frame_id=="map"` + 지도 경계 내

추가 문맥 조건: E-stop 활성 시 거부, 주행 중 새 요청 거부(v1), Nav2 action server 미준비 시 거부.

## 상태 머신

`idle / confirming / navigating / arrived / failed / estopped` —
전이 다이어그램은 `projectVica/VICA_LLM_로봇_통합_진행순서.md` §4 참조.

## 토픽

| 방향 | 토픽 | 타입 |
|---|---|---|
| 구독 | `/vica/intent` | vica_interfaces/VicaIntent |
| 구독 | `/vica/emergency` | vica_interfaces/EmergencyEvent (reliable, 전용 callback group) |
| 구독 | `/emergency_stop` | std_msgs/Bool (emergency_stop_node 20Hz 래치 상태) |
| 발행 | `/vica/tts_request` | std_msgs/String (ros_tts_node가 구독 — 음성 파트 작업) |
| 발행 | `/vica/robot_state` | vica_interfaces/RobotState (1Hz) |

## 실행

```bash
source /opt/ros/humble/setup.bash
source ~/tony/vica_ros2_ws/install/setup.bash
ros2 launch vica_mission_manager mission_manager.launch.py
# 경로 변경 시:
#   destinations_yaml:=/절대/경로/destinations.yaml map_yaml:=/절대/경로/map.yaml
```

## 구조와 테스트

- `mission_logic.py` — 게이트·상태 전이 **순수 로직 (rclpy 비의존)**. 모든 판단은 여기.
- `destinations.py` — destinations.yaml / Nav2 지도 yaml 로더 (rclpy 비의존).
- `mission_manager_node.py` — ROS 배선만. MultiThreadedExecutor, emergency 전용 callback group.

```bash
cd src/vica_mission_manager && python3 -m pytest test/ -q   # 65 tests
```

## 주의 (함정)

- destinations.yaml 대부분의 pose가 (0,0) 플레이스홀더 — 게이트 ⑤가 거부한다.
  캘리브레이션(진행순서 ①) 후 `calibrated: true`를 yaml에 명시할 것.
- yaw는 도(deg) 단위 저장, 쿼터니언 변환은 goal 생성 시에만.
- launch에서 `name=`을 지정하지 말 것 — 프로세스 내 BasicNavigator까지 리매핑되어 이름 충돌.
