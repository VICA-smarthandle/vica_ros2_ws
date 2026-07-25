# vica_mission_manager

음성 `VicaIntent`와 앱·CLI 목적지 요청을 같은 gate로 검사하고, 통과한 경우에만
Nav2 `NavigateToPose` Goal을 생성한다.

## 권한 경계

- LLM·앱·`vica_goto_goal.py`는 Nav2 Goal을 직접 발행하지 않는다.
- 공개 요청 서비스는 `/vica/mission/request_destination`이다.
- 요청의 `destination_id`는 현재 지도 catalog에 존재하는 UUID v4여야 한다.
- `authorization != public` 또는 `is_approachable == false`인 목적지는 거부한다.
- E-stop 해제 뒤 이전 Goal을 자동 재개하지 않는다.

## 인터페이스

| 방향 | 인터페이스 | 타입 |
|---|---|---|
| 구독 | `/vica/intent` | `vica_interfaces/msg/VicaIntent` |
| 구독 | `/vica/emergency` | `vica_interfaces/msg/EmergencyEvent` |
| 구독 | `/emergency_stop` | `std_msgs/msg/Bool` |
| 서비스 | `/vica/mission/request_destination` | `vica_interfaces/srv/RequestDestination` |
| 서비스 | `/vica/mission/reload_destinations` | `std_srvs/srv/Trigger` |
| 발행 | `/vica_goal_event` | `std_msgs/msg/String` JSON |
| 발행 | `/vica/tts_request` | `std_msgs/msg/String` |
| 발행 | `/vica/robot_state` | `vica_interfaces/msg/RobotState` |
| 발행 | `/speed_limit` | `nav2_msgs/msg/SpeedLimit` |

Nav2 feedback의 잔여거리가 기본 3.0 m 이하가 되면 Goal별로 70% 속도 제한을 래치한다.
성공·실패·취소·E-stop과 새 Goal 시작 시 `speed_limit=0.0`으로 해제한다. 거리와 비율은
`approach_slowdown_distance_m`, `approach_speed_limit_percent` launch parameter로
조정할 수 있으며 실제 주행 종단은 `[미검증]`이다.

목적지 정본은 다음 지도별 파일이다.

```text
~/vica_data/destinations/<map_id>/destinations.yaml
```

파일이 아직 없으면 빈 catalog로 시작한다. 기존 `locations.json`이나
`vica-voice-llm/config/destinations.yaml`은 자동 이관하지 않는다.

## 실행

```bash
source /opt/ros/humble/setup.bash
source vica_ros2_ws/install/setup.bash
ros2 launch vica_mission_manager mission_manager.launch.py \
  map_id:=vica_map_0630 \
  map_yaml:=vica_ros2_ws/maps/vica_map_0630.yaml
```

## 테스트

```bash
colcon test --packages-select vica_mission_manager
```
