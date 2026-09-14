# vica_mission_manager

음성 `VicaIntent`와 앱·CLI 목적지 요청을 같은 gate로 검사하고, 통과한 경우에만
Nav2 `NavigateToPose` Goal을 생성한다.

## 권한 경계

- LLM·앱·`vica_goto_goal.py`는 Nav2 Goal을 직접 발행하지 않는다.
- 공개 요청 서비스는 `/vica/mission/request_destination`이다.
- 물류 배송 요청은 `/vica/mission/request_delivery`다. 요청 모양은 같고 `private` 목적지만 추가로 허용한다. 접근 가능·지도 경계·Nav2·E-stop 검사는 그대로다.
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
| 서비스 | `/vica/mission/request_delivery` | `vica_interfaces/srv/RequestDestination` (물류 배송 전용, `private` 허용) |
| 서비스 | `/vica/mission/reload_destinations` | `std_srvs/srv/Trigger` |
| 발행 | `/vica_goal_event` | `std_msgs/msg/String` JSON |
| 발행 | `/vica/tts_request` | `std_msgs/msg/String` |
| 발행 | `/vica/robot_state` | `vica_interfaces/msg/RobotState` |
| 발행 | `/speed_limit` | `nav2_msgs/msg/SpeedLimit` |

## 목적지 접근 감속

Nav2 feedback의 잔여거리에 따라 `/speed_limit`(percentage)으로 최대속도 상한을 단계적으로
내린다. 급정지의 실체는 감속률이 아니라 속도 낙차(Δv)이므로, 세우기 전에 이미 느리게
만드는 설계다. `/speed_limit`은 `max_vel_x`·`max_vel_theta`만 바꾸고 `decel_lim_*`은
건드리지 않아 비상 제동력은 그대로다.

| 잔여거리 | 제한 | max_vel_x | 정지 시 Δv |
|---|---|---|---|
| ~1.5 m | 100 % | 0.260 | 0.260 |
| ≤ 1.5 m | 70 % | 0.182 | 0.182 |
| ≤ 1.0 m | 55 % | 0.143 | 0.143 |
| ≤ 0.5 m | 40 % | 0.104 | 0.104 |

한 번 내려간 제한은 그 Goal 동안 다시 올라가지 않는다(latch). Nav2 잔여거리는 재계획마다
출렁이므로, 그때마다 제한이 오르내리면 사용자 손에 울컥거림으로 전달된다. 성공·실패·취소·
일시정지·E-stop과 새 Goal 시작 시 `speed_limit=0.0`으로 해제하고 사다리를 초기화한다.

단계는 `approach_slowdown_distances_m`, `approach_speed_limit_percents` launch
parameter(double 배열, 순번끼리 짝)로 조정한다. 단계 개수도 바꿀 수 있고, 빈 배열이면
접근 감속을 끈다. 가까운 단계일수록 비율이 낮아야 하며 어기면 기동 시 거부한다.

첫 감속 지점은 3.0 m가 아니라 1.5 m다. 최고속도 0.26 m/s에서 3 m는 그 자체로 11.5초라,
아직 도착 준비가 필요 없는 구간을 미리 늦추면 손해만 크다는 판단이다(2026-08-01).
음성 안내는 그대로 3 m에서 나가므로 사용자는 먼저 듣고 그 다음 감속을 느낀다.

마지막 단계는 `max_vel_theta`도 0.16 rad/s로 낮추므로 **목적지 직전 0.5 m에 회전이 남으면
도착이 지연된다.** 실제 주행 종단은 `[미검증]`이다.

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
