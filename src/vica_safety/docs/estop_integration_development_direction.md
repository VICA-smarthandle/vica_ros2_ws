# VICA E-stop·Safety 통합 방향

## 책임 경계

| 구성요소 | 책임 |
| --- | --- |
| `emergency_stop_node` | 물리 F1·앱·음성 source 관리, 중앙 E-stop latch, 내부 latch reset |
| `safety_supervisor_node` | `/cmd_vel_req` freshness·0 명령·E-stop 검사, `/cmd_vel_safe` 최종 승인 |
| `app_emergency_node` | 앱·유지보수 공개 reset, Nav2 활성 Goal 확인·필요 시 전체 취소, 두 내부 reset 순서 제어 |
| `mdrobot_can_control` | 승인된 `/cmd_vel_safe`를 CAN motor 명령으로 변환 |

motor node는 E-stop 래치, `/estop_reset`, reset 오케스트레이션을 소유하지 않는다.

## 중앙 래치

다음 중 하나라도 발생하면 `/emergency_stop=true`를 중앙 래치한다.

- CAN F1 물리 버튼 활성
- `/app_emergency_stop=true`
- `/voice_emergency_stop=true`
- F1 상태가 아직 없거나 timeout으로 stale

물리 버튼을 놓거나 앱·음성 입력이 false가 되어도 래치는 자동 해제되지 않는다. 내부
`/vica_safety/internal/estop_reset`은 모든 source가 false이고 F1이 fresh할 때만 래치를
해제한다. 이 서비스는 공개 운영 인터페이스가 아니다.

## 공개 reset 절차

`/app_estop_reset`과 유지보수 `/safety_reset`은 같은 callback을 사용한다.

1. 앱 E-stop source를 false로 내린다.
2. Nav2 action status의 마지막 상태값을 확인한다. accepted/executing/canceling Goal이
   있으면 전체 취소 후 취소 요청보다 새로운 terminal 상태를 기다리고, 마지막 상태가
   terminal이면 cancel 호출을 생략한다. status 수신 이력이 없으면 Goal이 한 번도
   생성되지 않은 정상 상태로 판정해 Goal 검사를 생략한다. action server도 없으면 Nav2
   미실행으로 구분한다.
3. 중앙 E-stop latch 내부 reset을 호출한다.
4. fresh `/emergency_stop=false`를 확인한다.
5. Supervisor 내부 reset을 호출한다.
6. `/safety_state=READY_TO_GO`를 확인한다.

Nav2 status를 한 번도 수신하지 않았다면 action server 존재 여부와 관계없이 Goal 검사를
생략한다. action status는 주기 heartbeat가 아니라 상태 변경 이벤트이므로 메시지 나이를
reset 조건으로 사용하지 않는다. 마지막 상태가 활성 상태이면 수신 시각과 관계없이
취소하고 새 terminal 상태를 확인한다. Goal 검사를 생략하더라도 Supervisor의
`/cmd_vel_req=0` 또는 stale 확인은 생략하지 않는다. 어느 단계든 실패하면 이후 단계로
진행하지 않으며 이전 Goal은 자동 재개하지 않는다.
관리자 인증은 아직 구현되지 않았고 `/safety_reset`도 호출자를 식별하지 못하는 `[GAP]`이다.

## 로그

Safety launch는 `RCUTILS_COLORIZED_OUTPUT=1`을 설정한다. ROS 2는 상태값 한 단어가 아니라
로그 심각도에 따라 로그 한 줄 전체를 색으로 표시한다.

| 상태 | 심각도 | 표식 |
| --- | --- | --- |
| E-stop 활성 | ERROR | `[ESTOP ACTIVE]` |
| 입력 stale·CAN fault | ERROR | `[FAULT]` |
| source 해제 후 reset 대기 | WARN | `[WAIT RESET]` |
| reset 거부 | WARN/ERROR | `[RESET REJECTED] step=... reason=...` |
| reset 전체 성공 | INFO | `[RESET ACCEPTED] ... estop=False` |
| 주행 준비·정상 주행 | INFO | `[SAFETY READY]`, `[RUNNING]` |

상태 전이는 즉시 한 번 출력하며 raw F1 반복 로그와 CAN 오류 로그는 throttle한다.

## 독립 실행 경계

```bash
ros2 launch vica_safety safety_bringup.launch.py
ros2 launch vica_localization wheel_ekf.launch.py
ros2 launch mdrobot_can_control motor_bringup.launch.py
```

Localization·SLAM·Nav2 실행이 motor를 자동 기동하지 않도록 세 launch를 결합하지 않는다.
CAN F1, 실제 버튼, Nav2 live Goal 취소, motor 0 출력 및 실제 reset은 `[미검증]`이다.
