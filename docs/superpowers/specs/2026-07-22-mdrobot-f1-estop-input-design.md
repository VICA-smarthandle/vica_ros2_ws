# MDROBOT F1 물리 E-stop 입력 전환 설계

## 목적

`motor_safety_bringup.launch.py`로 실행하는 `emergency_stop_node`가 시험 토픽이 아니라
MDROBOT의 실제 CAN F1 프레임을 물리 E-stop 입력으로 사용하게 한다.

## 변경 범위

- `motor_safety_bringup.launch.py`의 `input_mode`를 `can_f1`으로 설정한다.
- CAN 인터페이스 `can1`과 응답 ID `0x701`을 launch에 명시한다.
- motor node에 선언되지 않은 `estop_bit_pressed_value` launch 파라미터를 제거한다.
- 현재 앱·음성 입력과 `/emergency_stop` OR 집계는 유지한다.
- `/cmd_vel_req`, `/cmd_vel_safe`, reset 소유권 등 다른 Safety 계약은 변경하지 않는다.
- `safety_bringup.launch.py`의 시험 용도와 기본 입력 모드는 이번 범위에서 변경하지 않는다.

## 데이터 흐름

```text
MDROBOT 0x701 / F1 packet 0
→ emergency_stop_node(input_mode=can_f1)
→ physical_active OR app_active OR voice_active
→ /emergency_stop
→ safety_supervisor_node
→ /cmd_vel_safe
```

F1 입력이 아직 수신되지 않았거나 `f1_timeout_sec`보다 오래 끊기면 기존 fail-safe 동작대로
`/emergency_stop=true`를 발행한다.

## F1 판정과 미검증 항목

현재 코드는 packet index `0`, `data[2]`와 `data[3]`, mask `0x10`, active value `0`을
사용하며 두 바이트가 모두 active인 경우를 눌림으로 판정한다. 앞선 읽기 전용 캡처에서
`F1 00 08 48 04 05 30 30` 프레임 수신은 확인했지만 당시 버튼 상태가 확정되지 않았다.

따라서 이번 변경에서는 CAN ID, 바이트, mask, 극성 또는 AND/OR 판정식을 바꾸지 않는다.
버튼 해제·눌림 상태를 각각 캡처한 뒤 별도 근거로 판정식을 확정한다.

## 검증

1. launch 계약 테스트에서 `input_mode=can_f1`, `can1`, `0x701` 설정과 잘못된 motor
   파라미터 제거를 확인한다.
2. `mdrobot_can_control` 패키지만 build/test한다.
3. 안전 조건 확인 후 노드를 재실행하고 실제 파라미터가 `can_f1`인지 확인한다.
4. 속도 명령과 E-stop reset을 발행하지 않은 상태에서 버튼 해제·눌림 F1 프레임과
   `/emergency_stop` 변화를 함께 관찰한다.

## 안전 및 롤백

실기기 실행 전 바퀴 들림, 주변 통제, 물리 E-stop, 즉시 전원 차단 수단을 확인한다.
실행 중인 `can1`을 down/up 하지 않는다. 문제가 있으면 launch의 `input_mode`를
`test_topic`으로 되돌리고 패키지를 다시 빌드한다.
