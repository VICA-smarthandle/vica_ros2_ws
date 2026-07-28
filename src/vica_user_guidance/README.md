# vica_user_guidance

Smart Handle 사용자 안내 계층이다. EKF `/odom`의 yaw 변화량으로 좌·우 회전을 감지해
핸들의 서보(촉각)와 LED(시각)로 사용자에게 방향을 안내한다.

## 안전 경계

이 패키지는 **주행 명령 권한이 없는 순수 출력 계층**이다.

- `/cmd_vel_req`, `/cmd_vel_safe`를 발행하지 않는다.
- Nav2 goal을 보내거나 취소하지 않는다.
- E-stop 래치를 소유하거나 reset하지 않는다. `/estop_state`를 구독만 한다.
- 서보는 조향 장치가 아니다. 로봇 진행 방향에 영향을 주지 않는다.
- 햅틱은 알림이지 모터 정지 보증이 아니다.

Smart Handle 단절은 **표시·진단만** 한다. 이를 주행 정지 조건으로 삼을지는 별도
Safety 결정 사항이다.

## 구성

| 파일 | 역할 |
| --- | --- |
| `timebase.py` | 시간 판정 (STEADY_TIME + 정수 ns + 미수신 `None`) |
| `protocol.py` | 아두이노 1바이트 상태코드 프로토콜 상수 |
| `turn_detector.py` | yaw 변화량 → LEFT/RIGHT 판정 (순수 로직) |
| `guidance_priority.py` | E-stop > 도착 > 회전 > 기본 우선순위 병합 (순수 로직) |
| `serial_link.py` | 시리얼 전송 래퍼 |
| `turn_guide_node.py` | `/odom` → `/vica/turn_guide` |
| `user_guidance_driver_node.py` | cue 병합 → 시리얼 전송 + 진단 발행 |
| `firmware/` | 아두이노 나노 펌웨어와 bench 시험 도구 |

순수 로직 모듈은 `rclpy`에 의존하지 않으며, 시각은 전부 호출자가 정수 나노초로
주입한다. 하드웨어 없이 pytest로 검증할 수 있다.

## 펌웨어

`firmware/smart_handle_firmware/smart_handle_firmware.ino`가 서보·LED를 구동한다.
**ROS와 1바이트 상태코드 프로토콜을 공유**하므로 같은 패키지에 둔다. `protocol.py`의
상수를 바꾸면 펌웨어도 함께 바꿔야 하며, `test_protocol.py`가 두 값의 일치를
자동으로 검사한다.

```bash
export PATH="$HOME/bin:$PATH"
arduino-cli compile --fqbn arduino:avr:nano firmware/smart_handle_firmware
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano \
    firmware/smart_handle_firmware

# 로봇과 분리한 상태에서 상태코드를 수동 전송해 표시를 확인한다
python3 firmware/bench_test.py --list
python3 firmware/bench_test.py --all
```

필요 라이브러리: `Adafruit NeoPixel`, `Servo` (Servo는 AVR 코어에 미포함).

> 아두이노 IDE는 스케치 폴더명과 `.ino` 파일명이 같아야 하므로 `smart_handle_firmware/`
> 하위 디렉터리 구조를 유지한다.

## `timebase.py`가 `vica_safety/freshness.py`의 복제인 이유

guidance는 권한 없는 출력 계층이고 `vica_safety`는 안전 권한 계층이다. 여기서
`vica_safety`를 의존으로 끌어오면 의존 방향이 역전되고, `python3-can` 의존까지
따라온다. 대신 `test/test_timebase.py`가 두 구현의 계약 동일성을 고정한다.
**계약을 바꿀 때는 반드시 양쪽을 함께 바꾼다.**

## 검증

```bash
# 순수 로직 (rclpy 불필요)
cd src/vica_user_guidance && python3 -m pytest test -v

# 통합
colcon build --packages-select vica_interfaces vica_user_guidance
colcon test --packages-select vica_user_guidance
```

## 관련 문서

설계 근거와 bench 실측 결과는 작업공간 루트의
`devlog/2026-07-28-smart-handle-guidance-plan.md`에 있다(별도 저장소).
