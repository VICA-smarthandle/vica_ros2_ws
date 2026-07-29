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
| `udev/` | Smart Handle 고정 장치 이름 규칙 |

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
arduino-cli upload -p /dev/vica_smart_handle --fqbn arduino:avr:nano \
    firmware/smart_handle_firmware

# 로봇과 분리한 상태에서 상태코드를 수동 전송해 표시를 확인한다
python3 firmware/bench_test.py --list
python3 firmware/bench_test.py --all
```

필요 라이브러리: `Adafruit NeoPixel`, `Servo` (Servo는 AVR 코어에 미포함).

> 아두이노 IDE는 스케치 폴더명과 `.ino` 파일명이 같아야 하므로 `smart_handle_firmware/`
> 하위 디렉터리 구조를 유지한다.

## yaw 드리프트 진단 (`yaw_drift_check`)

회전 임계값(기본 25°)이 실주행에서 안전한지 판단하려면 "yaw가 얼마나 흔들리나"
보다 **"정지 상태에서 회전 오탐이 나는가"** 를 직접 재는 편이 낫다. 이 도구는
`turn_detector.TurnDetector`를 **그대로 재사용**해 실제 판정 경로로 측정한다.
로직을 복제하면 실제 노드와 달라져 측정이 무의미해진다.

**로봇을 움직이지 않는다.** `/odom`을 구독만 하며 어떤 명령도 발행하지 않는다.

```bash
ros2 launch vica_localization wheel_ekf.launch.py          # 다른 터미널
ros2 run vica_user_guidance yaw_drift_check --ros-args \
    -p duration_sec:=300.0 -p csv_path:=/tmp/yaw.csv
```

측정 중 로봇을 건드리지 않는다. 사람이 기대거나 바닥이 흔들리면 IMU가 반응해
실제보다 나쁘게 나온다.

판정 기준:

| 결과 | 조건 | 조치 |
| --- | --- | --- |
| 안전 | 오탐 0, 임계값까지 여유 3배 이상 | 그대로 진행 |
| 주의 | 오탐 0, 여유 3배 미만 | Phase 5b에서 임계값 재검토 |
| 위험 | 정지 상태 오탐 발생 | **임계값이 아니라 EKF 설정을 먼저 본다** |

`/odom`이 오지 않으면 15초 뒤 원인을 안내하고 종료한다. EKF가 떠 있어도
`/imu/base_link`나 `/wheel/odom`이 없으면 `/odom`이 나오지 않는다.

## 장치 이름 고정 (udev)

`/dev/ttyUSB*`의 번호는 USB 재열거 순서에 따라 바뀐다. `udev/99-vica-smart-handle.rules`
가 `/dev/vica_smart_handle` 심볼릭 링크를 고정 부여한다.

```bash
sudo cp udev/99-vica-smart-handle.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/vica_smart_handle
```

**링크를 확인한 뒤에** `config/user_guidance.yaml`의 `serial_port`를 바꾼다. 규칙 없이
먼저 바꾸면 모든 실행이 `FAULT_PORT_OPEN`이 된다.

규칙은 보드 시리얼(`B003UMKG`)까지 조건에 넣는다. FTDI `0403:6001`은 흔한 값이라
이것만으로는 다른 USB-시리얼 장치에도 링크가 걸린다. **보드를 교체하면 규칙도 함께
갱신해야 한다.**

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
