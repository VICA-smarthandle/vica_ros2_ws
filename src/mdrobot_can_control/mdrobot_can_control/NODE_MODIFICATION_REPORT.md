# mdrobot_can_control 노드 코드 수정사항 보고서

작성일: 2026-06-01

## 1. 개요

본 보고서는 `mdrobot_can_control` 패키지 내 현재 노드 코드 상태를 기준으로, 모터 제어 및 노브 입력 처리와 관련된 주요 수정/구성 사항을 정리한 문서이다.

현재 패키지에는 ROS2 실행 노드인 `mdrobot_can_keyboard_knob_node.py`와 CAN/노브 단독 구동 테스트용 스크립트인 `knob_scale_drive.py`가 포함되어 있다.

## 2. 대상 파일

| 파일 | 역할 |
| --- | --- |
| `mdrobot_can_keyboard_knob_node.py` | `/cmd_vel`을 CAN 속도 명령으로 변환하고, 드라이버의 knob1 값을 최고속도 제한값으로 사용하는 ROS2 노드 |
| `knob_scale_drive.py` | ROS2 없이 CAN 노브 입력만으로 모터 RPM을 스케일링해 구동하는 테스트 스크립트 |
| `launch/mdrobot_can_control.launch.py` | 메인 ROS2 모터 노드 실행 및 파라미터 정리용 launch |
| `setup.py` | ROS2 console script 및 launch 파일 설치 설정 |

## 3. ROS2 메인 노드 수정사항

### 3.1 `/cmd_vel` 기반 모터 제어 구조

`mdrobot_can_keyboard_knob_node.py`는 `geometry_msgs/Twist` 타입의 `/cmd_vel`을 구독하여 키보드 또는 상위 제어기에서 들어오는 선속도/각속도 명령을 모터 RPM 명령으로 변환한다.

- 구독 토픽: `/cmd_vel`
- 노드명: `mdrobot_can_keyboard_knob_node`
- 실행 엔트리포인트: `keyboard_knob`
- CAN 인터페이스 기본값: `can1`
- 드라이버 ID 기본값: `0x001`

### 3.2 바퀴 및 모터 매핑 반영

차동구동 계산 결과를 MDROBOT 드라이버의 모터 번호에 맞게 매핑하였다.

- `MOT1`: 오른쪽 바퀴
- `MOT2`: 왼쪽 바퀴
- ROS 기준 `linear.x > 0`은 전진
- ROS 기준 `angular.z > 0`은 좌회전

계산식은 다음과 같다.

```text
v_left  = linear_x - angular_z * wheel_base / 2
v_right = linear_x + angular_z * wheel_base / 2
```

이후 오른쪽 바퀴 RPM은 `MOT1`, 왼쪽 바퀴 RPM은 `MOT2`로 송신된다.

### 3.3 knob1 기반 최고속도 제한

드라이버에서 수신되는 `0xF1` PNT I/O monitor 패킷에서 knob 값을 읽고, `knob1`을 최고속도 제한 비율로 사용하도록 구성하였다.

- `d[0] == 0xF1`인 패킷을 monitor 패킷으로 판정
- `d[1] == 0`일 때만 knob 데이터로 사용
- `d[6]`: `knob1`
- `d[7]`: `knob2`
- 현재 메인 ROS2 노드는 `knob1`만 최고속도 제한에 사용

`knob1` 값이 0~100% 범위에서 속도 제한 비율로 변환된다.

```text
speed_ratio = knob1 / 100
allowed_linear  = max_linear_mps * speed_ratio
allowed_angular = max_angular_radps * speed_ratio
```

### 3.4 안전 정지 조건 추가

입력 신호가 끊겼을 때 모터가 계속 움직이지 않도록 타임아웃 기반 안전 정지 조건을 적용하였다.

| 조건 | 기본값 | 동작 |
| --- | ---: | --- |
| knob 수신 타임아웃 | `0.8 sec` | knob 패킷이 끊기면 속도 제한값을 0으로 처리 |
| `/cmd_vel` 수신 타임아웃 | `0.5 sec` | 명령이 끊기면 선속도/각속도를 0으로 처리 |
| knob deadzone | `5 %` | knob 값이 5% 이하이면 정지로 처리 |

### 3.5 CAN 송신 주기 및 재전송 제한

CAN 버스 부하를 줄이기 위해 송신 타이머와 실제 재전송 조건을 분리하였다.

- `send_hz`: `30.0 Hz`
- 타이머 주기: 약 `33 ms`
- `resend_interval_sec`: `0.05 sec`
- 동일 RPM 명령은 매 타이머마다 무조건 보내지 않고, 명령 변경 또는 최소 재전송 간격 도달 시에만 송신

이 구조로 인해 제어 루프는 30Hz로 돌지만, 동일 명령 반복 송신은 최소 50ms 간격으로 제한된다.

### 3.6 return_type / ret_type 관련 정리

현재 메인 ROS2 노드의 속도 명령 송신은 다음과 같이 구분되어 있다.

| 상황 | ret_type |
| --- | ---: |
| 일반 주행 속도 명령 | `5` |
| 종료/정지 명령 | `0` |

일반 주행 명령에서 `ret_type=5`를 사용하는 이유는 엔코더/오돔 생성에 필요한 드라이버 리턴값을 유지하기 위함이다.

이전에 오돔이 생성되지 않았던 원인은 오돔용 `return_type` 값을 모터 ACK용 리턴값과 혼동하여 `5`에서 `0`으로 변경했기 때문이다. 현재 메인 제어 루프에서는 다시 `ret_type=5`로 송신하도록 되어 있으므로 엔코더 기반 오돔 수신 조건이 복구된 상태이다.

코드에서는 혼동을 줄이기 위해 `ret_type` 값을 다음 상수로 분리한다.

```python
RET_TYPE_NONE = 0
RET_TYPE_ODOM = 5
```

주행 중 속도 명령은 `RET_TYPE_ODOM`을 사용하므로, `encoder_feedback` 노드가 기본 수신 전용 모드여도 모터 드라이버가 위치 피드백 프레임을 반환할 수 있다. 종료/정지 명령은 `RET_TYPE_NONE`을 사용한다.

### 3.7 종료 시 모터 정지 처리

노드 종료 시 `destroy_node()`에서 `stop_motors()`를 호출하여 모터 정지 명령을 2회 송신한다.

- 정지 RPM: `(0, 0)`
- 정지 ret_type: `0`
- 1차 정지 후 `0.02 sec` 대기
- 2차 정지 명령 재송신

## 4. `knob_scale_drive.py` 테스트 스크립트 상태

`knob_scale_drive.py`는 ROS2 토픽을 사용하지 않고 CAN과 knob monitor 패킷만으로 모터를 직접 구동하는 초기 단독 테스트용 스크립트이다.

주요 설정은 다음과 같다.

| 항목 | 값 |
| --- | ---: |
| CAN 인터페이스 | `can1` |
| 드라이버 ID | `0x001` |
| 기준 RPM | `300` |
| 송신 주기 | `50 Hz` |
| deadzone | `5 %` |
| 최소 구동 RPM | `40` |
| knob 타임아웃 | `0.8 sec` |

이 스크립트는 `knob1`, `knob2`를 각각 `rpm1`, `rpm2`로 직접 스케일링하여 송신한다.

현재 테스트 스크립트의 일반 속도 명령은 `ret_type=2`를 사용한다. 메인 ROS2 노드의 오돔용 `RET_TYPE_ODOM=5`와 다르므로, 현재 통합 주행/오돔 검증 기준에서는 사용하지 않는다.

## 5. 실행 방법 및 launch 파라미터

`setup.py`의 `console_scripts`에는 현재 메인 ROS2 노드만 등록되어 있다.

```text
keyboard_knob = mdrobot_can_control.mdrobot_can_keyboard_knob_node:main
```

직접 실행 명령은 다음 형태가 된다.

```bash
ros2 run mdrobot_can_control keyboard_knob
```

통합 테스트에서는 launch 실행을 우선 사용한다.

```bash
ros2 launch mdrobot_can_control mdrobot_can_control.launch.py
```

launch에서 조정할 수 있는 주요 파라미터는 `can_iface`, `driver_id`, `wheel_radius_m`, `wheel_base_m`, `max_linear_mps`, `max_angular_radps`, `max_rpm`, `send_hz`, `resend_interval_sec`, `deadzone_pct`, `knob_timeout_sec`, `cmd_timeout_sec`, `invert_mot1`, `invert_mot2`, `min_rpm_when_moving`이다.

예시:

```bash
ros2 launch mdrobot_can_control mdrobot_can_control.launch.py max_rpm:=200 max_linear_mps:=0.3 max_angular_radps:=0.8
```

`knob_scale_drive.py`는 현재 `console_scripts`에 등록되어 있지 않으므로 ROS2 실행 명령으로 바로 실행되는 노드는 아니다.

## 6. 현재 핵심 파라미터 요약

| 파라미터 | 기본값 | 의미 |
| --- | ---: | --- |
| `can_iface` | `can1` | 사용할 SocketCAN 인터페이스 |
| `driver_id` | `0x001` | MDROBOT 드라이버 CAN ID |
| `wheel_radius_m` | `0.065` | 바퀴 반지름 |
| `wheel_base_m` | `0.37` | 좌우 바퀴 중심거리 |
| `max_linear_mps` | `1.0` | knob 100%일 때 최대 선속도 |
| `max_angular_radps` | `2.0` | knob 100%일 때 최대 각속도 |
| `max_rpm` | `400` | 모터 RPM 제한 |
| `send_hz` | `30.0` | 제어 루프 타이머 주기 |
| `resend_interval_sec` | `0.05` | 동일 명령 최소 재전송 간격 |
| `deadzone_pct` | `5` | knob 정지 처리 범위 |
| `knob_timeout_sec` | `0.8` | knob 수신 타임아웃 |
| `cmd_timeout_sec` | `0.5` | `/cmd_vel` 수신 타임아웃 |
| `min_rpm_when_moving` | `0` | 저속 구동 보정 RPM |

## 7. 정리

현재 패키지는 `/cmd_vel` 입력과 MDROBOT CAN 속도 명령을 연결하고, 드라이버 knob 값을 최고속도 제한 장치로 사용하는 구조로 정리되어 있다. 또한 knob 및 `/cmd_vel` 타임아웃, deadzone, 종료 시 정지 명령을 통해 기본적인 안전 정지 조건을 포함한다.

오돔 미생성 문제의 직접 원인은 오돔용 `return_type=5`를 모터 ACK용 값과 혼동하여 `0`으로 변경한 것이며, 현재 메인 노드의 주행 명령부는 `ret_type=5`로 복구되어 있다.

현재 통합 주행 기준은 `mdrobot_can_control.launch.py`로 메인 모터 노드를 실행하고, `encoder_feedback`은 기본 수신 전용 모드로 함께 실행하는 것이다. 실제 바닥 주행 전에는 바퀴를 띄운 상태에서 `/odom`, `odom -> base_link`, CAN 프레임 반환을 먼저 확인한다.
