# vica_user_guidance_cpp

Smart Handle 안내 계층 두 노드의 C++ 판(2026-10-13).

| 실행 파일 | 하는 일 | 파이썬 판 |
| --- | --- | --- |
| `user_guidance_driver_node` | 손잡이 시리얼(서보·LED·진동 전송, 초음파·터치 수신) | `user_guidance_driver_node.py` |
| `turn_guide_node` | /odom yaw 로 회전 판정 + 레일 코너 예고 → `/vica/turn_guide` | `turn_guide_node.py` |

두 판은 실행 파일·노드 이름·파라미터·토픽·QoS·로그 글자가 같다. `user_guidance.launch.py`가 기본으로 C++ 판을
띄우고, `driver_impl:=python` · `turn_impl:=python` 으로 하나씩 파이썬 판으로 되돌린다.

## 왜 옮겼나

두 노드 다 주행·정지와 무관하게 늘 돈다. 손잡이 드라이버는 상향 시리얼 읽기 20 Hz·상태 전송 10 Hz·진단 2 Hz,
방향지시는 판정 20 Hz 에 /odom 30 Hz 를 받는다. 그리고 둘 다 TF 를 쓰느라 `/tf`(약 135 건/s)를 모두 받는다
(드라이버는 초음파 TF 게이트, 방향지시는 레일 예고의 로봇 위치). 파이썬에서 드라이버가 코어 하나의 15~23 %,
방향지시가 12 % 안팎을 썼다(run89~94 자원 기록).

| 같은 부하에서 CPU | 파이썬 | C++ |
| --- | --- | --- |
| 손잡이 드라이버(격리 시험 /tf 부하) | 17.8 % | 1.6 % |
| 손잡이 드라이버(run94 실주행) | 20.9 %(정지 중 실측) | 2.8 %(중앙) |
| 방향지시(run94 녹화 실시간 재생, 두 판 동시) | 11.6 % | 1.9 % |

C++ 판은 TF 수신을 `tf2_ros::TransformListener`의 자기 스레드에서 받고, 나머지 콜백은 한 줄에서 하나씩 돈다
(드라이버는 "시리얼 읽기는 uplink_loop 한 곳", 방향지시는 판정·/odom·레일이 한 줄).

- 드라이버 파이썬 판도 같은 구조다(2스레드 실행기 + TF 구독만 재진입 그룹).
- 방향지시 파이썬 판은 `TransformListener(spin_thread=True)` 로 TF 를 따로 받으려 했지만, 뒤이은
  `rclpy.spin(node)` 가 노드를 전용 실행기에서 빼 가 **실제로는 TF 도 판정과 한 줄에서 돈다**(Humble
  `Node.executor` setter, 2026-10-13 검토). 판정 규칙은 같고, C++ 판이 TF 를 더 최신으로 본다 — run94 재생에서
  신호가 중앙 29 ms 일찍 나왔다(C++ 이 늦은 쪽은 최대 22 ms, 20 Hz 타이머 위상 차).

## 구성

| 파일 | 역할 | 파이썬 판 |
| --- | --- | --- |
| `driver_core.*` | 드라이버 콜백·주기 처리의 판정과 순서 | `user_guidance_driver_node.py` 메서드 |
| `guidance.*` | 우선순위 병합·도착 이벤트·TF 게이트·신선도 | `guidance_priority.py`·`range_tf_gate.py`·`timebase.py` |
| `frames.*` | 상향 프레임 누적기(초음파·터치·통계·설정) | `ultrasonic_frame.py`·`touch_frame.py`·`ultrasonic_stats.py` |
| `serial_link.*`, `posix_serial.cpp` | 시리얼 래퍼, pyserial 3.5 동작을 옮긴 tty | `serial_link.py` + pyserial |
| `turn_core.*` | 방향지시 콜백·주기 처리의 판정과 순서 | `turn_guide_node.py` 메서드 |
| `turn_detector.*` | yaw 창 누적 회전 판정 | `turn_detector.py` |
| `rail_turn_forecast.*` | 레일 코너 찾기·투영·예고/사후 판정 합치기 | `rail_turn_forecast.py` |
| `pymath.*` | `math.hypot`·`math.dist`(CPython 3.10 vector_norm)·`atan2` 특수값 | `math` |
| `rclpy_compat.*` | 타이머 주기·Time/Duration·rosidl 칸 검사의 예외를 같은 글자로 | rclpy·rosidl |
| `pyjson.*` | `json.loads` 와 같은 문법(NaN·Infinity 허용 — 미션 `json.dumps` 기본값) | `json` |
| `pyfmt.*` | 로그 글자를 같게: `repr(float)`·`repr(str)`·`int()`·`round()` | — |
| `*_node.cpp` | 파라미터·구독·발행·타이머·TF | `__init__`·`main` |

## 시험

```bash
colcon build --packages-select vica_user_guidance_cpp
colcon test --packages-select vica_user_guidance_cpp
```

- `test/parity`(드라이버): 무작위 시나리오를 **파이썬 원본 노드의 메서드**와 C++ 코어에 똑같이 넣고 시리얼 바이트·
  포트 열고 닫기·Range/SmartHandleState·TF 조회·로그 글자를 줄 단위로 비교한다(기본 설정 30 시드 × 100 초,
  설정을 바꾼 66 가지, 기동 실패 문구).
- `test/parity`(방향지시, `turn_*`): 가상 로봇이 레일(코너·U턴·S자·같은 방향 연속 코너)을 달리는 시나리오 —
  출발 제자리 회전·비켜가기·레일 이탈·정지·도착 정렬·목적지 바꾸기·/odom 끊김·TF 끊김·1 Hz 로 다시 잘려 오는
  레일·frame 이 다른/짧은/NaN 섞인 레일. TurnGuide 칸(float32 는 바이트)·로그·레일 판정 이유(prepare·now_inherit·
  prepare_chain…)를 줄 단위로 비교한다. TF 는 양쪽 다 진짜 tf2 버퍼라 조회 실패 글도 비교된다. 기본 설정 30 시드 ×
  100 초, 설정을 바꾼 56 가지, ROS 시계 끝값 4 가지, 멈추는 오류 문구 7 가지.
  `TURN_BAGS=<녹화 폴더>:<녹화 폴더>` 를 주면 실주행 녹화(/odom·/tf·/rail_plan)도 재생해 비교한다
  (`turn_bag_scenario.py`). 2026-10-13 run90~94 다섯 개가 글자까지 같았고, 재생한 파이썬 출력이 녹화된 실제
  노드의 신호 시작과 run91~94 에서 전부(68/68) 맞았다.
- 두 대조 시험의 law_py 는 진짜 rclpy 노드를 만든다 — 도메인 231·localhost 로만 띄운다.
- `test_guidance_core`: 부품 시험과 pty(가짜 tty)로 쓰기·읽기·선 뽑힘 복구.
- `test_source_contract.py`: 주행 명령 심볼 없음, `/estop_state` 는 구독만, 방향지시는 `/vica/turn_guide` 만 발행,
  이름·파라미터 순서가 파이썬과 같음.
- `test/integration`(수동, 격리 네트워크 공간): `run_both.py`(드라이버 — 진짜 pty·USB 뽑기),
  `turn_run_both.py <녹화>`(방향지시 — 두 판을 함께 띄우고 녹화를 실시간 재생, 신호·CPU·Ctrl+C·기동 실패 비교).

## 파이썬 판과 일부러 다른 점

- `/vica_goal_event` 가 약 1000 겹 넘게 중첩된 JSON 이면 파이썬은 RecursionError 로 노드가 죽는다. C++ 은
  512 겹을 넘으면 파싱 실패 경고 한 줄로 넘긴다.
- 설정 오류로 기동을 막을 때 파이썬은 트레이스백 전체를, C++ 은 그 마지막 줄(`ValueError: …`)만 stderr 에
  찍는다. 종료 코드는 둘 다 1.
- 방향지시 파이썬 판은 레일 예고를 켠 채 기동에 실패하면(예: `publish_rate_hz: 0.0`) 트레이스백을 찍고도
  **프로세스가 끝나지 않는다** — 먼저 띄운 TF 리스너 스레드(비데몬)가 남는다. C++ 판은 종료 코드 1 로 바로 끝난다.
- 파라미터 형이 틀리거나(실수 자리에 정수, 예 `publish_rate_hz: 20`) 토픽 이름이 틀리면(예 `odom_topic: "odom topic"`)
  두 판 다 기동을 거부하지만 문구는 rclpy·rclcpp 것이라 다르다. 그 앞까지 찍는 로그 줄은 같다.
- C++ 의 `TransformListener` 는 `qos_overrides./tf…` 파라미터를 선언해 `ros2 param list` 에 파이썬 판에 없는 줄이
  보인다. `tf2_ros::Buffer(노드 시계)` 는 sim time 이 거꾸로 가면 TF 버퍼를 비운다(use_sim_time 일 때만 — 로봇은 안 씀).
- 주기가 int64 ns 를 넘는 타이머(예 `publish_rate_hz: 1e-12`·`-1e-12`)는 두 판 다 TypeError 로 멈추지만, 파이썬
  문구에는 메모리 주소가 섞여 C++ 은 줄인 글(`… (timer period)`)을 찍는다.
- 레일 좌표가 inf 이거나 거대하면(1e37 수준) 레일 다시 찍기(`_resample`)가 끝나지 않는다 — 두 판 같은 결함이다
  (파이썬은 CPU 100 % 로 계속 돌고, C++ 은 메모리가 바닥나 종료 코드 1). Route Server 레일에서는 생기지 않는다.
- `repr(str)` 의 출력 가능 판정은 흔한 비출력 범위만 맞춘다(아직 배정 안 된 유니코드는 출력 가능으로 본다).
  도착 이벤트 파싱 실패 경고의 payload 표시에만 쓰인다.

Ctrl+C 때 방향지시는 두 판 다 마지막 IDLE 을 **못 보낸다**(rclpy·rclcpp 가 먼저 통신을 닫는다 — 2026-10-13
격리 시험 0/1). 소비자(손잡이 드라이버)는 cue 1 초 시한으로 스스로 풀린다.

## 빌드·배포 주의

- `vica_interfaces`(TurnGuide·SmartHandleState)를 바꾸면 **이 패키지도 다시 빌드한다.** C++ 판은 메시지 모양을
  컴파일할 때 박아 넣는다 — 옛 바이너리가 새 메시지와 짝이 어긋나면 조용히 값이 깨질 수 있다(파이썬 판은 그때그때 읽어
  이 문제가 없다). 메인 작업공간에서는 `colcon build --packages-up-to vica_user_guidance` 가 둘 다 짓는다.
- C++ 판이 설치돼 있지 않으면 launch 가 경고를 남기고 파이썬 판으로 띄운다(손잡이 안내가 통째로 꺼지지 않게).
- 덧씌워 시험할 때(`source ~/wt-guidance-cpp/install/local_setup.bash`)는 손잡이 칸에서 직접 띄운다.
  `scripts/run_handle.sh` 는 `env -i` 로 메인 작업공간만 읽어 덧씌움을 무시한다. 확인:
  `readlink /proc/$(pgrep -x user_guidance_d)/exe`·`readlink /proc/$(pgrep -x turn_guide_node)/exe` 가
  `…/vica_user_guidance_cpp/…` 이면 C++ 판이다(파이썬 판은 `/usr/bin/python3.10`).
- 실행 중인 노드가 쓰는 설치본을 다시 빌드해도 그 노드는 옛 파일을 쥔 채 계속 돈다(`readlink` 끝에 `(deleted)`).
  새 빌드는 칸을 다시 띄울 때부터 적용된다.

## 되돌리기

```bash
ros2 launch vica_user_guidance user_guidance.launch.py driver_impl:=python turn_impl:=python
```

파이썬 판 코드는 이 판의 정답지라 지우지 않는다. 파이썬 판을 고치면 여기도 고치고 대조 시험으로 맞춘다.
