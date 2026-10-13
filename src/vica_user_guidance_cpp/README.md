# vica_user_guidance_cpp

Smart Handle 드라이버 `user_guidance_driver_node`의 C++ 판(2026-10-13). 파이썬 판
(`vica_user_guidance/user_guidance_driver_node.py`)과 실행 파일·노드 이름·파라미터·토픽·QoS·로그 글자가
같다. `user_guidance.launch.py`가 기본으로 이 판을 띄우고, `driver_impl:=python`이면 파이썬 판을 띄운다.

## 왜 옮겼나

이 노드는 주행·정지와 무관하게 상향 시리얼 읽기 20 Hz·상태 전송 10 Hz·진단 2 Hz를 돌리고, 초음파 TF
게이트 때문에 `/tf`(약 135 건/s)를 모두 받는다. 파이썬에서 코어 하나의 15~23 %를 썼다(run89·90·93 자원
기록). C++ 판은 TF 수신을 `tf2_ros::TransformListener`의 자기 스레드에서 받고, 나머지 콜백은 한 줄에서
하나씩 돈다(파이썬 판의 2스레드 실행기와 같은 구조 — "시리얼 읽기는 uplink_loop 한 곳").

## 구성

| 파일 | 역할 | 파이썬 판 |
| --- | --- | --- |
| `driver_core.*` | 콜백·주기 처리의 판정과 순서 | `user_guidance_driver_node.py` 메서드 |
| `guidance.*` | 우선순위 병합·도착 이벤트·TF 게이트·신선도 | `guidance_priority.py`·`range_tf_gate.py`·`timebase.py` |
| `frames.*` | 상향 프레임 누적기(초음파·터치·통계·설정) | `ultrasonic_frame.py`·`touch_frame.py`·`ultrasonic_stats.py` |
| `serial_link.*`, `posix_serial.cpp` | 시리얼 래퍼, pyserial 3.5 동작을 옮긴 tty | `serial_link.py` + pyserial |
| `pyjson.*` | `json.loads` 와 같은 문법(NaN·Infinity 허용 — 미션 `json.dumps` 기본값) | `json` |
| `pyfmt.*` | 로그 글자를 같게: `repr(float)`·`repr(str)`·`int()`·`round()` | — |
| `user_guidance_driver_node.cpp` | 파라미터·구독·발행·타이머·TF | `__init__`·`main` |

## 시험

```bash
colcon build --packages-select vica_user_guidance_cpp
colcon test --packages-select vica_user_guidance_cpp
```

- `test/parity`: 무작위 시나리오를 **파이썬 원본 노드의 메서드**와 C++ 코어에 똑같이 넣고 시리얼 바이트·
  포트 열고 닫기·Range/SmartHandleState·TF 조회·로그 글자를 줄 단위로 비교한다(기본 설정 30 시드 × 100 초,
  설정을 바꾼 66 가지, 기동 실패 문구). law_py 는 진짜 rclpy 노드를 만들므로 도메인 231·localhost 로만 띄운다.
- `test_guidance_core`: 부품 시험과 pty(가짜 tty)로 쓰기·읽기·선 뽑힘 복구.
- `test_source_contract.py`: 주행 명령 심볼 없음, `/estop_state` 는 구독만, 이름·파라미터 순서가 파이썬과 같음.

## 파이썬 판과 일부러 다른 점

- `/vica_goal_event` 가 약 1000 겹 넘게 중첩된 JSON 이면 파이썬은 RecursionError 로 노드가 죽는다. C++ 은
  512 겹을 넘으면 파싱 실패 경고 한 줄로 넘긴다.
- 설정 오류로 기동을 막을 때 파이썬은 트레이스백 전체를, C++ 은 그 마지막 줄(`ValueError: …`)만 stderr 에
  찍는다. 종료 코드는 둘 다 1.
- `repr(str)` 의 출력 가능 판정은 흔한 비출력 범위만 맞춘다(아직 배정 안 된 유니코드는 출력 가능으로 본다).
  도착 이벤트 파싱 실패 경고의 payload 표시에만 쓰인다.

## 빌드·배포 주의

- `vica_interfaces`(TurnGuide·SmartHandleState)를 바꾸면 **이 패키지도 다시 빌드한다.** C++ 판은 메시지 모양을
  컴파일할 때 박아 넣는다 — 옛 바이너리가 새 메시지와 짝이 어긋나면 조용히 값이 깨질 수 있다(파이썬 판은 그때그때 읽어
  이 문제가 없다). 메인 작업공간에서는 `colcon build --packages-up-to vica_user_guidance` 가 둘 다 짓는다.
- C++ 판이 설치돼 있지 않으면 launch 가 경고를 남기고 파이썬 판으로 띄운다(방향지시까지 같이 꺼지지 않게).
- 덧씌워 시험할 때(`source ~/wt-guidance-cpp/install/local_setup.bash`)는 손잡이 칸에서 직접 띄운다.
  `scripts/run_handle.sh` 는 `env -i` 로 메인 작업공간만 읽어 덧씌움을 무시한다. 확인:
  `readlink /proc/$(pgrep -x user_guidance_d)/exe` 가 `…/vica_user_guidance_cpp/…` 이면 C++ 판이다.

## 되돌리기

```bash
ros2 launch vica_user_guidance user_guidance.launch.py driver_impl:=python
```

파이썬 판 코드는 이 판의 정답지라 지우지 않는다. 파이썬 판을 고치면 여기도 고치고 대조 시험으로 맞춘다.
