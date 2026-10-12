# mdrobot_can_control_cpp

`mdrobot_can_control`의 모터 노드(`keyboard_knob`)를 C++로 옮긴 판이다. 하는 일은 같다 —
`/cmd_vel_safe`를 받아 MDROBOT CAN 속도 명령으로 보내고, 최고속도는 드라이버가 올려 주는
knob1로 제한한다. E-stop 래치는 이 노드가 아니라 `vica_safety`가 소유한다.

```bash
ros2 launch mdrobot_can_control motor_bringup.launch.py                    # C++ 판(기본)
ros2 launch mdrobot_can_control motor_bringup.launch.py motor_impl:=python  # 파이썬 판
```

## 왜 옮겼나

이 노드는 주행·정지와 무관하게 30 Hz 타이머와 30 Hz `/cmd_vel_safe`를 돌린다. 기록
25회차(2026-07-31 ~ 10-12)에서 코어 하나의 평균 8.7 %(6.2 ~ 11.6 %)를 썼다. 격리 벤치(가상
CAN, 실제 버스와 같은 프레임 주기)에서 파이썬 5.4 ~ 6.7 %, C++ 1.0 ~ 1.3 %였다.

## 무엇이 같은가

- 노드 이름 `mdrobot_can_keyboard_knob_node`, 실행 파일 이름 `keyboard_knob`
  (터미네이터 guard `pgrep -x keyboard_knob`, CPU 기록기, 진단 프로브가 이 이름을 본다)
- 파라미터 이름·타입·기본값, 토픽(`/cmd_vel_safe` 구독, `/motor/can_ok` 발행)·QoS, 진단
  (`mdrobot_can_keyboard_knob_node: CAN link`)
- 제어 판정과 CAN 프레임 바이트, 로그 글자. 파이썬 `round()`의 짝수 쪽 반올림, NaN 명령이면
  그 사이클에 프로세스가 죽는 것까지 같다.
- python-can 소켓 옵션(오류 프레임 수신 포함)과 오류 글자(진단 `last_error`에 실린다)

`test/parity`가 같은 무작위 시나리오를 파이썬 원본 메서드와 C++ 핵심에 넣어 출력을 줄 단위로
비교한다. 판정이나 로그를 바꾸면 두 판을 함께 고치고 이 시험을 돌린다.

## 다른 것 (2026-10-12 격리 시험으로 확인)

- **터미널 Ctrl+C 때 정지 명령**: 노드는 SIGINT를 두 번 받는다(터미널 + launch 전달). 파이썬
  판은 두 번째 신호를 정지 명령 사이의 `time.sleep(0.02)`에서 맞아 KeyboardInterrupt로 죽어
  정지 프레임을 **한 번만** 보낸다(기존 약점). C++ 판은 두 번 다 보낸다. SIGTERM 하나만 받거나
  launch가 먼저 죽은 경우는 두 판 모두 두 번 보낸다.
- 종료 코드: 파이썬 판은 -2 또는 1(위 예외, 또는 원본 `main()`의 `rclpy.shutdown()` 이중 호출),
  C++ 판은 0("process has finished cleanly").
- 기동 실패(예: `[MOTOR START BLOCKED]`) 때 파이썬은 traceback을, C++는 오류 한 줄을 찍는다.
- rclcpp가 스스로 만드는 것: `qos_overrides./parameter_events.publisher.*` 파라미터 4개,
  `/parameter_events` 구독 하나. `/rosout` 로그의 파일·함수·줄 번호 칸.
- 밀리초 단위 실행 시각(C++가 빠르고 덜 흔들린다). 판정 논리는 같다.

같게 맞춘 신호 처리: 파이썬 인터프리터처럼 SIGPIPE를 무시한다. 그래야 launch가 먼저 죽어
화면 출력 통로가 끊겨도 노드가 즉사하지 않고 정지 판정을 이어 간다.
