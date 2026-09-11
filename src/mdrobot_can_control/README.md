# mdrobot_can_control

`/cmd_vel_safe`를 MDROBOT CAN 속도 명령으로 변환하는 actuator adapter다. E-stop latch와
reset 권한은 이 패키지가 아니라 `vica_safety`가 소유한다.

## 실행

실제 장치에서는 바퀴를 띄우고 주변 통제, 물리 E-stop과 즉시 전원 차단 수단을 확보한 뒤
실행한다.

```bash
ros2 launch mdrobot_can_control motor_bringup.launch.py
```

## CAN 시작 전 검사

motor node는 SocketCAN 객체를 열거나 모터 명령을 보내기 전에
`/sys/class/net/<can_iface>/flags`의 Linux `IFF_UP` 비트를 읽기 전용으로 확인한다.

- 인터페이스가 존재하고 `IFF_UP`이면 실행한다.
- 인터페이스가 없거나 DOWN이면 `[MOTOR START BLOCKED]` 오류를 출력하고 node 시작을
  중단한다.
- motor node는 `can1`을 자동으로 UP하거나 bitrate를 변경하지 않는다.

오류 예시:

```text
[FATAL] [mdrobot_can_keyboard_knob_node]:
[MOTOR START BLOCKED] CAN interface can1 is DOWN;
configure and bring it UP before starting the motor
```

읽기 전용 상태 확인:

```bash
ip -details link show can1
```

CAN 설정은 motor·encoder·물리 E-stop이 공유하는 시스템 인프라다. 확정된 bitrate와
권한 정책에 따라 ROS motor launch보다 먼저 별도로 구성한다.
