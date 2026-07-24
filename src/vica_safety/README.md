# vica_safety

VICA의 모터 패키지와 독립된 소프트웨어 안전 계층이다. 물리 CAN F1·앱·음성 E-stop을
중앙 래치하고, `/cmd_vel_req`를 검사해 `/cmd_vel_safe`만 모터에 전달한다. 공개 reset은
Nav2가 실행 중이면 fresh status의 활성 Goal만 전체 취소하고, 처음부터 미실행이면 Goal
검사를 생략한 뒤 `READY_TO_GO`까지 오케스트레이션한다.

## 실행

```bash
ros2 launch vica_safety safety_bringup.launch.py
```

이 launch는 다음 세 노드만 실행하며 motor node는 포함하지 않는다.

- `emergency_stop_node`
- `safety_supervisor_node`
- `app_emergency_node`

기본 물리 입력은 `can_f1`, `can1`, response ID `0x701`이다. 실제 장치 실행은 바퀴를
띄우고 주변 통제, 물리 E-stop, 즉시 전원 차단 수단을 확보한 뒤 수행한다.

## Reset

```bash
ros2 service call /safety_reset std_srvs/srv/Trigger "{}"
```

`/safety_reset`은 영구 유지보수 인터페이스다. Flutter의 `/app_estop_reset`과 동일한
오케스트레이션을 실행하며 내부 안전 검사를 우회하지 않는다. 현재 두 `Trigger` 서비스에는
호출자 인증 정보가 없으므로 관리자 인증과 접근 통제는 `[GAP]`이다.

reset 시작 시 Nav2 status 수신 이력이 없고 action server도 없으면 Nav2 미실행으로 판정해
Goal 검사를 생략한다. action server가 있는데 status가 없거나, 이전에 받은 status가
stale이면 reset을 거부한다. status가 fresh하고 활성 Goal이 없으면 cancel 서비스를
호출하지 않으며, 활성 Goal이 있을 때만 전체 취소 후 terminal 상태를 확인한다. Goal
유무와 무관하게 Supervisor는 `/cmd_vel_req`가 0 또는 stale인지 별도로 확인한다.

세부 계약은 `docs/estop_integration_development_direction.md`를 따른다.
