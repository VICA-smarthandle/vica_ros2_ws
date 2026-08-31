"""Diagnostic summaries for the three vica_safety nodes.

이 모듈이 왜 필요한가 — 2026-08-01 Jetson 실기에서 확인한 오탐이다.
`vica_safety`가 `/diagnostics`를 하나도 내지 않으면 aggregator의 Safety 분석기가
항목 0개로 남고, 경로 자체가 Stale이 된다.

```text
/VICA/Safety   level=3   message: "Stale"
   -> robot_health_monitor: safety | DIAG_COMPONENT_STALE
   -> 앱 화면: "진단이 갱신되지 않았습니다"
```

Safety가 `READY_TO_GO`로 정상 동작하는 중에도 관리자에게는 안전 장치가 고장난 것처럼
보였다. 상시 거짓 경보는 진짜 결함을 무시하게 만든다.

**비상정지는 등급이 아니라 래치다.**
버튼이 눌린 것을 ERROR로 올리면 정상 조작이 고장으로 표시된다. 앱이
`refactor(app)!: 등급 축에서 비상 정지를 떼어낸다`로 떼어낸 축을 진단에서 되살리면
안 된다. 그래서 여기서 등급으로 올리는 것은 **노드가 안전 판정을 수행할 수 있는가**
뿐이고, 래치 상태는 key-value로 싣는다.

등급 기준은 이렇게 나눈다.

| 등급 | 뜻 |
| --- | --- |
| ERROR | 안전 판정 자체가 불가능하다. 눈이 멀었다 |
| WARN  | 판정은 하지만 해제의 전제 조건이 빠져 있다 |
| OK    | 정상. 래치가 걸려 있어도 OK다 |

`motor_can_stale`을 ERROR가 아니라 WARN으로 두는 이유는, 그 근본 원인을 이미
motor 컴포넌트가 자기 진단으로 보고하기 때문이다. 같은 원인을 두 곳에서 ERROR로
올리면 관리자가 원인을 두 개로 착각한다. Safety가 말해야 하는 것은 그 결과
"비상정지를 해제할 수 없다"이다.
"""

# diagnostic_msgs/DiagnosticStatus의 등급이다. 값만 베끼고 메시지 타입은 들이지
# 않는다 — 이 모듈은 ROS 없이 시험할 수 있어야 한다.
#
# int가 아니라 bytes인 이유: `DiagnosticStatus.level`은 .msg에서 `byte`로 선언되어
# 있어 rosidl이 길이 1의 bytes를 요구한다. int를 넘기면 발행 순간에
# `AssertionError: The 'level' field must be of type 'bytes'`로 노드가 죽는다.
# 2026-08-01에 실제로 이렇게 죽였다. bytes는 순수 파이썬 타입이므로 ROS 의존이
# 생기지 않으며, test_level_constants_match_ros가 두 값이 갈라지지 않게 고정한다.
OK = b'\x00'
WARN = b'\x01'
ERROR = b'\x02'
STALE = b'\x03'

# aggregator가 Safety로 분류하는 근거다. diagnostic_updater가 만드는 최종 이름은
# `<node_name>: <label>`이므로, 세 노드 모두 이름 안에 `safety:`가 들어간다.
#
#   emergency_stop_node: safety: estop latch
#   safety_supervisor_node: safety: cmd_vel gate
#   app_emergency_node: safety: app bridge
#
# diagnostic_aggregator.yaml의 `contains: ['safety:', 'emergency']`가 이것을 잡고,
# 같은 파일의 `expected`가 세 이름을 그대로 적어 노드 하나가 죽어도 Stale로 드러낸다.
# 이 문자열을 바꾸면 그 yaml도 함께 바꿔야 한다. test_safety_diagnostics가 고정한다.
LABEL_LATCH = 'safety: estop latch'
LABEL_GATE = 'safety: cmd_vel gate'
LABEL_BRIDGE = 'safety: app bridge'


def latch_summary(
    *,
    can_ready: bool,
    physical_fresh: bool,
    motor_can_fresh: bool,
    latch_state: str,
) -> tuple[int, str]:
    """Summarise whether the central latch node can still judge safety.

    `can_ready`는 물리 입력 경로가 열려 있는지다. `can_f1` 모드에서 SocketCAN을
    열지 못했으면 False다. 열지 못한 채로 조용히 OK를 내면, 아무도 감시하지 않는
    비상정지 버튼을 감시하고 있다고 믿게 된다.
    """
    if not can_ready:
        return ERROR, '물리 비상정지 입력을 열지 못했습니다'
    # 부팅 유예 안의 미수신은 '끊김'이 아니라 '아직 안 옴'이다. ERROR로 찍으면
    # 관리자가 없는 고장을 찾는다. 그렇다고 OK도 아니다 - 아직 버튼을 감시하지
    # 못하는 것은 사실이므로 WARN에 둔다. 유예를 넘기면 latch_state가 FAULT로
    # 바뀌므로 아래 ERROR 규칙이 그대로 받는다.
    if latch_state == 'WAITING_INPUT':
        return WARN, '부팅 후 첫 입력 대기 중입니다'
    if not physical_fresh:
        return ERROR, '물리 비상정지 입력이 끊겼습니다'
    if not motor_can_fresh:
        return WARN, '모터 CAN 신호가 없어 비상정지를 해제할 수 없습니다'
    return OK, f'정상 (래치 {latch_state})'


def gate_summary(*, estop_fresh: bool, gate_state: str) -> tuple[int, str]:
    """Summarise whether the drive gate still trusts the central latch.

    입력이 끊겨도 게이트는 0을 내보내므로 로봇은 안전하다. 그러나 그것은
    "정상 정지"가 아니라 "상위 노드가 죽었다"이므로 ERROR로 올린다.
    """
    if not estop_fresh:
        return ERROR, '중앙 비상정지 신호가 끊겨 주행 출력을 차단했습니다'
    return OK, f'정상 (게이트 {gate_state})'


def bridge_summary(
    *,
    emergency_fresh: bool,
    safety_state_fresh: bool,
) -> tuple[int, str]:
    """Summarise whether the app-facing bridge can still report and reset.

    이 노드가 앱의 유일한 창구다. 여기가 조용히 죽으면 관리자는 리셋을 요청할
    수단을 잃는데 화면에는 아무 표시가 없다.
    """
    if not emergency_fresh:
        return ERROR, '중앙 비상정지 노드의 신호를 받지 못했습니다'
    if not safety_state_fresh:
        return WARN, 'Safety Supervisor 상태를 받지 못했습니다'
    return OK, '정상'


def sources_text(active_sources) -> str:
    """Render active latch sources for a diagnostic value.

    빈 튜플을 빈 문자열로 두면 화면에서 "값이 없다"와 "원인이 없다"가 구별되지
    않는다. `freshness._age_sec`가 None을 유지하는 것과 같은 이유다.
    """
    return ','.join(active_sources) if active_sources else 'none'
