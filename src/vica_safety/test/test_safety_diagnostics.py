"""Pin the safety diagnostic grading rules and the names the aggregator expects.

이 시험이 지키는 것은 두 가지다.

1. **비상정지가 등급으로 새어나가지 않는다.** 버튼이 눌린 상태에서 OK가 나와야 한다.
   여기가 무너지면 정상 조작이 관리자 화면에 고장으로 뜬다.
2. **label과 aggregator 설정이 갈라지지 않는다.** 이름이 어긋나면 그 항목은 영구
   Stale로 남아, 고치려고 만든 진단이 새 오탐이 된다.
"""

from pathlib import Path

import pytest
import yaml

from vica_safety.diagnostics import (
    ERROR,
    LABEL_BRIDGE,
    LABEL_GATE,
    LABEL_LATCH,
    OK,
    STALE,
    WARN,
    bridge_summary,
    gate_summary,
    latch_summary,
    sources_text,
)


HEALTHY_LATCH = {
    'can_ready': True,
    'physical_fresh': True,
    'motor_can_fresh': True,
}


# ---------------------------------------------------------------------------
# 등급 규칙
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    'latch_state',
    ['CLEARED', 'ESTOP_ACTIVE', 'ESTOP_RELEASED_WAIT_RESET'],
)
def test_latch_state_never_raises_the_level(latch_state):
    """래치 상태는 등급을 올리지 않는다. 눌린 비상정지는 결함이 아니다.

    앱이 `등급 축에서 비상 정지를 떼어낸다`로 분리한 축을 진단에서 되살리면,
    관리자가 버튼을 누를 때마다 화면에 고장이 뜬다.
    """
    level, _ = latch_summary(**HEALTHY_LATCH, latch_state=latch_state)
    assert level == OK


def test_unopened_can_input_is_an_error():
    """물리 입력을 열지 못한 채 OK를 내면 감시하지 않는 버튼을 감시한다고 믿게 된다."""
    level, message = latch_summary(
        can_ready=False,
        physical_fresh=False,
        motor_can_fresh=True,
        latch_state='FAULT',
    )
    assert level == ERROR
    assert '열지 못했' in message


def test_stale_physical_input_is_an_error():
    """물리 버튼이 끊기면 안전 판정 자체가 불가능하다."""
    level, message = latch_summary(
        can_ready=True,
        physical_fresh=False,
        motor_can_fresh=True,
        latch_state='FAULT',
    )
    assert level == ERROR
    assert '물리' in message


def test_stale_motor_can_is_a_warning_not_an_error():
    """모터 CAN 두절의 근본 원인은 motor 컴포넌트가 이미 ERROR로 보고한다.

    같은 원인을 Safety도 ERROR로 올리면 관리자가 원인을 두 개로 착각한다.
    Safety가 말해야 하는 것은 그 결과인 "해제 불가"다.
    """
    level, message = latch_summary(
        can_ready=True,
        physical_fresh=True,
        motor_can_fresh=False,
        latch_state='FAULT',
    )
    assert level == WARN
    assert '해제' in message


def test_unopened_can_outranks_stale_motor_can():
    """둘 다 나쁘면 더 근본적인 쪽을 보고한다. 눈이 먼 것이 먼저다."""
    level, _ = latch_summary(
        can_ready=False,
        physical_fresh=False,
        motor_can_fresh=False,
        latch_state='FAULT',
    )
    assert level == ERROR


def test_gate_blocked_by_estop_is_not_a_fault():
    """게이트가 막고 있는 것은 정상 동작이다. 등급을 올릴 이유가 없다."""
    level, _ = gate_summary(estop_fresh=True, gate_state='ESTOP_ACTIVE')
    assert level == OK


def test_gate_without_estop_input_is_an_error():
    """입력이 끊기면 로봇은 멈추지만, 그것은 정상 정지가 아니라 상위 노드의 사망이다."""
    level, message = gate_summary(estop_fresh=False, gate_state='FAULT')
    assert level == ERROR
    assert '끊' in message


def test_bridge_without_central_signal_is_an_error():
    """중앙 노드가 죽으면 관리자는 리셋 수단을 잃는다."""
    level, _ = bridge_summary(emergency_fresh=False, safety_state_fresh=True)
    assert level == ERROR


def test_bridge_without_supervisor_state_is_a_warning():
    """Supervisor 상태만 없으면 리셋 경로는 살아 있다. 표시만 낡는다."""
    level, _ = bridge_summary(emergency_fresh=True, safety_state_fresh=False)
    assert level == WARN


def test_bridge_healthy_is_ok():
    level, _ = bridge_summary(emergency_fresh=True, safety_state_fresh=True)
    assert level == OK


def test_sources_text_distinguishes_empty_from_missing():
    """빈 문자열을 두면 "값 없음"과 "원인 없음"이 화면에서 구별되지 않는다."""
    assert sources_text(()) == 'none'
    assert sources_text(('motor_can_stale',)) == 'motor_can_stale'
    assert sources_text(('app', 'voice')) == 'app,voice'


# ---------------------------------------------------------------------------
# aggregator 설정과의 계약
# ---------------------------------------------------------------------------

def find_aggregator_yaml() -> Path:
    """Locate vica_system_monitor's aggregator config inside the same workspace.

    두 패키지는 같은 저장소에 있으므로 소스 트리에서 찾는다. 찾지 못하면 시험을
    건너뛴다 — vica_safety만 따로 빌드하는 환경을 실패로 만들지 않기 위해서다.
    """
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = (
            parent
            / 'vica_system_monitor'
            / 'config'
            / 'diagnostic_aggregator.yaml'
        )
        if candidate.is_file():
            return candidate
    return None


def test_level_constants_match_ros():
    """등급 값이 diagnostic_msgs와 같아야 한다. 타입까지 같아야 한다.

    `DiagnosticStatus.level`은 .msg에서 `byte`라 길이 1의 bytes만 받는다. int를
    넘기면 진단을 발행하는 순간 AssertionError로 노드가 죽는다. 2026-08-01에
    실제로 그렇게 죽여서 비상정지 해제가 불가능해졌다. 이 시험이 재발을 막는다.
    """
    try:
        from diagnostic_msgs.msg import DiagnosticStatus
    except ImportError:
        pytest.skip('ROS 환경이 아닙니다')

    assert OK == DiagnosticStatus.OK
    assert WARN == DiagnosticStatus.WARN
    assert ERROR == DiagnosticStatus.ERROR
    assert STALE == DiagnosticStatus.STALE


def test_level_constants_are_single_bytes():
    """ROS 없이도 타입 실수를 잡는다. 이것이 위 시험의 skip을 메운다."""
    for level in (OK, WARN, ERROR, STALE):
        assert isinstance(level, bytes), level
        assert len(level) == 1, level


def test_labels_start_with_the_component_prefix():
    """aggregator의 `contains: ['safety:', ...]`가 세 항목을 모두 잡는다.

    diagnostic_updater가 만드는 최종 이름은 `<node_name>: <label>`이다. label이
    `safety: `로 시작해야 노드 이름과 무관하게 Safety로 분류된다.
    """
    for label in (LABEL_LATCH, LABEL_GATE, LABEL_BRIDGE):
        assert label.startswith('safety: '), label


def test_aggregator_expected_matches_the_labels():
    """설정의 expected와 코드의 label이 어긋나면 그 항목이 영구 Stale이 된다."""
    path = find_aggregator_yaml()
    if path is None:
        pytest.skip('vica_system_monitor 설정을 찾지 못했습니다')

    params = yaml.safe_load(path.read_text(encoding='utf-8'))
    safety = params['aggregator_node']['ros__parameters']['analyzers']['safety']

    expected = set(safety['expected'])
    ours = {
        f'emergency_stop_node: {LABEL_LATCH}',
        f'safety_supervisor_node: {LABEL_GATE}',
        f'app_emergency_node: {LABEL_BRIDGE}',
    }
    assert expected == ours, f'차이: {expected ^ ours}'


def test_app_analyzer_does_not_capture_the_safety_supervisor():
    """'supervisor'는 `safety_supervisor_node:`까지 잡아 항목이 App에도 걸린다.

    aggregator는 조건에 맞는 모든 analyzer에 항목을 넣으므로, 안전 항목이 앱 고장으로
    보이게 된다. 그래서 'vica_supervisor'로 좁혔다.
    """
    path = find_aggregator_yaml()
    if path is None:
        pytest.skip('vica_system_monitor 설정을 찾지 못했습니다')

    params = yaml.safe_load(path.read_text(encoding='utf-8'))
    app = params['aggregator_node']['ros__parameters']['analyzers']['app']

    node_name = 'safety_supervisor_node: ' + LABEL_GATE
    for pattern in app['contains']:
        assert pattern not in node_name, (
            f"app analyzer의 '{pattern}'가 '{node_name}'를 잡습니다"
        )


# ---------------------------------------------------------------------------
# 부팅 유예 (WAITING_INPUT)
# ---------------------------------------------------------------------------
#
# 부팅 직후의 미수신은 '아직 안 옴'이지 '끊김'이 아니다. 여기서 ERROR를 내면
# WAITING_INPUT을 도입한 취지가 사라진다 - 관리자가 없는 고장을 찾게 된다.
# 그렇다고 OK도 아니다. 아직 버튼을 감시하지 못하는 것은 사실이므로 WARN이다.


def test_boot_grace_is_warn_not_error():
    level, message = latch_summary(
        can_ready=True,
        physical_fresh=False,
        motor_can_fresh=False,
        latch_state='WAITING_INPUT',
    )

    assert level == WARN
    assert '대기' in message


def test_boot_grace_does_not_mask_a_closed_can_path():
    # CAN을 아예 열지 못한 것은 유예로 봐줄 일이 아니다. 열리지 않으면 첫 신호도
    # 영원히 오지 않는다.
    level, _ = latch_summary(
        can_ready=False,
        physical_fresh=False,
        motor_can_fresh=False,
        latch_state='WAITING_INPUT',
    )

    assert level == ERROR


def test_stale_after_grace_is_still_error():
    # 유예를 넘긴 뒤에는 종전대로 ERROR다. 회귀 방지.
    level, _ = latch_summary(
        can_ready=True,
        physical_fresh=False,
        motor_can_fresh=True,
        latch_state='FAULT',
    )

    assert level == ERROR
