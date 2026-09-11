"""Tests for the mapping start gate."""

import signal

from vica_cartographer.mapping_session import (
    blocking_reason,
    duplicated_names,
    is_stack_up,
    MappingState,
    missing_prerequisites,
    normalise_map_name,
    running_stacks,
    StopEscalation,
)


def test_empty_graph_allows_start():
    assert blocking_reason(MappingState.IDLE, ['/rosapi']) is None


def test_nav2_blocks_start():
    """Nav2 와 SLAM 은 둘 다 wheel_ekf 를 include 해서 /odom 이 이중 발행된다."""
    reason = blocking_reason(MappingState.IDLE, ['/amcl'])
    assert reason is not None
    assert 'Nav2' in reason


def test_any_single_nav2_node_is_enough_to_block():
    """부분 기동도 '떴다'로 본다. 애매하면 막는 쪽이 싸다."""
    for name in ('/bt_navigator', '/controller_server', '/planner_server'):
        assert blocking_reason(MappingState.IDLE, [name]) is not None


def test_mapping_already_running_blocks_start():
    reason = blocking_reason(MappingState.IDLE, ['/cartographer_node'])
    assert reason is not None
    assert '이미 실행 중' in reason


def test_duplicate_node_blocks_before_anything_else():
    """이미 사고가 난 상태다. 한 벌을 더 얹으면 안 된다."""
    reason = blocking_reason(
        MappingState.IDLE, ['/ekf_filter_node', '/ekf_filter_node']
    )
    assert reason is not None
    assert 'ekf_filter_node' in reason


def test_shared_nodes_alone_do_not_mean_a_stack_is_up():
    """EKF·encoder 는 두 스택이 함께 쓰므로 어느 쪽인지 가르지 못한다."""
    stacks = running_stacks(['/ekf_filter_node', '/encoder_feedback'])
    assert stacks == {'nav2': False, 'mapping': False}
    assert blocking_reason(MappingState.IDLE, ['/ekf_filter_node']) is None


def test_namespace_is_ignored_when_matching():
    assert running_stacks(['/robot1/amcl'])['nav2'] is True


def test_non_idle_state_blocks_start():
    """시작 버튼을 두 번 누르는 것을 코드가 막는다."""
    for state in (
        MappingState.STARTING,
        MappingState.MAPPING,
        MappingState.SAVING,
        MappingState.STOPPING,
        MappingState.ERROR,
    ):
        assert blocking_reason(state, []) is not None


def test_duplicated_names_lists_only_repeats():
    assert duplicated_names(['/a', '/b', '/a', '/c', '/c', '/c']) == ['a', 'c']
    assert duplicated_names(['/a', '/b']) == []


def test_missing_prerequisites_reports_what_is_not_up():
    """d455·imu 는 사람이 띄운다. 앱은 떠 있는지 확인만 한다."""
    missing = missing_prerequisites(
        ['/camera/camera', '/safety_supervisor_node'],
        ['camera/camera', 'imu_base_link_adapter', 'safety_supervisor_node'],
    )
    assert missing == ['imu_base_link_adapter']


def test_is_stack_up_waits_for_cartographer():
    assert is_stack_up(['/ekf_filter_node']) is False
    assert is_stack_up(['/ekf_filter_node', '/cartographer_node']) is True


def test_date_is_appended_automatically():
    assert normalise_map_name('lobby', '0821') == ('lobby_0821', '')


def test_date_is_not_appended_twice():
    assert normalise_map_name('lobby_0821', '0821') == ('lobby_0821', '')


def test_hangul_and_spaces_are_rejected():
    """Map yaml 과 앱의 HTTP 경로가 이 이름을 그대로 쓴다."""
    for bad in ('로비', 'my map', 'lobby.png', 'a/b'):
        name, error = normalise_map_name(bad, '0821')
        assert name is None, bad
        assert '영문' in error


def test_empty_name_is_rejected():
    name, error = normalise_map_name('   ', '0821')
    assert name is None
    assert '입력' in error


def test_too_long_name_is_rejected():
    name, error = normalise_map_name('a' * 60, '0821')
    assert name is None
    assert '깁니다' in error


def test_motor_alone_does_not_block_start():
    """Motor 는 두 스택이 공유한다. 떠 있다고 매핑을 막을 이유가 없다."""
    assert blocking_reason(
        MappingState.IDLE, ['/mdrobot_can_keyboard_knob_node']
    ) is None


PREREQS = [
    'camera/camera',
    'imu_base_link_adapter',
    'mdrobot_can_keyboard_knob_node',
]

ALL_PREREQS_UP = [
    '/camera/camera',
    '/imu_base_link_adapter',
    '/mdrobot_can_keyboard_knob_node',
]


def test_missing_motor_blocks_start_with_actionable_message():
    names = ['/camera/camera', '/imu_base_link_adapter']
    reason = blocking_reason(MappingState.IDLE, names, PREREQS)
    assert reason is not None
    assert '모터' in reason
    assert '⑤' in reason


def test_all_prerequisites_up_allows_start():
    assert blocking_reason(MappingState.IDLE, ALL_PREREQS_UP, PREREQS) is None


def test_prerequisites_default_empty_keeps_old_behaviour():
    """인자 required 없이 부르면 종전과 같다 — 상태 표시용 호출이 안 흔들린다."""
    assert blocking_reason(MappingState.IDLE, ['/rosapi']) is None


def test_nav2_check_still_wins_over_prerequisites():
    """위험한 것(중복 /odom)이 헛수고(필수 노드)보다 먼저 보여야 한다."""
    reason = blocking_reason(MappingState.IDLE, ['/amcl'], PREREQS)
    assert 'Nav2' in reason


def test_stop_starts_with_sigint():
    stop = StopEscalation(grace_sec=8.0, now=100.0)
    assert stop.first_signal == signal.SIGINT


def test_no_escalation_before_grace():
    stop = StopEscalation(grace_sec=8.0, now=100.0)
    assert stop.escalate_signal(107.9) is None


def test_escalates_to_sigterm_then_sigkill():
    stop = StopEscalation(grace_sec=8.0, now=100.0)
    assert stop.escalate_signal(108.0) == signal.SIGTERM
    assert stop.escalate_signal(115.9) is None
    assert stop.escalate_signal(116.0) == signal.SIGKILL


def test_nothing_above_sigkill():
    """SIGKILL 은 무시될 수 없다 — 더 올릴 데가 없고, poll 이 시체를 거둔다."""
    stop = StopEscalation(grace_sec=8.0, now=100.0)
    stop.escalate_signal(108.0)
    stop.escalate_signal(116.0)
    assert stop.escalate_signal(999.0) is None
