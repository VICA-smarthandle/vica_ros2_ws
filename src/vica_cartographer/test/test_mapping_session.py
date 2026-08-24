"""Tests for the mapping start gate.

이 판정이 느슨하면 회차를 잃는다. 2026-08-11 20:07 에 실제로 두 회차를 잃었고,
그 사고를 코드가 막게 하는 것이 이 파일의 목적이다. 그래서 "시작되는가"보다
**"언제 막히는가"**를 더 촘촘히 검증한다.
"""

from vica_cartographer.mapping_session import (
    blocking_reason,
    duplicated_names,
    is_motor_up,
    is_stack_up,
    MappingState,
    missing_prerequisites,
    normalise_map_name,
    running_stacks,
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


# ---------------------------------------------------------------------------
# 지도 이름 — "제목은 사람이, 날짜는 자동"
# ---------------------------------------------------------------------------


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


def test_motor_detection_ignores_namespace():
    """Motor 가 이미 떠 있으면 launch 에 start_motor:=false 를 넘겨야 한다."""
    assert is_motor_up(['/mdrobot_can_keyboard_knob_node']) is True
    assert is_motor_up(['/robot1/mdrobot_can_keyboard_knob_node']) is True
    assert is_motor_up(['/cartographer_node']) is False


def test_motor_alone_does_not_block_start():
    """Motor 는 두 스택이 공유한다. 떠 있다고 매핑을 막을 이유가 없다."""
    assert blocking_reason(
        MappingState.IDLE, ['/mdrobot_can_keyboard_knob_node']
    ) is None
