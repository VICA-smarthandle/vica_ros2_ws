"""Tests for the mapping start gate.

이 판정이 느슨하면 회차를 잃는다. 2026-08-11 20:07 에 실제로 두 회차를 잃었고,
그 사고를 코드가 막게 하는 것이 이 파일의 목적이다. 그래서 "시작되는가"보다
**"언제 막히는가"**를 더 촘촘히 검증한다.
"""

import signal

from vica_cartographer.mapping_session import (
    blocking_reason,
    duplicated_names,
    is_stack_up,
    MappingState,
    missing_prerequisites,
    map_meta_document,
    normalise_map_name,
    plan_map_save,
    save_label,
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


def test_motor_alone_does_not_block_start():
    """Motor 는 두 스택이 공유한다. 떠 있다고 매핑을 막을 이유가 없다."""
    assert blocking_reason(
        MappingState.IDLE, ['/mdrobot_can_keyboard_knob_node']
    ) is None


# -- 필수 노드 검사 (2026-09-01) ------------------------------------------
#
# 종전에는 handle_start 의 주석만 "검사했다"고 말하고 코드는 검사하지 않았다.
# 모터 없이 시작하면 /wheel/odom 이 영영 안 나와 40초 뒤 STARTING 시한
# 초과로만 죽었다 — 회차 무효. 이제 시작 전에 사람이 할 조치를 알려주며 막는다.

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
    assert '모터' in reason          # 노드 이름이 아니라 사람 말로
    assert '⑤' in reason             # 어디서 띄우는지까지


def test_all_prerequisites_up_allows_start():
    assert blocking_reason(MappingState.IDLE, ALL_PREREQS_UP, PREREQS) is None


def test_prerequisites_default_empty_keeps_old_behaviour():
    """인자 required 없이 부르면 종전과 같다 — 상태 표시용 호출이 안 흔들린다."""
    assert blocking_reason(MappingState.IDLE, ['/rosapi']) is None


def test_nav2_check_still_wins_over_prerequisites():
    """위험한 것(중복 /odom)이 헛수고(필수 노드)보다 먼저 보여야 한다."""
    reason = blocking_reason(MappingState.IDLE, ['/amcl'], PREREQS)
    assert 'Nav2' in reason


# -- 정지 사다리 (2026-09-01) ---------------------------------------------
#
# 종전 _terminate_process 는 콜백 안에서 최대 24초 기다려 rosbridge 까지
# 막았다. 이제 타이머가 매 tick 물어보고, 이 객체는 기다리지 않는다.

def test_stop_starts_with_sigint():
    stop = StopEscalation(grace_sec=8.0, now=100.0)
    assert stop.first_signal == signal.SIGINT


def test_no_escalation_before_grace():
    stop = StopEscalation(grace_sec=8.0, now=100.0)
    assert stop.escalate_signal(107.9) is None


def test_escalates_to_sigterm_then_sigkill():
    stop = StopEscalation(grace_sec=8.0, now=100.0)
    assert stop.escalate_signal(108.0) == signal.SIGTERM
    assert stop.escalate_signal(115.9) is None      # 새 유예가 다시 돈다
    assert stop.escalate_signal(116.0) == signal.SIGKILL


def test_nothing_above_sigkill():
    """SIGKILL 은 무시될 수 없다 — 더 올릴 데가 없고, poll 이 시체를 거둔다."""
    stop = StopEscalation(grace_sec=8.0, now=100.0)
    stop.escalate_signal(108.0)
    stop.escalate_signal(116.0)
    assert stop.escalate_signal(999.0) is None


# ---------------------------------------------------------------------------
# 표시 이름(한글) / 자동 id — 2026-09-04
# ---------------------------------------------------------------------------


def test_ascii_name_keeps_the_old_id_rule():
    assert plan_map_save('lobby', '0904', '151230') == ('lobby_0904', 'lobby_0904', '')


def test_hangul_name_becomes_display_name_with_generated_id():
    map_id, display, error = plan_map_save('병원 2층', '0904', '151230')
    assert (map_id, display, error) == ('map_0904_151230', '병원 2층', '')


def test_display_name_is_nfc_normalised():
    # macOS 는 '병'을 자모로 풀어(NFD) 보낼 수 있다. 눈엔 같은데 바이트가 다르면
    # 터미네이터에서 이름으로 찾을 때 못 찾는다.
    import unicodedata
    decomposed = unicodedata.normalize('NFD', '병원')
    assert decomposed != '병원'
    assert plan_map_save(decomposed, '0904', '151230')[1] == '병원'


def test_display_name_rejects_paths_and_length():
    assert plan_map_save('병원/2층', '0904', '151230')[0] is None
    assert plan_map_save('가' * 41, '0904', '151230')[0] is None
    assert plan_map_save('가' * 40, '0904', '151230')[0] == 'map_0904_151230'


def test_save_label_shows_id_only_when_names_differ():
    assert save_label('lobby_0904', 'lobby_0904') == 'lobby_0904'
    assert save_label('map_0904_151230', '병원 2층') == '병원 2층 (map_0904_151230)'


def test_meta_document_carries_both_names():
    assert map_meta_document('map_0904_151230', '병원 2층', '2026-09-04T15:12:30') == {
        'map_id': 'map_0904_151230',
        'display_name': '병원 2층',
        'saved_at': '2026-09-04T15:12:30',
    }
