"""Unit tests for /diagnostics_agg name parsing.

계층 name(`/VICA/Hardware/Motor`)과 평면 name(`mdrobot: CAN link`)을 모두 다뤄야 한다.
diagnostics_topic 파라미터로 aggregator를 우회해 단독 디버깅할 수 있어야 하기 때문이다.
"""

from vica_system_monitor.agg_parser import (
    DIAG_ERROR,
    DIAG_OK,
    DIAG_STALE,
    DIAG_WARN,
    DiagItem,
    localize_message,
    normalize_level,
    parse_name,
    to_fault_code,
)


# ---------------------------------------------------------------------------
# 계층 name 파싱
# ---------------------------------------------------------------------------


def test_hierarchical_name_maps_to_known_component():
    """Aggregator 계층 경로에서 컴포넌트를 뽑는다."""
    assert parse_name('/VICA/Hardware/Motor') == 'motor'
    assert parse_name('/VICA/Hardware/LiDAR') == 'lidar'
    assert parse_name('/VICA/Safety') == 'safety'


def test_hierarchical_leaf_item_maps_to_its_group():
    """말단 항목도 그룹 컴포넌트로 매핑된다."""
    assert parse_name('/VICA/Hardware/Motor/CAN link') == 'motor'


def test_hierarchical_name_is_case_insensitive():
    """대소문자 표기가 달라도 같은 컴포넌트다."""
    assert parse_name('/VICA/hardware/MOTOR') == 'motor'


def test_perception_aliases():
    """nvblox·카메라는 perception으로 모은다."""
    assert parse_name('/VICA/Hardware/Perception') == 'perception'
    assert parse_name('/VICA/Hardware/nvblox') == 'perception'
    assert parse_name('/VICA/Hardware/Camera') == 'perception'


# ---------------------------------------------------------------------------
# 평면 name 파싱 (aggregator 우회)
# ---------------------------------------------------------------------------


def test_flat_name_from_motor_node():
    """현재 실제로 발행되는 평면 name을 매핑한다."""
    assert parse_name('mdrobot_can_keyboard_knob_node: CAN link') == 'motor'


def test_flat_name_from_adapter_probe():
    """어댑터가 내는 토픽 진단 name을 매핑한다."""
    assert parse_name('sensor_diagnostics: /scan frequency') == 'lidar'
    assert (
        parse_name('sensor_diagnostics: /nvblox_node/static_map_slice frequency')
        == 'perception'
    )
    assert (
        parse_name('sensor_diagnostics: /camera/camera/depth/camera_info frequency')
        == 'perception'
    )
    assert parse_name('sensor_diagnostics: /odom frequency') == 'localization'


def test_unknown_name_falls_back_to_monitor():
    """모르는 name은 버리지 않고 monitor로 모아 표시한다.

    버리면 새 노드가 진단을 추가했을 때 조용히 사라진다. 표시하는 편이 안전하다.
    """
    assert parse_name('something_totally_new: whatever') == 'monitor'
    assert parse_name('') == 'monitor'


# ---------------------------------------------------------------------------
# level 정규화
# ---------------------------------------------------------------------------


def test_normalize_level_accepts_int_bytes_str():
    """Diagnostic level이 어떤 형태로 와도 숫자로 만든다.

    vica_status_app_node.py의 _diagnostic_level이 쓰던 방어와 같은 이유다.
    """
    assert normalize_level(2) == 2
    assert normalize_level(b'\x02') == 2
    assert normalize_level('\x02') == 2


def test_normalize_level_handles_empty_and_none():
    """빈 값은 OK로 본다."""
    assert normalize_level(b'') == DIAG_OK
    assert normalize_level('') == DIAG_OK
    assert normalize_level(None) == DIAG_OK


def test_stale_level_is_recognized():
    """aggregator가 판정한 Stale(3)을 알아본다."""
    assert normalize_level(3) == DIAG_STALE


# ---------------------------------------------------------------------------
# fault code 변환
# ---------------------------------------------------------------------------


def test_error_level_maps_to_error_code():
    """ERROR는 DIAG_COMPONENT_ERROR로 전달한다."""
    assert to_fault_code(DIAG_ERROR) == 'DIAG_COMPONENT_ERROR'


def test_warn_level_maps_to_warn_code():
    """WARN은 등급을 낮춰 전달한다."""
    assert to_fault_code(DIAG_WARN) == 'DIAG_COMPONENT_WARN'


def test_stale_level_maps_to_stale_code():
    """aggregator의 Stale은 별도 코드로 구분한다.

    "오류가 났다"와 "소식이 없다"는 정비하는 사람에게 다른 정보다.
    """
    assert to_fault_code(DIAG_STALE) == 'DIAG_COMPONENT_STALE'


def test_ok_level_has_no_fault_code():
    """OK는 결함이 아니다."""
    assert to_fault_code(DIAG_OK) == ''


# ---------------------------------------------------------------------------
# DiagItem 도우미
# ---------------------------------------------------------------------------


def test_diag_item_reports_fault_only_above_warn():
    """WARN 이상만 결함으로 본다."""
    assert not DiagItem('x', DIAG_OK, '').is_fault
    assert DiagItem('x', DIAG_WARN, '').is_fault
    assert DiagItem('x', DIAG_ERROR, '').is_fault
    assert DiagItem('x', DIAG_STALE, '').is_fault


def test_diag_item_component_uses_parse_name():
    """항목이 스스로 컴포넌트를 알려준다."""
    item = DiagItem('/VICA/Hardware/Motor/CAN link', DIAG_ERROR, 'CAN link FAILED')
    assert item.component == 'motor'
    assert item.fault_code == 'DIAG_COMPONENT_ERROR'


# ---------------------------------------------------------------------------
# aggregator 영문 요약어 번역
#
# 2026-07-31 실행 검증에서 관리자 화면에 `detail: Missing`, `Error`, `Stale`이 그대로
# 떴다. diagnostic_aggregator가 그룹·미보고 항목에 붙이는 영문 요약어다. `?초`
# 자리표시자가 샜던 것과 같은 종류이고, 정보량도 없다.
# ---------------------------------------------------------------------------


def test_missing_is_translated():
    """항목이 아예 보고되지 않은 경우를 한국어로 알린다."""
    assert localize_message('Missing') == '진단 항목이 보고되지 않았습니다.'


def test_stale_is_translated():
    """오래된 진단과 오류를 구분해 알린다."""
    assert localize_message('Stale') == '진단이 갱신되지 않았습니다.'


def test_error_and_warning_are_translated():
    """그룹 요약어도 한국어로 바꾼다."""
    assert localize_message('Error') == '오류를 보고했습니다.'
    assert localize_message('Warning') == '경고를 보고했습니다.'


def test_no_events_recorded_is_translated():
    """diagnostic_updater가 아직 한 건도 못 받았을 때의 문구다."""
    assert localize_message('No events recorded.') == (
        '아직 한 건도 수신하지 못했습니다.'
    )


def test_translation_is_case_and_space_insensitive():
    """대소문자·앞뒤 공백이 달라도 번역한다."""
    assert localize_message('  missing ') == '진단 항목이 보고되지 않았습니다.'
    assert localize_message('STALE') == '진단이 갱신되지 않았습니다.'


def test_korean_message_from_our_node_passes_through():
    """우리 노드가 쓴 문구는 그대로 둔다. 정본은 로봇 쪽에 있다."""
    ours = '프로세스를 찾지 못했습니다 (미구성 또는 미실행)'
    assert localize_message(ours) == ours


def test_informative_english_from_a_third_party_passes_through():
    """모르는 문구를 버리지 않는다. 버리면 정비 단서가 사라진다."""
    other = 'CAN link FAILED: no frame for 0.82s'
    assert localize_message(other) == other


def test_empty_message_becomes_empty():
    """빈 문구는 만들어내지 않는다."""
    assert localize_message('') == ''
    assert localize_message(None) == ''


def test_diag_item_detail_is_localized():
    """DiagItem이 표시용 문구를 직접 제공한다."""
    item = DiagItem('/VICA/Hardware/Motor', DIAG_STALE, 'Missing')
    assert item.detail == '진단 항목이 보고되지 않았습니다.'


def test_no_diag_item_detail_is_a_bare_aggregator_token():
    """aggregator가 만드는 영문 요약어가 하나도 사용자 문구로 새지 않는다."""
    leaked = ('missing', 'stale', 'error', 'warning', 'ok', 'no events recorded.')
    for raw in ('Missing', 'Stale', 'Error', 'Warning', 'OK', 'No events recorded.'):
        detail = DiagItem('/VICA/Hardware/Motor', DIAG_ERROR, raw).detail
        assert detail.strip().lower() not in leaked, detail
