"""Unit tests for probe specification parsing and validation.

ROS 파라미터에서 읽은 평범한 값(dict)만 다룬다. 설정 오타로 감시가 조용히 빠지는 것을
막는 것이 이 모듈의 목적이다.
"""

from vica_system_monitor.probe_config import (
    classify_zero_message,
    parse_process_probe,
    parse_topic_probe,
    QOS_DEFAULT,
    QOS_SENSOR_DATA,
    validate_component,
    validate_fault_code,
    ZERO_NO_PUBLISHER,
    ZERO_QOS_SUSPECTED,
)


def topic_values(**overrides):
    """Build a valid topic probe value map with optional overrides."""
    values = {
        'component': 'lidar',
        'topic': '/scan',
        'msg_type': 'sensor_msgs/msg/LaserScan',
        'qos': 'sensor_data',
        'min_hz': 8.0,
        'max_hz': 12.0,
        'fault_code': 'LIDAR_SCAN_STALE',
        'optional': False,
    }
    values.update(overrides)
    return values


def process_values(**overrides):
    """Build a valid process probe value map with optional overrides."""
    values = {
        'component': 'localization',
        'cmdline_pattern': 'imu_base_link_adapter',
        'warn_percent': 60.0,
    }
    values.update(overrides)
    return values


# ---------------------------------------------------------------------------
# topic 프로브 파싱
# ---------------------------------------------------------------------------


def test_parses_valid_topic_probe():
    """정상 설정을 그대로 읽는다."""
    spec, problems = parse_topic_probe('lidar_scan', topic_values())

    assert problems == []
    assert spec is not None
    assert spec.name == 'lidar_scan'
    assert spec.component == 'lidar'
    assert spec.topic == '/scan'
    assert spec.min_hz == 8.0
    assert spec.optional is False


def test_qos_string_maps_to_constant():
    """Map the qos string onto its constant."""
    sensor, _ = parse_topic_probe('a', topic_values(qos='sensor_data'))
    default, _ = parse_topic_probe('b', topic_values(qos='default'))

    assert sensor.qos == QOS_SENSOR_DATA
    assert default.qos == QOS_DEFAULT


def test_unknown_qos_is_rejected():
    """모르는 qos 값을 조용히 기본값으로 바꾸지 않는다.

    QoS 비호환은 메시지를 하나도 받지 못하게 만들고, 감시 도구가 스스로 영구 오탐한다.
    설정 오타를 기본값으로 흡수하면 그 오탐의 원인을 찾을 수 없다.
    """
    spec, problems = parse_topic_probe('a', topic_values(qos='reliable'))

    assert spec is None
    assert any('qos' in p for p in problems)


def test_unknown_component_is_rejected():
    """fault_catalog에 없는 컴포넌트를 거부한다."""
    spec, problems = parse_topic_probe('a', topic_values(component='lidarr'))

    assert spec is None
    assert any('component' in p for p in problems)


def test_unknown_fault_code_is_rejected():
    """카탈로그에 없는 fault code를 거부한다.

    오타가 있으면 앱에 "알 수 없는 진단 코드" 문구가 뜬다. 기동 시 잡는 게 낫다.
    """
    spec, problems = parse_topic_probe(
        'a', topic_values(fault_code='LIDAR_SCAN_STAL')
    )

    assert spec is None
    assert any('fault_code' in p for p in problems)


def test_empty_topic_is_rejected():
    """토픽 이름이 비어 있으면 구독할 대상이 없다."""
    spec, problems = parse_topic_probe('a', topic_values(topic=''))

    assert spec is None
    assert any('topic' in p for p in problems)


def test_empty_msg_type_is_rejected():
    """타입이 없으면 구독을 만들 수 없다."""
    spec, problems = parse_topic_probe('a', topic_values(msg_type=''))

    assert spec is None
    assert any('msg_type' in p for p in problems)


def test_min_hz_above_max_hz_is_rejected():
    """범위가 뒤집혀 있으면 FrequencyStatus가 항상 실패한다."""
    spec, problems = parse_topic_probe(
        'a', topic_values(min_hz=12.0, max_hz=8.0)
    )

    assert spec is None
    assert any('hz' in p for p in problems)


def test_zero_min_hz_is_rejected():
    """0 Hz를 기대하면 미수신을 감지할 수 없다."""
    spec, problems = parse_topic_probe('a', topic_values(min_hz=0.0))

    assert spec is None
    assert any('hz' in p for p in problems)


def test_missing_optional_defaults_to_false():
    """optional을 생략하면 필수로 본다. 조용히 빠지는 것보다 안전하다."""
    values = topic_values()
    del values['optional']
    spec, problems = parse_topic_probe('a', values)

    assert problems == []
    assert spec.optional is False


def test_all_problems_are_reported_at_once():
    """오류를 하나만 알려주고 멈추지 않는다. 한 번에 다 고칠 수 있게 한다."""
    spec, problems = parse_topic_probe(
        'a',
        topic_values(component='nope', qos='nope', fault_code='NOPE'),
    )

    assert spec is None
    assert len(problems) >= 3


# ---------------------------------------------------------------------------
# process 프로브 파싱
# ---------------------------------------------------------------------------


def test_parses_valid_process_probe():
    """정상 설정을 그대로 읽는다."""
    spec, problems = parse_process_probe('imu_adapter', process_values())

    assert problems == []
    assert spec.name == 'imu_adapter'
    assert spec.cmdline_pattern == 'imu_base_link_adapter'
    assert spec.warn_percent == 60.0


def test_empty_cmdline_pattern_is_rejected():
    """패턴이 비면 모든 프로세스를 잡거나 아무것도 못 잡는다."""
    spec, problems = parse_process_probe('a', process_values(cmdline_pattern=''))

    assert spec is None
    assert any('cmdline_pattern' in p for p in problems)


def test_non_positive_warn_percent_is_rejected():
    """0 이하 임계는 항상 경고를 낸다."""
    spec, problems = parse_process_probe('a', process_values(warn_percent=0.0))

    assert spec is None
    assert any('warn_percent' in p for p in problems)


def test_process_probe_component_is_validated():
    """컴포넌트 이름을 함께 검사한다."""
    spec, problems = parse_process_probe('a', process_values(component='cpu'))

    assert spec is None
    assert any('component' in p for p in problems)


# ---------------------------------------------------------------------------
# 검증 보조 함수
# ---------------------------------------------------------------------------


def test_validate_component_accepts_known_names():
    """fault_catalog.COMPONENTS를 정본으로 쓴다."""
    assert validate_component('motor') is None
    assert validate_component('perception') is None


def test_validate_component_rejects_unknown():
    """모르는 이름은 문제 문구를 돌려준다."""
    problem = validate_component('motorr')
    assert problem is not None
    assert 'motorr' in problem


def test_validate_fault_code_accepts_catalog_entries():
    """카탈로그 항목을 통과시킨다."""
    assert validate_fault_code('MOTOR_CAN_TIMEOUT') is None
    # nvblox 는 감시 대상에서 뺐다(test_nvblox_is_no_longer_watched). 살아 있는 코드로 본다.
    assert validate_fault_code('LIDAR_SCAN_STALE') is None


def test_validate_fault_code_rejects_unknown():
    """모르는 코드는 문제 문구를 돌려준다."""
    problem = validate_fault_code('NOT_A_CODE')
    assert problem is not None
    assert 'NOT_A_CODE' in problem


# ---------------------------------------------------------------------------
# 미수신 원인 분류
# ---------------------------------------------------------------------------


def test_zero_message_with_no_publisher():
    """발행자가 없으면 그 노드가 안 뜬 것이다. 정상일 수 있다."""
    assert classify_zero_message(0) == ZERO_NO_PUBLISHER


def test_zero_message_with_publisher_suspects_qos():
    """발행자가 있는데 못 받으면 QoS 비호환이 유력하다.

    이것이 감시 도구 자체의 결함이며 가장 찾기 어려운 실패다. FrequencyStatus만 보면
    발행자 부재와 구별되지 않는다.
    """
    assert classify_zero_message(1) == ZERO_QOS_SUSPECTED
    assert classify_zero_message(3) == ZERO_QOS_SUSPECTED


def test_zero_message_handles_negative_count():
    """이상 입력에 예외를 던지지 않는다."""
    assert classify_zero_message(-1) == ZERO_NO_PUBLISHER
