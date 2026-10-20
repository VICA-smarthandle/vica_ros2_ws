"""Contract tests between config/*.yaml and the code that consumes them."""

import pathlib

from vica_system_monitor.probe_config import (
    parse_process_probe,
    parse_topic_probe,
    validate_component,
)
import yaml


PACKAGE_DIR = pathlib.Path(__file__).resolve().parent.parent
CONFIG_DIR = PACKAGE_DIR / 'config'
PROBES_YAML = CONFIG_DIR / 'probes.yaml'
AGGREGATOR_YAML = CONFIG_DIR / 'diagnostic_aggregator.yaml'
COMPONENTS_YAML = CONFIG_DIR / 'required_components.yaml'
LAUNCH_FILE = PACKAGE_DIR / 'launch' / 'system_monitor.launch.py'

NODE_NAME = 'external_diagnostics_node'
MONITOR_NODE_NAME = 'robot_health_monitor_node'
AGGREGATOR_NODE_NAME = 'aggregator_node'


def load_probe_params():
    """Load the ros__parameters block from probes.yaml."""
    with open(PROBES_YAML, encoding='utf-8') as handle:
        document = yaml.safe_load(handle)
    return document[NODE_NAME]['ros__parameters']


def load_monitor_params():
    """Load the ros__parameters block from required_components.yaml."""
    with open(COMPONENTS_YAML, encoding='utf-8') as handle:
        document = yaml.safe_load(handle)
    return document[MONITOR_NODE_NAME]['ros__parameters']


def load_aggregator_params():
    """Load the ros__parameters block from diagnostic_aggregator.yaml."""
    with open(AGGREGATOR_YAML, encoding='utf-8') as handle:
        document = yaml.safe_load(handle)
    return document[AGGREGATOR_NODE_NAME]['ros__parameters']


def test_probes_yaml_exists():
    """설정 파일이 설치 대상 위치에 있다."""
    assert PROBES_YAML.is_file(), f'{PROBES_YAML}가 없습니다'


def test_top_level_key_matches_node_name():
    """최상위 키가 노드 이름과 다르면 파라미터가 조용히 무시된다."""
    with open(PROBES_YAML, encoding='utf-8') as handle:
        document = yaml.safe_load(handle)

    assert NODE_NAME in document, (
        f'최상위 키가 {NODE_NAME}이어야 한다. '
        '다르면 launch가 파라미터를 넘겨도 전부 기본값으로 동작한다.'
    )
    assert 'ros__parameters' in document[NODE_NAME]


def test_required_scalar_parameters_exist():
    """노드가 declare하는 스칼라 파라미터가 설정에 있다."""
    params = load_probe_params()

    for key in ('diagnostic_period_sec', 'process_scan_period_sec'):
        assert key in params, f'{key}가 probes.yaml에 없습니다'
        assert float(params[key]) > 0.0


def test_every_named_topic_probe_has_a_definition():
    """이름 목록에 있으나 정의가 없으면 그 프로브가 조용히 빠진다."""
    params = load_probe_params()
    names = params.get('topic_probe_names', [])

    assert names, 'topic_probe_names가 비어 있습니다'
    for name in names:
        assert name in params, (
            f"'{name}'이 topic_probe_names에 있으나 정의 블록이 없습니다. "
            '이 프로브는 조용히 빠집니다.'
        )


def test_every_named_process_probe_has_a_definition():
    """Process 프로브도 같은 검사를 한다."""
    params = load_probe_params()
    names = params.get('process_probe_names', [])

    assert names, 'process_probe_names가 비어 있습니다'
    for name in names:
        assert name in params, f"'{name}'의 정의 블록이 없습니다"


def test_no_orphan_probe_definitions():
    """정의만 있고 이름 목록에 없는 블록을 잡는다."""
    params = load_probe_params()
    listed = set(params.get('topic_probe_names', []))
    listed |= set(params.get('process_probe_names', []))
    listed |= set(params.get('link_probe_names', []))

    scalars = {
        'diagnostic_period_sec',
        'process_scan_period_sec',
        'topic_probe_names',
        'process_probe_names',
        'link_probe_names',
    }

    for key, value in params.items():
        if key in scalars:
            continue
        if not isinstance(value, dict):
            continue
        assert key in listed, (
            f"'{key}' 정의 블록이 어느 이름 목록에도 없습니다. 읽히지 않습니다."
        )


def test_every_topic_probe_parses_cleanly():
    """실제 설정이 probe_config 검증을 통과한다."""
    params = load_probe_params()
    failures = []

    for name in params.get('topic_probe_names', []):
        spec, problems = parse_topic_probe(name, params[name])
        if problems:
            failures.append(f'{name}: {problems}')
        else:
            assert spec is not None

    assert not failures, '\n'.join(failures)


def test_every_process_probe_parses_cleanly():
    """Process 프로브도 검증을 통과한다."""
    params = load_probe_params()
    failures = []

    for name in params.get('process_probe_names', []):
        spec, problems = parse_process_probe(name, params[name])
        if problems:
            failures.append(f'{name}: {problems}')
        else:
            assert spec is not None

    assert not failures, '\n'.join(failures)


def test_camera_probes_never_subscribe_to_raw_frames():
    """카메라 계열 프로브는 원본 프레임을 구독하지 않는다."""
    params = load_probe_params()
    heavy = ('image', 'points', 'depth/color/points')

    for name in params.get('topic_probe_names', []):
        topic = params[name]['topic']
        for token in heavy:
            assert token not in topic, (
                f"'{name}'이 {topic}을 구독한다. 원본 프레임은 감시하지 않는다."
            )


def test_camera_probes_are_optional():
    """카메라가 없어도 어댑터가 기동 실패하지 않아야 한다."""
    params = load_probe_params()
    for name in ('depth_scan', 'camera_depth', 'camera_color'):
        assert params[name]['optional'] is True, (
            f"'{name}'이 optional이 아니다. 카메라가 없으면 어댑터가 죽는다."
        )


def test_nvblox_is_no_longer_watched():
    """Nvblox 감시가 되살아나지 않게 잠근다."""
    params = load_probe_params()
    names = list(params.get('topic_probe_names', [])) + list(
        params.get('process_probe_names', [])
    )
    for name in names:
        assert 'nvblox' not in name, f"'{name}'이 남아 있다."
        topic = params[name].get('topic', '')
        pattern = params[name].get('cmdline_pattern', '')
        assert 'nvblox' not in topic and 'nvblox' not in pattern, (
            f"'{name}'이 아직 nvblox를 본다."
        )


def test_baseline_probes_for_optimization_gate_exist():
    """최적화 baseline 계측 항목이 빠지지 않았는지 고정한다."""
    params = load_probe_params()

    topic_names = params.get('topic_probe_names', [])
    assert 'odom' in topic_names, 'EKF 실효 Hz baseline 프로브가 없습니다'

    process_names = params.get('process_probe_names', [])
    assert 'imu_adapter' in process_names, 'imu adapter CPU baseline 프로브가 없습니다'


def test_unmeasured_configs_stay_marked_unverified():
    """아직 실측하지 않은 설정은 `[미검증]` 표기를 유지해야 한다."""
    for path in (AGGREGATOR_YAML, COMPONENTS_YAML):
        text = path.read_text(encoding='utf-8')
        assert '[미검증]' in text, f'{path.name}에 [미검증] 표기가 없습니다'


def test_probes_yaml_records_measurement_and_remaining_gaps():
    """probes.yaml은 실측 근거와 **남은 미측정 항목**을 함께 적어야 한다."""
    text = PROBES_YAML.read_text(encoding='utf-8')

    assert '실측 2026-08-01' in text, 'probes.yaml에 실측 근거 표기가 없습니다'
    assert '[미측정]' in text, (
        'probes.yaml에 남은 미측정 항목 표기가 없습니다. '
        '정지 상태에서만 쟀다는 사실이 사라지면 안 됩니다'
    )


def test_monitor_yaml_top_level_key_matches_node_name():
    """최상위 키가 노드 이름과 다르면 파라미터가 조용히 무시된다."""
    with open(COMPONENTS_YAML, encoding='utf-8') as handle:
        document = yaml.safe_load(handle)

    assert MONITOR_NODE_NAME in document
    assert 'ros__parameters' in document[MONITOR_NODE_NAME]


def test_every_named_component_has_a_policy_block():
    """이름 목록에 있으나 정책 블록이 없으면 기본값으로 동작한다."""
    params = load_monitor_params()
    names = params.get('component_names', [])

    assert names, 'component_names가 비어 있습니다'
    for name in names:
        assert name in params, f"'{name}'의 정책 블록이 없습니다"


def test_every_component_policy_has_required_fields():
    """노드가 읽는 네 필드가 모두 있다."""
    params = load_monitor_params()

    for name in params.get('component_names', []):
        policy = params[name]
        for field in ('required', 'observable', 'severity', 'grace_sec'):
            assert field in policy, f"'{name}'에 {field}가 없습니다"


def test_component_names_are_known_to_fault_catalog():
    """정책 컴포넌트가 카탈로그 이름 집합과 일치한다."""
    params = load_monitor_params()
    failures = [
        validate_component(name)
        for name in params.get('component_names', [])
        if validate_component(name) is not None
    ]
    assert not failures, failures


def test_component_names_cover_readiness_fields():
    """RobotHealth의 readiness 필드와 정책 컴포넌트가 1:1이다."""
    params = load_monitor_params()
    names = set(params.get('component_names', []))

    expected = {
        'motor',
        'safety',
        'localization',
        'navigation',
        'lidar',
        'perception',
        'guidance',
        'voice',
        'app',
    }
    assert names == expected, f'차이: {names ^ expected}'


def test_severity_values_are_in_range():
    """등급 값이 RobotFault.SEVERITY_* 범위 안이다."""
    params = load_monitor_params()

    for name in params.get('component_names', []):
        severity = int(params[name]['severity'])
        assert 1 <= severity <= 5, f"'{name}' severity {severity}가 범위를 벗어났습니다"


def test_unobservable_components_are_not_required():
    """관측 수단이 없는 컴포넌트를 필수로 두면 READY에 영원히 도달하지 못한다."""
    params = load_monitor_params()

    for name in params.get('component_names', []):
        policy = params[name]
        if not policy['observable']:
            assert not policy['required'], (
                f"'{name}'이 observable=false인데 required=true다. "
                'READY에 영원히 도달하지 못한다.'
            )


def test_lidar_is_required_and_stop_severity():
    """LiDAR는 두 costmap의 유일한 장애물 입력이므로 필수·STOP이다."""
    params = load_monitor_params()
    assert params['lidar']['required'] is True
    assert int(params['lidar']['severity']) == 3


def test_motor_severity_blocks_driving():
    """모터 CAN 이상은 주행 불가 사유다(초안 9.1절)."""
    params = load_monitor_params()
    assert int(params['motor']['severity']) == 3


def test_no_component_policy_uses_a_removed_severity():
    """설정이 제거된 등급 값을 쓰지 않는다."""
    params = load_monitor_params()
    for name, policy in params.items():
        if isinstance(policy, dict) and 'severity' in policy:
            assert 0 <= int(policy['severity']) <= 4, name


def test_safety_input_timeouts_exist():
    """aggregator를 거치지 않는 안전 신호 timeout이 정의되어 있다."""
    params = load_monitor_params()

    for key in (
        'emergency_stop_timeout_sec',
        'safety_state_timeout_sec',
        'tf_timeout_sec',
        'diagnostics_timeout_sec',
    ):
        assert key in params, f'{key}가 없습니다'
        assert float(params[key]) > 0.0


def test_monitor_reads_aggregator_output_by_default():
    """기본 입력이 aggregator 출력이다. 단독 디버깅은 launch 인자로 바꾼다."""
    params = load_monitor_params()
    assert params['diagnostics_topic'] == '/diagnostics_agg'


def test_aggregator_yaml_top_level_key():
    """diagnostic_aggregator 노드 이름과 맞아야 파라미터가 전달된다."""
    with open(AGGREGATOR_YAML, encoding='utf-8') as handle:
        document = yaml.safe_load(handle)

    assert AGGREGATOR_NODE_NAME in document
    params = document[AGGREGATOR_NODE_NAME]['ros__parameters']
    assert params['path'] == 'VICA'
    assert 'analyzers' in params


def test_every_analyzer_declares_a_type_and_path():
    """analyzer에 type이나 path가 없으면 aggregator가 기동 실패한다."""
    params = load_aggregator_params()

    def walk(node, trail):
        for name, spec in node.items():
            if not isinstance(spec, dict):
                continue
            assert 'type' in spec, f'{trail}/{name}에 type이 없습니다'
            assert 'path' in spec, f'{trail}/{name}에 path가 없습니다'
            if 'analyzers' in spec:
                walk(spec['analyzers'], f'{trail}/{name}')

    walk(params['analyzers'], '')


def test_every_leaf_analyzer_has_a_timeout():
    """timeout이 없으면 Stale 판정이 되지 않아 죽은 노드를 못 잡는다."""
    params = load_aggregator_params()

    def walk(node, trail):
        for name, spec in node.items():
            if not isinstance(spec, dict):
                continue
            if 'analyzers' in spec:
                walk(spec['analyzers'], f'{trail}/{name}')
                continue
            assert 'timeout' in spec, f'{trail}/{name}에 timeout이 없습니다'
            assert float(spec['timeout']) > 0.0

    walk(params['analyzers'], '')


def test_aggregator_covers_every_policy_component():
    """정책에 있는 컴포넌트가 aggregator 트리에서 분류된다."""
    agg_text = AGGREGATOR_YAML.read_text(encoding='utf-8')
    monitor_params = load_monitor_params()

    for name in monitor_params.get('component_names', []):
        assert f'{name}:' in agg_text or name in agg_text, (
            f"'{name}' 컴포넌트가 aggregator 설정에 없습니다. "
            '진단이 Other로 흘러 분류되지 않습니다.'
        )


def test_monitor_self_diagnostic_is_expected_by_aggregator():
    """감시 계층이 서로를 감시하는 구조를 고정한다."""
    params = load_aggregator_params()
    monitor_analyzer = params['analyzers']['monitor']

    assert 'expected' in monitor_analyzer
    joined = ' '.join(monitor_analyzer['expected'])
    assert 'external_diagnostics_node' in joined


def test_launch_file_exists():
    """Launch 파일이 설치 대상 위치에 있다."""
    assert LAUNCH_FILE.is_file()


def test_launch_declares_all_three_nodes():
    """어댑터·aggregator·모니터 세 프로세스를 띄운다."""
    text = LAUNCH_FILE.read_text(encoding='utf-8')

    for executable in (
        'external_diagnostics_node',
        'aggregator_node',
        'robot_health_monitor_node',
    ):
        assert executable in text, f'{executable}가 launch에 없습니다'


def test_launch_node_names_match_yaml_top_level_keys():
    """Node(name=...)가 yaml 최상위 키와 달라지면 파라미터가 조용히 무시된다."""
    text = LAUNCH_FILE.read_text(encoding='utf-8')

    for name in (NODE_NAME, MONITOR_NODE_NAME, AGGREGATOR_NODE_NAME):
        assert f"name='{name}'" in text, f"launch에 name='{name}'이 없습니다"


def test_launch_exposes_enable_aggregator_argument():
    """Aggregator 없이 단독 디버깅할 수 있어야 한다."""
    text = LAUNCH_FILE.read_text(encoding='utf-8')
    assert 'enable_aggregator' in text
    assert 'IfCondition' in text
    assert 'UnlessCondition' in text


def test_launch_uses_all_three_config_files():
    """설정 파일 세 개가 모두 연결되어 있다."""
    text = LAUNCH_FILE.read_text(encoding='utf-8')

    for name in (
        'probes.yaml',
        'diagnostic_aggregator.yaml',
        'required_components.yaml',
    ):
        assert name in text, f'{name}이 launch에 연결되지 않았습니다'


def _probe(name):
    params = load_probe_params()
    assert name in params.get('topic_probe_names', []), f'{name} 프로브가 없습니다'
    return params[name]


def test_guidance_is_observable_with_two_rungs():
    """드라이버 생존과 아두이노 응답을 따로 본다 — 한 칸이면 구분이 안 된다."""
    comp = load_monitor_params()['guidance']
    assert comp['observable'] is True

    state = _probe('handle_state')
    uplink = _probe('handle_uplink')
    assert state['component'] == 'guidance'
    assert uplink['component'] == 'guidance'
    assert state['topic'] != uplink['topic']


def test_guidance_does_not_block_driving():
    """핸들이 없어도 로봇은 달린다. STOP 으로 올리면 진짜 주행 결함과 섞인다."""
    comp = load_monitor_params()['guidance']
    assert comp['required'] is False
    assert comp['severity'] == 2


def test_app_bridge_is_observable_and_warn_only():
    """브리지는 로봇 동작과 무관하다 — WARN 상한이다."""
    comp = load_monitor_params()['app']
    assert comp['observable'] is True
    assert comp['required'] is False
    assert comp['severity'] == 1
    assert _probe('app_bridge')['component'] == 'app'


def test_new_probe_fault_codes_exist_in_catalog():
    """문구 없는 결함 코드는 화면에 코드만 뜬다."""
    from vica_system_monitor.fault_catalog import CATALOG
    for name in ('handle_state', 'handle_uplink', 'app_bridge'):
        code = _probe(name)['fault_code']
        assert code in CATALOG, f'{code} 문구가 카탈로그에 없습니다'


def test_new_fault_messages_say_whether_driving_is_affected():
    """관리자가 로봇을 세우러 뛰어갈지 말지를 문구에서 바로 알아야 한다."""
    from vica_system_monitor.fault_catalog import CATALOG
    for code in ('GUIDANCE_NODE_SILENT', 'GUIDANCE_UPLINK_STALE', 'APP_BRIDGE_SILENT'):
        assert '주행' in CATALOG[code].detail_template, f'{code} 에 주행 영향이 없습니다'


def test_guidance_messages_name_both_handle_and_ultrasonic():
    """아두이노 하나에 둘이 물려 있다. 하나만 적으면 나머지를 못 찾는다."""
    from vica_system_monitor.fault_catalog import CATALOG
    for code in ('GUIDANCE_NODE_SILENT', 'GUIDANCE_UPLINK_STALE'):
        action = CATALOG[code].suggested_action
        assert '초음파' in action, f'{code} 에 초음파 안내가 없습니다'


def test_voice_is_observed_by_process_only():
    """주기 토픽이 없어 프로세스로만 본다."""
    params = load_probe_params()
    names = params.get('process_probe_names', [])
    for name in ('voice_wakeword', 'voice_tts', 'voice_llm'):
        assert name in names, f'{name} 프로브가 없습니다'
        assert params[name]['component'] == 'voice'
    for name in params.get('topic_probe_names', []):
        assert params[name]['component'] != 'voice'


def test_voice_process_absence_is_a_fault():
    """Required 가 없으면 프로세스가 없어도 OK 로 지나가 죽음을 못 잡는다."""
    params = load_probe_params()
    for name in ('voice_wakeword', 'voice_tts', 'voice_llm'):
        assert params[name].get('required') is True, f'{name} 에 required 가 없습니다'


def test_voice_is_observable_but_not_required():
    """실행 여부는 보되 주행을 막지 않는다."""
    comp = load_monitor_params()['voice']
    assert comp['observable'] is True
    assert comp['required'] is False
    assert comp['severity'] == 2


def test_cmd_vel_req_link_is_watched():
    """주행 명령이 나갈 길이 열렸는지 본다."""
    params = load_probe_params()
    assert 'cmd_vel_req_link' in params.get('link_probe_names', [])
    spec = params['cmd_vel_req_link']
    assert spec['topic'] == '/cmd_vel_req'
    assert spec['component'] == 'navigation'
    assert spec['min_publishers'] >= 1


def test_cmd_vel_req_link_blocks_driving():
    """길이 막히면 주행이 안 된다 — WARN 으로 두면 놓친다."""
    from vica_system_monitor.fault_catalog import CATALOG, SEVERITY_STOP
    code = load_probe_params()['cmd_vel_req_link']['fault_code']
    assert CATALOG[code].severity == SEVERITY_STOP


def test_cmd_vel_req_link_tells_what_to_do():
    """원인을 짐작해 껐다 켜는 일이 없어야 한다."""
    from vica_system_monitor.fault_catalog import CATALOG
    spec = CATALOG[load_probe_params()['cmd_vel_req_link']['fault_code']]
    assert 'collision_monitor' in spec.suggested_action
    assert 'Nav2' in spec.suggested_action


def test_link_probe_is_not_a_topic_probe():
    """구독하면 안 된다. 그래프만 본다 — 대역폭을 쓰지 않는 것이 설계다."""
    params = load_probe_params()
    for name in params.get('link_probe_names', []):
        assert name not in params.get('topic_probe_names', [])
        assert 'msg_type' not in params[name]
        assert 'min_hz' not in params[name]


def _probe_blocks(params, list_key):
    """이름 목록에 적힌 프로브 블록들을 (이름, 블록) 으로 돌려준다."""
    return [(name, params[name]) for name in params.get(list_key, []) if name]


def test_link_probe_min_publishers_is_a_whole_number():
    """발행자 수는 정수다. 노드 기본값도 정수(1)라 YAML 도 1 로 적는다."""
    params = load_probe_params()
    for name, block in _probe_blocks(params, 'link_probe_names'):
        value = block.get('min_publishers')
        assert isinstance(value, int) and not isinstance(value, bool), (
            f"'{name}'.min_publishers 는 정수여야 합니다: {value!r}"
        )
        assert value >= 1


def test_topic_probe_rates_are_numbers_and_flags_are_booleans():
    """주기 한계는 숫자, optional 은 참/거짓이다. 따옴표로 감싼 숫자는 거른다."""
    params = load_probe_params()
    for name, block in _probe_blocks(params, 'topic_probe_names'):
        for field in ('min_hz', 'max_hz'):
            value = block.get(field)
            assert isinstance(value, (int, float)) and not isinstance(value, bool), (
                f"'{name}'.{field} 는 숫자여야 합니다: {value!r}"
            )
        if 'optional' in block:
            assert isinstance(block['optional'], bool), f"'{name}'.optional"


def test_process_probe_percent_is_a_number():
    """CPU 경고 임계는 숫자, required 는 참/거짓이다."""
    params = load_probe_params()
    for name, block in _probe_blocks(params, 'process_probe_names'):
        if 'warn_percent' in block:
            value = block['warn_percent']
            assert isinstance(value, (int, float)) and not isinstance(value, bool), (
                f"'{name}'.warn_percent 는 숫자여야 합니다: {value!r}"
            )
        if 'required' in block:
            assert isinstance(block['required'], bool), f"'{name}'.required"


def test_monitor_declares_adapter_grace():
    """어댑터 사망 판정의 기동 유예가 설정에 있고 양수다."""
    params = load_monitor_params()
    assert 'adapter_grace_sec' in params
    assert float(params['adapter_grace_sec']) > 0.0


def test_monitor_declares_raise_confirm_ticks():
    """발생 확인 tick 이 설정에 있고 1 이상이다."""
    params = load_monitor_params()
    assert 'raise_confirm_ticks' in params
    assert int(params['raise_confirm_ticks']) >= 1
