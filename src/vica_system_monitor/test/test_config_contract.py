"""Contract tests between config/*.yaml and the code that consumes them.

`vica_nav2/test/test_nav2_params_contract.py`와 같은 목적이다. 설정 파일과 코드의 이름
집합이 어긋나면 감시가 조용히 빠지는데, 그것을 기동해 보지 않고 잡는다.

계획서 3절의 방어책 2에 해당한다. 노드를 고칠 때 YAML을 같은 커밋에서 바꾸도록 이 테스트가
누락을 강제한다.
"""

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

# 각 yaml의 최상위 키. launch의 Node(name=...)와 일치해야 한다.
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


# ---------------------------------------------------------------------------
# 파일 구조
# ---------------------------------------------------------------------------


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


# ---------------------------------------------------------------------------
# 이름 목록과 정의의 일치
# ---------------------------------------------------------------------------


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
    """정의만 있고 이름 목록에 없는 블록을 잡는다.

    이런 블록은 읽히지 않는다. 오타로 이름 목록에서 빠진 경우를 찾는다.
    """
    params = load_probe_params()
    listed = set(params.get('topic_probe_names', []))
    listed |= set(params.get('process_probe_names', []))

    scalars = {
        'diagnostic_period_sec',
        'process_scan_period_sec',
        'topic_probe_names',
        'process_probe_names',
    }

    for key, value in params.items():
        if key in scalars:
            continue
        if not isinstance(value, dict):
            continue
        assert key in listed, (
            f"'{key}' 정의 블록이 어느 이름 목록에도 없습니다. 읽히지 않습니다."
        )


# ---------------------------------------------------------------------------
# 각 프로브가 코드 검증을 통과하는가
# ---------------------------------------------------------------------------


def test_every_topic_probe_parses_cleanly():
    """실제 설정이 probe_config 검증을 통과한다.

    component 오타, 카탈로그에 없는 fault_code, 허용되지 않는 qos, 뒤집힌 Hz 범위를
    기동 전에 잡는다.
    """
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


# ---------------------------------------------------------------------------
# 설계 의도 고정
# ---------------------------------------------------------------------------


def test_camera_probes_watch_camera_info_not_image():
    """카메라는 원본 image가 아니라 camera_info를 봐야 한다.

    30 Hz depth 프레임을 복사하면 감시 노드가 대역폭 소비자가 된다. 같은 주기로
    나오면서 수백 바이트인 camera_info로 충분하다.
    """
    params = load_probe_params()

    for name in params.get('topic_probe_names', []):
        topic = params[name]['topic']
        if '/camera/' not in topic:
            continue
        assert 'camera_info' in topic, (
            f"'{name}'이 {topic}을 구독한다. 카메라는 camera_info를 봐야 한다."
        )


def test_nvblox_probe_is_optional():
    """nvblox_msgs symlink가 빠져도 어댑터가 기동 실패하지 않아야 한다."""
    params = load_probe_params()
    assert params['nvblox_slice']['optional'] is True


def test_baseline_probes_for_optimization_gate_exist():
    """최적화 baseline 계측 항목이 빠지지 않았는지 고정한다.

    리뷰 7.1절: imu adapter CPU 38.6 %와 EKF 30 Hz 미달이 유일한 before이며 재현할 수
    없다. 이 프로브가 빠지면 baseline을 잃는다.
    """
    params = load_probe_params()

    topic_names = params.get('topic_probe_names', [])
    assert 'odom' in topic_names, 'EKF 실효 Hz baseline 프로브가 없습니다'

    process_names = params.get('process_probe_names', [])
    assert 'imu_adapter' in process_names, 'imu adapter CPU baseline 프로브가 없습니다'


def test_thresholds_are_marked_unverified_in_comments():
    """임계값이 실측 전임을 파일에 명시했는지 확인한다.

    계획서 3절: 1차에서 임계값을 확정하지 않는다. 표기가 사라지면 다음 사람이 이 값을
    검증된 것으로 오해한다.
    """
    for path in (PROBES_YAML, AGGREGATOR_YAML, COMPONENTS_YAML):
        text = path.read_text(encoding='utf-8')
        assert '[미검증]' in text, f'{path.name}에 [미검증] 표기가 없습니다'


# ---------------------------------------------------------------------------
# required_components.yaml
# ---------------------------------------------------------------------------


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
    """RobotHealth의 readiness 필드와 정책 컴포넌트가 1:1이다.

    빠지면 그 필드가 영구 UNKNOWN으로 남고, 남으면 정책이 읽히지 않는다.
    """
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
    """관측 수단이 없는 컴포넌트를 필수로 두면 READY에 영원히 도달하지 못한다.

    Smart Handle은 상향 통신이 없고 음성·앱은 진단을 내지 않는다. 이들을 required로
    두면 로봇이 영구히 READY가 아니게 된다. 하드웨어·진단이 추가될 때 두 값을 함께
    바꿔야 한다.
    """
    params = load_monitor_params()

    for name in params.get('component_names', []):
        policy = params[name]
        if not policy['observable']:
            assert not policy['required'], (
                f"'{name}'이 observable=false인데 required=true다. "
                'READY에 영원히 도달하지 못한다.'
            )


def test_lidar_is_required_and_stop_severity():
    """LiDAR는 두 costmap의 유일한 장애물 입력이므로 필수·STOP이다.

    nav2_params.yaml에서 global·local costmap의 observation_sources가 scan 하나뿐이다.
    """
    params = load_monitor_params()
    assert params['lidar']['required'] is True
    assert int(params['lidar']['severity']) == 3


def test_motor_severity_is_estop():
    """모터 CAN 이상은 즉시 정지 사유다(초안 9.1절)."""
    params = load_monitor_params()
    assert int(params['motor']['severity']) == 4


def test_safety_input_timeouts_exist():
    """aggregator를 거치지 않는 안전 신호 timeout이 정의되어 있다.

    이 값들만 정책 계층이 소유한다. 진단 항목 timeout은 aggregator yaml이 갖는다.
    """
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


# ---------------------------------------------------------------------------
# diagnostic_aggregator.yaml
# ---------------------------------------------------------------------------


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
    """정책에 있는 컴포넌트가 aggregator 트리에서 분류된다.

    빠지면 그 컴포넌트의 진단이 Other로 흘러가 계층 name 매핑이 되지 않는다.
    lidar·perception처럼 우리가 만든 label 규칙(`<component>:`)에 맞춰 contains를 둔다.
    """
    agg_text = AGGREGATOR_YAML.read_text(encoding='utf-8')
    monitor_params = load_monitor_params()

    for name in monitor_params.get('component_names', []):
        assert f'{name}:' in agg_text or name in agg_text, (
            f"'{name}' 컴포넌트가 aggregator 설정에 없습니다. "
            '진단이 Other로 흘러 분류되지 않습니다.'
        )


def test_monitor_self_diagnostic_is_expected_by_aggregator():
    """감시 계층이 서로를 감시하는 구조를 고정한다.

    어댑터가 죽으면 aggregator가 expected 미충족으로 잡고, aggregator가 죽으면 모니터가
    /diagnostics_agg stale로 잡는다. 어느 한쪽이라도 빠지면 감시의 감시가 사라진다.
    """
    params = load_aggregator_params()
    monitor_analyzer = params['analyzers']['monitor']

    assert 'expected' in monitor_analyzer
    joined = ' '.join(monitor_analyzer['expected'])
    assert 'external_diagnostics_node' in joined


# ---------------------------------------------------------------------------
# launch
# ---------------------------------------------------------------------------


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
