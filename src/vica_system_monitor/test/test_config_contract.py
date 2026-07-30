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
)
import yaml


CONFIG_DIR = pathlib.Path(__file__).resolve().parent.parent / 'config'
PROBES_YAML = CONFIG_DIR / 'probes.yaml'

# probes.yaml의 최상위 키. launch의 Node(name=...)와 일치해야 한다.
NODE_NAME = 'external_diagnostics_node'


def load_probe_params():
    """Load the ros__parameters block from probes.yaml."""
    with open(PROBES_YAML, encoding='utf-8') as handle:
        document = yaml.safe_load(handle)
    return document[NODE_NAME]['ros__parameters']


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
    text = PROBES_YAML.read_text(encoding='utf-8')
    assert '[미검증]' in text, 'probes.yaml에 [미검증] 표기가 없습니다'
