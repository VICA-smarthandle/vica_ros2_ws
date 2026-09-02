"""Probe specification parsing and validation.

ROS 의존이 없다. 파라미터에서 읽은 평범한 dict만 다룬다.

이 모듈의 목적은 **설정 오타로 감시가 조용히 빠지는 것을 막는 것**이다. 잘못된 값을
기본값으로 흡수하지 않고 거부한다. 특히 QoS는 잘못 고르면 구독이 매칭되지 않아 메시지를
하나도 받지 못하고, 감시 도구가 스스로 "센서가 죽었다"고 영구 오탐한다. 그 오탐의 원인을
나중에 찾는 것은 매우 어렵다.

오류는 하나만 알려주고 멈추지 않고 전부 모아서 돌려준다. 한 번에 다 고칠 수 있게 한다.
"""

from typing import Dict, List, NamedTuple, Optional, Tuple

from .fault_catalog import CATALOG, COMPONENTS


# 구독 QoS 선택. 실제 rclpy QoSProfile 매핑은 노드가 한다 — 이 모듈은 ROS를 import하지
# 않는다.
QOS_SENSOR_DATA = 'sensor_data'
QOS_DEFAULT = 'default'

QOS_CHOICES = (QOS_SENSOR_DATA, QOS_DEFAULT)


class TopicProbeSpec(NamedTuple):
    """One topic-rate probe.

    optional=True는 메시지 타입을 import할 수 없을 때 이 프로브만 건너뛰라는 뜻이다.
    nvblox_msgs symlink가 빠져도 어댑터 전체가 기동 실패하지 않게 한다.
    """

    name: str
    component: str
    topic: str
    msg_type: str
    qos: str
    min_hz: float
    max_hz: float
    fault_code: str
    optional: bool


class ProcessProbeSpec(NamedTuple):
    """One /proc based CPU probe."""

    name: str
    component: str
    cmdline_pattern: str
    warn_percent: float
    # 이 프로세스가 없으면 결함인가.
    #
    # 기본은 False 다 — 이 프로브의 본뜻은 CPU baseline 기록이고, Docker 안의
    # 프로세스는 안 떠 있는 것이 정상인 경우가 많아 "미구성"으로 조용히 넘긴다.
    #
    # True 로 두면 부재 자체를 ERROR 로 올린다. **노드가 살아 있는지만 보고
    # 싶을 때** 쓴다 — 음성처럼 주기 토픽이 없어 다른 관측 수단이 없는 부품이다
    # (2026-09-02).
    required: bool = False


def validate_component(component: str) -> Optional[str]:
    """Return a problem message when the component is unknown, else None.

    정본은 fault_catalog.COMPONENTS다. 여기서 걸러야 앱에 알 수 없는 컴포넌트가 뜨지 않는다.
    """
    if component in COMPONENTS:
        return None
    return (
        f"component '{component}'은 알 수 없는 이름입니다. "
        f'허용: {", ".join(COMPONENTS)}'
    )


def validate_fault_code(fault_code: str) -> Optional[str]:
    """Return a problem message when the fault code is not in the catalog."""
    if fault_code in CATALOG:
        return None
    return (
        f"fault_code '{fault_code}'가 fault_catalog에 없습니다. "
        '카탈로그에 추가하거나 오타를 고쳐 주세요.'
    )


def parse_topic_probe(
    name: str,
    values: Dict[str, object],
) -> Tuple[Optional[TopicProbeSpec], List[str]]:
    """Parse and validate one topic probe.

    Returns (spec, problems). problems가 비어 있지 않으면 spec은 None이다.
    """
    problems: List[str] = []

    component = _as_str(values.get('component'))
    topic = _as_str(values.get('topic'))
    msg_type = _as_str(values.get('msg_type'))
    qos = _as_str(values.get('qos'))
    fault_code = _as_str(values.get('fault_code'))
    min_hz = _as_float(values.get('min_hz'))
    max_hz = _as_float(values.get('max_hz'))
    optional = bool(values.get('optional', False))

    problem = validate_component(component)
    if problem:
        problems.append(problem)

    problem = validate_fault_code(fault_code)
    if problem:
        problems.append(problem)

    if not topic:
        problems.append(f"'{name}': topic이 비어 있습니다.")
    if not msg_type:
        problems.append(f"'{name}': msg_type이 비어 있습니다.")

    if qos not in QOS_CHOICES:
        problems.append(
            f"'{name}': qos '{qos}'는 허용되지 않습니다. "
            f'허용: {", ".join(QOS_CHOICES)}. '
            'QoS가 맞지 않으면 메시지를 하나도 받지 못해 영구 오탐이 됩니다.'
        )

    if min_hz is None or min_hz <= 0.0:
        problems.append(
            f"'{name}': min_hz는 0보다 커야 합니다. 0이면 미수신을 감지할 수 없습니다."
        )
    if max_hz is None or max_hz <= 0.0:
        problems.append(f"'{name}': max_hz는 0보다 커야 합니다.")
    if (
        min_hz is not None
        and max_hz is not None
        and min_hz > 0.0
        and max_hz > 0.0
        and min_hz > max_hz
    ):
        problems.append(
            f"'{name}': min_hz({min_hz})가 max_hz({max_hz})보다 큽니다. "
            '범위가 뒤집혀 있으면 항상 실패합니다.'
        )

    if problems:
        return None, problems

    return (
        TopicProbeSpec(
            name=name,
            component=component,
            topic=topic,
            msg_type=msg_type,
            qos=qos,
            min_hz=min_hz,
            max_hz=max_hz,
            fault_code=fault_code,
            optional=optional,
        ),
        [],
    )


def parse_process_probe(
    name: str,
    values: Dict[str, object],
) -> Tuple[Optional[ProcessProbeSpec], List[str]]:
    """Parse and validate one process CPU probe."""
    problems: List[str] = []

    component = _as_str(values.get('component'))
    pattern = _as_str(values.get('cmdline_pattern'))
    warn_percent = _as_float(values.get('warn_percent'))
    required = bool(values.get('required', False))

    problem = validate_component(component)
    if problem:
        problems.append(problem)

    if not pattern:
        problems.append(
            f"'{name}': cmdline_pattern이 비어 있습니다. "
            '빈 패턴은 모든 프로세스를 잡거나 아무것도 잡지 못합니다.'
        )

    if warn_percent is None or warn_percent <= 0.0:
        problems.append(
            f"'{name}': warn_percent는 0보다 커야 합니다. "
            '0 이하면 항상 경고를 냅니다.'
        )

    if problems:
        return None, problems

    return (
        ProcessProbeSpec(
            name=name,
            component=component,
            cmdline_pattern=pattern,
            warn_percent=warn_percent,
            required=required,
        ),
        [],
    )


# 메시지를 하나도 받지 못한 원인 분류.
ZERO_NO_PUBLISHER = 'no_publisher'
ZERO_QOS_SUSPECTED = 'qos_mismatch_suspected'


def classify_zero_message(publisher_count: int) -> str:
    """Explain why a probe received zero messages.

    이 구분이 중요하다. FrequencyStatus만 보면 두 상황이 똑같이 "주기 미달"로 보인다.

        발행자 0개  -> 그 노드가 아직 안 떴다. 정상일 수 있다(센서 미연결, 미실행)
        발행자 1개+ -> 발행은 되는데 우리가 못 받는다. **QoS 비호환이 유력하다**

    두 번째가 감시 도구 자체의 결함이며 가장 찾기 어려운 실패다. `/scan`을 RELIABLE로
    구독하면 rplidar가 sensor_data(BEST_EFFORT)로 발행할 때 매칭이 안 되어 영구 오탐이
    된다. 정비하는 사람이 센서를 붙였다 뗐다 하며 시간을 버리지 않도록 구분해 남긴다.
    """
    if publisher_count <= 0:
        return ZERO_NO_PUBLISHER
    return ZERO_QOS_SUSPECTED


def _as_str(value: object) -> str:
    """Coerce a parameter value to a stripped string."""
    if value is None:
        return ''
    return str(value).strip()


def _as_float(value: object) -> Optional[float]:
    """Coerce a parameter value to float, or None when not numeric."""
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None
