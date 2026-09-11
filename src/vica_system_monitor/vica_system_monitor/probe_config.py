"""Probe specification parsing and validation."""

from typing import Dict, List, NamedTuple, Optional, Tuple

from .fault_catalog import CATALOG, COMPONENTS


QOS_SENSOR_DATA = 'sensor_data'
QOS_DEFAULT = 'default'

QOS_CHOICES = (QOS_SENSOR_DATA, QOS_DEFAULT)


class TopicProbeSpec(NamedTuple):
    """One topic-rate probe."""

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


def validate_component(component: str) -> Optional[str]:
    """Return a problem message when the component is unknown, else None."""
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
    """Parse and validate one topic probe."""
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
        ),
        [],
    )


ZERO_NO_PUBLISHER = 'no_publisher'
ZERO_QOS_SUSPECTED = 'qos_mismatch_suspected'


def classify_zero_message(publisher_count: int) -> str:
    """Explain why a probe received zero messages."""
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
