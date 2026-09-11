"""Pure decision logic for one mapping session."""

from enum import Enum
import re
import signal


class MappingState(str, Enum):
    """What the supervisor is doing right now."""

    IDLE = 'idle'
    STARTING = 'starting'
    MAPPING = 'mapping'
    STOPPING = 'stopping'
    SAVING = 'saving'
    ERROR = 'error'


NAV2_NODES = ('amcl', 'bt_navigator', 'controller_server', 'planner_server')

MAPPING_NODES = ('cartographer_node', 'occupancy_grid_node')

SHARED_NODES = ('ekf_filter_node', 'encoder_feedback')

PREREQUISITE_LABELS = {
    'mdrobot_can_keyboard_knob_node': '모터(터미네이터 ⑤ 칸)',
    'camera/camera': '카메라(d455 Docker)',
    'imu_base_link_adapter': 'IMU 어댑터',
}


def _bare(name: str) -> str:
    """Strip the namespace so '/ns/amcl' and 'amcl' compare equal."""
    return name.rsplit('/', 1)[-1]


def duplicated_names(node_names) -> list:
    """Return node names that appear more than once."""
    counts = {}
    for name in node_names:
        bare = _bare(name)
        counts[bare] = counts.get(bare, 0) + 1
    return sorted(name for name, count in counts.items() if count > 1)


def running_stacks(node_names) -> dict:
    """Report which stacks are up, ignoring the shared nodes."""
    bare = {_bare(name) for name in node_names}
    return {
        'nav2': any(name in bare for name in NAV2_NODES),
        'mapping': any(name in bare for name in MAPPING_NODES),
    }


def blocking_reason(state: MappingState, node_names, required=()):
    """Return why mapping must not start now, or None when it may."""
    if state is not MappingState.IDLE:
        return f'이미 {state.value} 상태입니다. 먼저 종료해 주세요.'

    duplicates = duplicated_names(node_names)
    if duplicates:
        return (
            '같은 노드가 두 번 떠 있습니다: '
            + ', '.join(duplicates)
            + '. 한쪽을 내린 뒤 다시 시도해 주세요.'
        )

    stacks = running_stacks(node_names)
    if stacks['nav2']:
        return (
            'Nav2 가 실행 중입니다. SLAM 과 동시에 뜨면 /odom 발행자가 둘이 되어 '
            '위치추정이 깨집니다. Nav2 를 먼저 내려 주세요.'
        )
    if stacks['mapping']:
        return '매핑 스택이 이미 실행 중입니다.'

    missing = missing_prerequisites(node_names, list(required))
    if missing:
        labels = ', '.join(
            PREREQUISITE_LABELS.get(name, _bare(name)) for name in missing
        )
        return (
            f'매핑에 필요한 것이 아직 안 떠 있습니다: {labels}. '
            '전원·CAN 과 해당 터미널 칸을 확인한 뒤 다시 시도해 주세요.'
        )
    return None


def missing_prerequisites(node_names, required):
    """Return the required nodes that are not up yet."""
    bare = {_bare(name) for name in node_names}
    return [name for name in required if _bare(name) not in bare]


class StopEscalation:
    """SIGINT → SIGTERM → SIGKILL 사다리를 **기다리지 않고** 오른다."""

    LADDER = (signal.SIGINT, signal.SIGTERM, signal.SIGKILL)

    def __init__(self, grace_sec: float, now: float):
        """Arm the ladder. now 는 monotonic 초, 유예는 단계마다 grace_sec."""
        self._grace = grace_sec
        self._stage = 0
        self._deadline = now + grace_sec

    @property
    def first_signal(self):
        """시작하자마자 보낼 신호. 항상 SIGINT 다."""
        return self.LADDER[0]

    def escalate_signal(self, now: float):
        """유예가 지났으면 다음 신호를 돌려준다. 아직이면 None."""
        if now < self._deadline:
            return None
        self._deadline = now + self._grace
        if self._stage + 1 < len(self.LADDER):
            self._stage += 1
            return self.LADDER[self._stage]
        return None


def is_stack_up(node_names) -> bool:
    """Tell whether the mapping stack has finished coming up."""
    return running_stacks(node_names)['mapping']


MAP_NAME_PATTERN = re.compile(r'^[A-Za-z0-9_-]+$')
MAP_NAME_MAX = 48


def normalise_map_name(raw: str, date_suffix: str):
    """Turn a typed name into the final map id, or explain why it cannot."""
    name = (raw or '').strip()
    if not name:
        return None, '지도 이름을 입력해 주세요.'
    if not MAP_NAME_PATTERN.match(name):
        return None, (
            '지도 이름에는 영문·숫자·밑줄(_)·붙임표(-)만 쓸 수 있습니다. '
            '확장자와 경로는 붙이지 않습니다.'
        )
    if not name.endswith('_' + date_suffix):
        name = f'{name}_{date_suffix}'
    if len(name) > MAP_NAME_MAX:
        return None, f'지도 이름이 너무 깁니다({len(name)}자). {MAP_NAME_MAX}자 이내로 해주세요.'
    return name, ''
