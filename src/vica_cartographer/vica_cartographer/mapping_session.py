"""Pure decision logic for one mapping session.

rclpy 도 subprocess 도 쓰지 않는다. 노트북에서 pytest 로 전부 검증할 수 있어야
하기 때문이다. 프로세스를 실제로 띄우고 죽이는 일은 mapping_supervisor_node 가 한다.

**왜 시작 전에 검사하는가.** 중복 실행이 이 프로젝트에서 실제로 회차를 날린 사고다.

  - 2026-08-11 20:07 에 스택이 두 벌 돌아 매핑 두 회차를 잃었다
    (docs/cartographer_corridor_mapping.md 3절)
  - Nav2 와 SLAM 은 둘 다 wheel_ekf.launch.py 를 include 하므로 동시에 뜨면
    /odom 발행자가 둘이 된다. vica_map 프로파일 설명이 "nav2 는 뺀 것이 아니라
    넣으면 안 되는 것"이라고 적었다

버튼은 두 번 누르기가 너무 쉽다. 그래서 사람 손이 아니라 코드가 막는다.
"""

from enum import Enum
import re
import signal


class MappingState(str, Enum):
    """What the supervisor is doing right now."""

    IDLE = 'idle'            # 아무것도 안 띄웠다
    STARTING = 'starting'    # launch 를 띄웠고 노드가 올라오기를 기다린다
    MAPPING = 'mapping'      # 지도를 그리는 중
    STOPPING = 'stopping'    # 자식 프로세스를 정리하는 중
    SAVING = 'saving'        # 저장 명령이 도는 중
    ERROR = 'error'          # 사람이 봐야 한다


# Nav2 를 대표하는 노드. lifecycle 관리 대상이라 하나만 있어도 '떴다'로 본다.
NAV2_NODES = ('amcl', 'bt_navigator', 'controller_server', 'planner_server')

# 매핑을 대표하는 노드.
MAPPING_NODES = ('cartographer_node', 'occupancy_grid_node')

# 두 스택이 공유하는 노드. 여기서 같은 이름이 두 번 보이면 두 벌이 도는 것이다.
# 어느 쪽 스택인지는 가르지 못하므로 '떴는지' 판정에는 쓰지 않는다.
SHARED_NODES = ('ekf_filter_node', 'encoder_feedback')

# 필수 노드의 사람용 이름. blocking_reason 이 거부 사유를 만들 때 쓴다 —
# 노드 이름을 그대로 보여주면 관리자가 무엇을 어디서 띄워야 하는지 모른다.
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
    """Return why mapping must not start now, or None when it may.

    검사 순서가 곧 우선순위다. 가장 위험한 것부터 본다. 필수 노드 부재는 맨
    뒤다 — 위험이 아니라 헛수고를 막는 검사라서다. 없이 시작하면 40초 뒤
    STARTING 시한 초과로만 죽어 회차가 통째로 무효가 된다(모터가 없으면
    /wheel/odom 이 영영 안 나온다). 시작 전에 사람이 할 조치를 알려주는 편이
    낫다.
    """
    if state is not MappingState.IDLE:
        return f'이미 {state.value} 상태입니다. 먼저 종료해 주세요.'

    duplicates = duplicated_names(node_names)
    if duplicates:
        # 이미 사고가 난 상태다. 여기에 한 벌을 더 얹으면 안 된다.
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
    """Return the required nodes that are not up yet.

    d455 는 Docker 라, imu 는 자이로 보정에 20초 정지가 필요해서 앱이 띄우지 않는다.
    대신 떠 있는지 확인은 한다 — 없으면 회차가 통째로 무효가 되기 때문이다.
    (docs/cartographer_corridor_mapping.md 3절 점검표)
    """
    bare = {_bare(name) for name in node_names}
    return [name for name in required if _bare(name) not in bare]


class StopEscalation:
    """SIGINT → SIGTERM → SIGKILL 사다리를 **기다리지 않고** 오른다.

    종전에는 _terminate_process 가 각 단계에서 wait(유예 8초)로 서 있었다.
    콜백 안에서 부르면 최악 24초 동안 _lock 을 잡은 채라 start/stop/save 와
    상태 tick 이 전부 줄을 서고, rosbridge(default_call_service_timeout=0.0)
    까지 막혀 앱 전체가 먹통이 됐다 — 저장을 즉시 응답으로 바꾼 것과 같은
    병이다(모듈 머리말, 2026-08-21 교훈).

    이 객체는 시간을 기다리지 않는다. 노드 타이머가 매 tick 물어보면
    '지금 보낼 신호'만 알려준다. 신호를 실제로 보내고 프로세스 생사를 보는
    일은 노드 몫이다 — 이 모듈이 subprocess 를 쓰지 않는 원칙 그대로다.
    """

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
        """유예가 지났으면 다음 신호를 돌려준다. 아직이면 None.

        SIGKILL 뒤에는 더 올릴 데가 없으므로 None 만 돌려준다 — SIGKILL 은
        무시될 수 없어 poll() 이 곧 시체를 거둔다.
        """
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


# scripts/vica_map_save.sh 가 강제하는 규칙과 같아야 한다. 그 스크립트가
# 거부하면 사람이 지도를 다 그린 뒤에 거부당한다.
#   "ROS 의 map yaml 과 앱의 HTTP 경로가 이 이름을 그대로 쓰기 때문입니다"
MAP_NAME_PATTERN = re.compile(r'^[A-Za-z0-9_-]+$')
MAP_NAME_MAX = 48


def normalise_map_name(raw: str, date_suffix: str):
    """Turn a typed name into the final map id, or explain why it cannot.

    "제목은 사람이, 날짜는 자동"이 운영 규칙이다. 사람이 'lobby' 라고 적으면
    'lobby_0821' 이 된다. 이미 날짜가 붙어 있으면 두 번 붙이지 않는다.

    Returns (map_id, error). 둘 중 하나만 채워진다.
    """
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
