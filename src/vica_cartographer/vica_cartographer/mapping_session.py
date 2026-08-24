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

# motor node. 매핑 launch 가 띄울 수도 있고 사람이 터미널에서 띄울 수도 있어서,
# 시작 전에 이미 떠 있는지 보고 중복을 피한다.
#
# 터미네이터 vica_map 레이아웃에서 motor 칸은 HOLD(사람이 엔터를 눌러야 실행)라
# 자동으로 겹치지는 않는다. 하지만 사람이 먼저 눌렀을 수 있다.
MOTOR_NODES = ('mdrobot_can_keyboard_knob_node',)


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


def blocking_reason(state: MappingState, node_names):
    """Return why mapping must not start now, or None when it may.

    검사 순서가 곧 우선순위다. 가장 위험한 것부터 본다.
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
    return None


def missing_prerequisites(node_names, required):
    """Return the required nodes that are not up yet.

    d455 는 Docker 라, imu 는 자이로 보정에 20초 정지가 필요해서 앱이 띄우지 않는다.
    대신 떠 있는지 확인은 한다 — 없으면 회차가 통째로 무효가 되기 때문이다.
    (docs/cartographer_corridor_mapping.md 3절 점검표)
    """
    bare = {_bare(name) for name in node_names}
    return [name for name in required if _bare(name) not in bare]


def is_motor_up(node_names) -> bool:
    """Tell whether the motor node is already running.

    떠 있으면 매핑 launch 에 start_motor:=false 를 넘겨 두 벌이 되는 것을 막는다.
    """
    bare = {_bare(name) for name in node_names}
    return any(name in bare for name in MOTOR_NODES)


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
