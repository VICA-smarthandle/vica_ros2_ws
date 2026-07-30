"""복구 동작에 후진이 없는지 감시한다.

실주행에서는 핸들 뒤에 사람이 따라온다(guideline/vica_scenario.md). 그런데
2026-07-30 Hybrid 주행에서 로봇이 실제로 0.30 m 후진했다:

    /cmd_vel_safe 7346개 중 후진 173개, vx 정확히 -0.0500 고정, 연속 5.87 s
    behavior_server: "backup completed successfully"
    5.87 s x 0.05 m/s = 0.29 m  ->  기본 BT의 backup_dist="0.30"과 일치

planner를 DUBIN으로 바꾼 것으로는 막히지 않는다. DUBIN 경로에 후진 primitive가
없는 것과 별개로, BackUp은 planner를 거치지 않고 behavior_server가 직접
속도를 발행하기 때문이다. 그래서 BT에서 노드 자체를 지웠다.

behavior_plugins에서 "backup"만 빼는 방법은 쓰지 않는다.
BtActionNode::createActionClient가 액션 서버를 못 찾으면 예외를 던져
(throw std::runtime_error) BT 생성이 실패하고 주행 전체가 죽는다.
"""
import importlib.util
import math
import re
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.utilities import perform_substitutions

BT_NAME = 'vica_navigate_to_pose_no_backup.xml'
# 후진을 만들 수 있는 BT 노드. DriveOnHeading은 음수 거리를 받으면 후진한다.
REVERSE_CAPABLE_NODES = ('BackUp', 'DriveOnHeading', 'AssistedTeleop')
NAV2_DEFAULT_BT = Path(
    '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees'
    '/navigate_to_pose_w_replanning_and_recovery.xml'
)
PLACEHOLDER = 'SET_BY_VICA_NAV2_LAUNCH'

# Spin 회전각 상한 [rad]. nav2 기본값은 1.57(90도)이다.
# 0.35 rad = 20도. 근거는 test_spin_angle_is_small_enough_to_limit_the_swept_arc.
MAX_SPIN_DIST_RAD = 0.35


def _pkg_dir():
    return Path(__file__).parents[1]


def _bt_path():
    return _pkg_dir() / 'behavior_trees' / BT_NAME


def _params():
    path = _pkg_dir() / 'config' / 'nav2_params.yaml'
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _strip_comments(xml_text):
    return re.sub(r'<!--.*?-->', '', xml_text, flags=re.DOTALL)


def test_custom_bt_exists_and_is_valid_xml():
    from xml.etree import ElementTree

    path = _bt_path()
    assert path.is_file(), f'커스텀 BT가 없다: {path}'
    root = ElementTree.fromstring(path.read_text(encoding='utf-8'))
    assert root.tag == 'root'
    assert root.get('main_tree_to_execute') == 'MainTree'


@pytest.mark.parametrize('node_name', REVERSE_CAPABLE_NODES)
def test_custom_bt_has_no_reverse_capable_node(node_name):
    body = _strip_comments(_bt_path().read_text(encoding='utf-8'))
    assert f'<{node_name}' not in body, (
        f'{BT_NAME}에 {node_name} 노드가 있다. 핸들 뒤 사람에게 후진할 수 있다'
    )


def test_custom_bt_keeps_the_forward_only_recovery_actions():
    """후진만 빼고 나머지 복구 수단은 남아 있어야 한다.

    복구 수단을 다 없애면 한 번 갇혔을 때 빠져나올 방법이 사라진다.
    RoundRobin은 ClearingActions -> Spin -> Wait를 순환해야 한다.
    """
    body = _strip_comments(_bt_path().read_text(encoding='utf-8'))
    for keep in ('<Spin', '<Wait', '<ClearEntireCostmap',
                 '<ComputePathToPose', '<FollowPath'):
        assert keep in body, f'{BT_NAME}에서 {keep}이 사라졌다'


def test_custom_bt_only_removes_backup_from_the_nav2_default():
    """기본 트리와의 차이가 BackUp 한 줄뿐인지 확인한다.

    2026-07-28에 커스텀 BT로 SmoothPath를 넣었다가 실주행이 악화되어
    되돌린 이력이 있다(측면 여유 중앙값 0.476 -> 0.373 m). 커스텀 BT는
    최소 변경만 유지한다.
    """
    if not NAV2_DEFAULT_BT.is_file():
        pytest.skip(f'nav2 기본 BT 없음: {NAV2_DEFAULT_BT}')

    def lines(text):
        return [
            line.strip()
            for line in _strip_comments(text).splitlines()
            if line.strip()
        ]

    default = lines(NAV2_DEFAULT_BT.read_text(encoding='utf-8'))
    ours = lines(_bt_path().read_text(encoding='utf-8'))

    removed = [line for line in default if line not in ours]
    added = [line for line in ours if line not in default]

    # 허용되는 변경은 둘뿐이다.
    #   (1) <BackUp/> 제거      -- 핸들 뒤 사람. 근거는 이 파일 상단.
    #   (2) <Spin spin_dist> 값 -- 2026-07-30 의자 충돌. 근거는 BT 안 주석.
    # 그 밖에 줄이 추가되거나 제거되면 실패시킨다. 2026-07-28에 커스텀 BT로
    # SmoothPath를 넣었다가 실주행이 악화되어 되돌린 이력이 있다.
    added_non_spin = [l for l in added if not l.startswith('<Spin')]
    removed_non_backup = [
        l for l in removed
        if not (l.startswith('<BackUp') or l.startswith('<Spin'))
    ]
    assert added_non_spin == [], (
        f'BackUp 제거와 Spin 각 조정 외의 줄이 추가됐다: {added_non_spin}'
    )
    assert removed_non_backup == [], (
        f'BackUp 제거와 Spin 각 조정 외의 줄이 제거됐다: {removed_non_backup}'
    )
    assert any(l.startswith('<BackUp') for l in removed), (
        f'BackUp 줄이 제거되지 않았다. 제거된 줄: {removed}'
    )


def test_spin_angle_is_small_enough_to_limit_the_swept_arc():
    """253 밴드에서 Spin이 도는 것을 파라미터로는 막을 수 없으므로 각을 줄인다.

    Spin이 쓰는 CostmapTopicCollisionChecker::isCollisionFree는 254(LETHAL)에서만
    거부하고 253(INSCRIBED)은 통과시킨다. 헤더에 임계값 인자가 없어 설정으로
    바꿀 수 없다(costmap_topic_collision_checker.hpp: isCollisionFree(pose, fetch)).
    253은 '벽에서 내접반경 0.277 m 이내'이고 그 자리에서 회전하면 후방 꼭짓점이
    반경 0.675 m를 쓸어 거의 확실히 닿는다 -- 2026-07-30에 실제로 핸들이 의자에
    부딪혔다.

    inflation_radius 0.35 -> 0.40으로 253 진입을 줄여봤으나 회전 중 253 접촉
    샘플이 18개 -> 18개로 동일했다. 진입 억제로는 막지 못한다.

    각을 줄이면 쓸리는 각범위가 줄어든다. 기본값 1.57(90도)은 1/4바퀴다.
    RoundRobin이 복구를 순환하므로 작은 각도 재시도마다 누적되어 복구 능력을
    잃지 않는다.
    """
    import re
    body = _strip_comments(_bt_path().read_text(encoding='utf-8'))
    m = re.search(r'<Spin\s+spin_dist="([0-9.]+)"', body)
    assert m is not None, 'BT에서 Spin spin_dist를 찾을 수 없다'
    spin_dist = float(m.group(1))
    assert spin_dist <= MAX_SPIN_DIST_RAD, (
        f'spin_dist {spin_dist} rad({math.degrees(spin_dist):.0f}도)가'
        f' 상한 {MAX_SPIN_DIST_RAD} rad({math.degrees(MAX_SPIN_DIST_RAD):.0f}도)를'
        ' 넘는다. 253 밴드에서 회전하면 후방 0.675 m가 쓸린다'
    )
    # 0이면 복구 수단을 잃는다. RoundRobin 누적을 쓰려면 한 번에 최소한은 돌아야 한다.
    assert spin_dist >= 0.15, (
        f'spin_dist {spin_dist} rad이 너무 작아 자세 전환에 기여하지 못한다'
    )


def test_bt_navigator_declares_the_key_so_launch_can_rewrite_it():
    """RewrittenYaml은 '이미 존재하는 키'만 치환한다.

    키가 없으면 launch가 조용히 아무 일도 하지 않고 nav2 기본 트리가 쓰인다.
    그러면 BackUp이 되살아나는데 아무 경고도 나오지 않는다.
    """
    bt = _params()['bt_navigator']['ros__parameters']
    assert 'default_nav_to_pose_bt_xml' in bt, (
        'default_nav_to_pose_bt_xml 키가 없어 launch의 RewrittenYaml이'
        ' 아무 일도 하지 않는다 -> nav2 기본 트리(BackUp 포함)가 쓰인다'
    )
    assert bt['default_nav_to_pose_bt_xml'] == PLACEHOLDER, (
        'yaml에는 자리표시자만 둔다. 절대경로를 박으면 다른 장비에서 깨진다'
    )


def test_launch_rewrites_the_bt_path_to_the_installed_tree(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))

    launch_path = _pkg_dir() / 'launch' / 'nav2_map_test.launch.py'
    spec = importlib.util.spec_from_file_location('vica_bt_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    description = module.generate_launch_description()
    groups = [e for e in description.entities if isinstance(e, GroupAction)]
    assert len(groups) == 1
    includes = [
        a for a in groups[0].get_sub_entities()
        if isinstance(a, IncludeLaunchDescription)
    ]
    assert len(includes) == 1

    context = LaunchContext()
    args = dict(includes[0].launch_arguments)
    assert 'params_file' in args

    rewritten = args['params_file']
    rewrites = getattr(rewritten, '_RewrittenYaml__param_rewrites', None)
    assert rewrites is not None, (
        'params_file이 RewrittenYaml이 아니다. BT 경로를 넣을 자리가 없다'
    )
    assert 'default_nav_to_pose_bt_xml' in rewrites

    value = rewrites['default_nav_to_pose_bt_xml']
    if not isinstance(value, str):
        value = perform_substitutions(context, value)
    assert value.endswith(f'behavior_trees/{BT_NAME}'), (
        f'BT 경로가 커스텀 트리를 가리키지 않는다: {value}'
    )


def test_installed_bt_is_shipped_by_setup_py():
    """setup.py가 behavior_trees를 설치하지 않으면 launch가 경로를 못 찾는다."""
    setup_py = (_pkg_dir() / 'setup.py').read_text(encoding='utf-8')
    assert 'behavior_trees' in setup_py, (
        'setup.py data_files에 behavior_trees가 없어 share에 설치되지 않는다'
    )
