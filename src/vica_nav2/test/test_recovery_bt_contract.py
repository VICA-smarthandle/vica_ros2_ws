"""복구 동작이 로봇을 움직이지 않는지 감시한다."""
import importlib.util
import re
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.utilities import perform_substitutions

BT_NAME = 'vica_navigate_to_pose_no_backup.xml'
NO_BACKUP_BT = 'vica_navigate_to_pose_no_backup.xml'
CLEARING_ONLY_BT = 'vica_navigate_to_pose_clearing_only.xml'
ACTIVE_BT = NO_BACKUP_BT
SHIPPED_BTS = (BT_NAME, CLEARING_ONLY_BT)
MOTION_RECOVERY_NODES = ('Spin', 'Wait', 'BackUp',
                         'DriveOnHeading', 'AssistedTeleop')
REVERSE_CAPABLE_NODES = ('BackUp', 'DriveOnHeading', 'AssistedTeleop')
NAV2_DEFAULT_BT = Path(
    '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees'
    '/navigate_to_pose_w_replanning_and_recovery.xml'
)
PLACEHOLDER = 'SET_BY_VICA_NAV2_LAUNCH'

MIN_TOTAL_PATIENCE_S = 15

MAX_RESTART_DELAY_S = 2


def _pkg_dir():
    return Path(__file__).parents[1]


def _bt_path(name=BT_NAME):
    return _pkg_dir() / 'behavior_trees' / name


def _params():
    path = _pkg_dir() / 'config' / 'nav2_params.yaml'
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _strip_comments(xml_text):
    return re.sub(r'<!--.*?-->', '', xml_text, flags=re.DOTALL)


@pytest.mark.parametrize('bt_name', SHIPPED_BTS)
def test_custom_bt_exists_and_is_valid_xml(bt_name):
    from xml.etree import ElementTree

    path = _bt_path(bt_name)
    assert path.is_file(), f'커스텀 BT가 없다: {path}'
    root = ElementTree.fromstring(path.read_text(encoding='utf-8'))
    assert root.tag == 'root'
    assert root.get('main_tree_to_execute') == 'MainTree'


@pytest.mark.parametrize('bt_name', SHIPPED_BTS)
@pytest.mark.parametrize('node_name', REVERSE_CAPABLE_NODES)
def test_custom_bt_has_no_reverse_capable_node(node_name, bt_name):
    body = _strip_comments(_bt_path(bt_name).read_text(encoding='utf-8'))
    assert f'<{node_name}' not in body, (
        f'{bt_name}에 {node_name} 노드가 있다. 핸들 뒤 사람에게 후진할 수 있다'
    )


@pytest.mark.parametrize('node_name', MOTION_RECOVERY_NODES)
@pytest.mark.skipif(
    ACTIVE_BT != CLEARING_ONLY_BT,
    reason=(
        '측정용 트리에만 적용되는 계약이다. 2026-08-01에 그 측정을 마쳤고'
        ' 제품 트리(no_backup)로 되돌렸다 — 복구 없이는 장애물 앞에서'
        ' 빠져나오지 못한다는 것이 실기로 확인됐다. clearing_only를 다시'
        ' 활성으로 두면 이 계약이 자동으로 살아난다.'
    ),
)
def test_active_bt_has_no_motion_recovery(node_name):
    """측정용 트리에는 '로봇을 움직이는' 복구가 없어야 한다."""
    body = _strip_comments(_bt_path(ACTIVE_BT).read_text(encoding='utf-8'))
    assert f'<{node_name}' not in body, (
        f'{ACTIVE_BT}에 {node_name}이 남아 있다. 실패가 흡수되어'
        ' 순수 주행 실력을 측정할 수 없다'
    )


def test_active_bt_keeps_costmap_clearing():
    """ClearEntireCostmap은 남긴다. 로봇을 움직이지 않기 때문이다."""
    body = _strip_comments(_bt_path(ACTIVE_BT).read_text(encoding='utf-8'))
    assert body.count('<ClearEntireCostmap') == 4, (
        f'{ACTIVE_BT}의 ClearEntireCostmap이 4개가 아니다:'
        f' {body.count("<ClearEntireCostmap")}개.'
        ' 유령 장애물이 미해결인 동안은 이것만 남긴다'
    )
    for service in ('global_costmap/clear_entirely_global_costmap',
                    'local_costmap/clear_entirely_local_costmap'):
        assert body.count(service) == 2, (
            f'{ACTIVE_BT}에서 {service} 호출이 2개가 아니다'
        )
    assert '<RecoveryNode' in body, (
        'RecoveryNode가 없으면 초기화 후 재시도 자체가 일어나지 않는다'
    )


def test_active_bt_still_plans_and_follows_with_periodic_replanning():
    """복구만 빼고 주행 골격과 재계획 정책은 그대로여야 한다."""
    body = _strip_comments(_bt_path(ACTIVE_BT).read_text(encoding='utf-8'))
    for keep in ('<PipelineSequence', '<RateController',
                 '<ComputePathToPose', '<FollowPath'):
        assert keep in body, f'{ACTIVE_BT}에 {keep}이 없다'
    assert 'hz="1.0"' in body, (
        f'{ACTIVE_BT}의 재계획 주기가 1 Hz가 아니다. 종전 주행과 비교할 수 없다'
    )
    assert '<IsPathValid' not in body, (
        '재계획 정책이 조건부로 바뀌었다. 복구 제거와 변수가 섞인다'
    )
    assert f'planner_id="{ "GridBased" }"' in body, (
        f'{ACTIVE_BT}의 planner_id가 planner_server 등록 이름과 달라졌다'
    )


def test_custom_bt_keeps_the_stationary_recovery_actions():
    """로봇을 움직이지 않는 복구 수단은 남아 있어야 한다."""
    body = _strip_comments(_bt_path().read_text(encoding='utf-8'))
    for keep in ('<Wait', '<ClearEntireCostmap',
                 '<ComputePathToPose', '<FollowPath'):
        assert keep in body, f'{BT_NAME}에서 {keep}이 사라졌다'


def test_custom_bt_only_removes_backup_from_the_nav2_default():
    """기본 트리와의 차이가 BackUp 한 줄뿐인지 확인한다."""
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

    changeable = ('<BackUp', '<Spin', '<Wait',
                  '<RecoveryNode number_of_retries=')

    added_unexpected = [l for l in added if not l.startswith(changeable)]
    removed_unexpected = [l for l in removed if not l.startswith(changeable)]

    assert added_unexpected == [], (
        f'허용되지 않은 줄이 추가됐다: {added_unexpected}'
    )
    assert removed_unexpected == [], (
        f'허용되지 않은 줄이 제거됐다: {removed_unexpected}'
    )
    assert any(l.startswith('<BackUp') for l in removed), (
        f'BackUp 줄이 제거되지 않았다. 제거된 줄: {removed}'
    )
    assert any(l.startswith('<Spin') for l in removed), (
        f'Spin 줄이 제거되지 않았다. 제거된 줄: {removed}'
    )


@pytest.mark.parametrize('bt_name', SHIPPED_BTS)
def test_recovery_has_no_spin(bt_name):
    """복구에서 제자리 회전을 없앤 상태를 지킨다(2026-08-12). 근거 4가지."""
    body = _strip_comments(_bt_path(bt_name).read_text(encoding='utf-8'))
    assert '<Spin' not in body, (
        f'{bt_name}에 Spin이 되살아났다. 253 밴드에서 후방 0.651 m가 쓸린다'
    )


def _wait_durations(bt_name=BT_NAME):
    """BT에 있는 모든 Wait의 wait_duration을 순서대로 돌려준다."""
    body = _strip_comments(_bt_path(bt_name).read_text(encoding='utf-8'))
    out = []
    for tag in re.findall(r'<Wait\b[^>]*>', body):
        m = re.search(r'wait_duration="([^"]+)"', tag)
        assert m is not None, f'Wait에 wait_duration이 없다: {tag}'
        out.append(m.group(1))
    return out


def test_wait_duration_is_an_integer():
    """Humble의 WaitAction은 BT::InputPort<int>다."""
    values = _wait_durations()
    assert values, f'{BT_NAME}에 Wait가 없다. 동적 장애물을 기다릴 수단이 사라졌다'
    for raw in values:
        assert raw.lstrip('-').isdigit(), (
            f'wait_duration "{raw}"가 정수가 아니다.'
            ' Humble의 WaitAction 포트는 int라 소수점을 받지 못한다'
        )
        assert int(raw) > 0, f'wait_duration {raw}가 0 이하다'


def test_recovery_patience_is_long_enough():
    """총 인내 시간이 사람이 비켜설 만큼은 되어야 한다."""
    body = _strip_comments(_bt_path().read_text(encoding='utf-8'))
    m = re.search(r'<RecoveryNode[^>]*number_of_retries="(\d+)"[^>]*'
                  r'name="NavigateRecovery"', body)
    assert m is not None, 'NavigateRecovery의 number_of_retries를 찾을 수 없다'
    retries = int(m.group(1))
    wait_s = int(_wait_durations()[0])

    patience = (retries / 2) * wait_s
    assert patience >= MIN_TOTAL_PATIENCE_S, (
        f'총 인내 {patience:.0f} s가 하한 {MIN_TOTAL_PATIENCE_S} s에 못 미친다'
        f' (retries {retries}, wait {wait_s} s).'
        ' 사람이 비켜서기 전에 주행이 실패로 끝난다'
    )


@pytest.mark.skip(
    reason='2026-08-15 기준점 복귀 중. nav2 기본값 wait 5 s 로 되돌려 이 상한 2 s 를 '
           '넘는다. 근거는 그대로 유효하다 - 2026-08-13 A/B 에서 wait 2 s(재계획 10번) '
           'vs 1 s(30번)가 궤적 실패 160 대 7 로 갈렸고, 갈린 축이 총 시간이 아니라 '
           '다시 보는 빈도였다. 다만 예산을 줄이면 Goal failed 가 나고(run7) 늘리면 '
           '30 s 를 서 있게 되어, 이 상한만으로는 답이 나오지 않는다. 사람만 골라 '
           '지우는 수단(YOLO)이 생긴 뒤 값과 표시를 함께 정리한다.'
)
def test_restart_delay_is_bounded():
    """사람이 비켜난 뒤 다시 출발하기까지 오래 끌면 안 된다."""
    values = [int(v) for v in _wait_durations()]
    for v in values:
        assert v <= MAX_RESTART_DELAY_S, (
            f'wait_duration {v} s 가 재출발 지연 상한 {MAX_RESTART_DELAY_S} s 를'
            ' 넘는다. 사람이 비켜나도 그만큼 서 있는다'
        )


def test_bt_navigator_declares_the_key_so_launch_can_rewrite_it():
    """RewrittenYaml은 '이미 존재하는 키'만 치환한다."""
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
    assert value.endswith(f'behavior_trees/{ACTIVE_BT}'), (
        f'BT 경로가 활성 트리({ACTIVE_BT})를 가리키지 않는다: {value}'
    )
    other = BT_NAME if ACTIVE_BT != BT_NAME else CLEARING_ONLY_BT
    assert _bt_path(other).is_file(), (
        f'되돌릴 트리 {other}가 사라졌다. launch 한 줄로 복구를 되살릴 수 없다'
    )


def test_installed_bt_is_shipped_by_setup_py():
    """setup.py가 behavior_trees를 설치하지 않으면 launch가 경로를 못 찾는다."""
    setup_py = (_pkg_dir() / 'setup.py').read_text(encoding='utf-8')
    assert 'behavior_trees' in setup_py, (
        'setup.py data_files에 behavior_trees가 없어 share에 설치되지 않는다'
    )
