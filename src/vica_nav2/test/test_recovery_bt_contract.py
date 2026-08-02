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
# 2026-07-31부터 launch가 실제로 쓰는 트리. 복구가 costmap 초기화뿐이다.
# 지금까지의 완주 성적은 recovery가 흡수해서 나온 것이라, 순수 주행 실력을
# 측정하려고 로봇을 움직이는 복구를 걷어냈다. 근거는 그 파일 상단 주석.
NO_BACKUP_BT = 'vica_navigate_to_pose_no_backup.xml'
CLEARING_ONLY_BT = 'vica_navigate_to_pose_clearing_only.xml'
# 2026-08-01: 측정이 끝나 제품 트리로 되돌렸다. clearing_only는 복구가 실패를
# 흡수하는 것을 막아 순수 주행 실력을 재려고 만든 실험 트리였고, 그 측정에서
# "복구 없이는 장애물 앞에서 못 빠져나온다"가 확인됐다(안내소 -> 방2, 목표
# 3 m 앞 ABORT, 우측 3.75 m가 뚫려 있었는데 우회 못 함).
ACTIVE_BT = NO_BACKUP_BT
# 두 트리 모두 저장소에 남긴다. no_backup은 되돌릴 자리이므로 계속 검사한다.
SHIPPED_BTS = (BT_NAME, CLEARING_ONLY_BT)
# 측정용 트리에서 빼야 하는 것 = behavior_server가 로봇을 '움직이는' 복구.
# ClearEntireCostmap은 로봇을 움직이지 않으므로 여기 없다 -- 의도적으로 남겼다.
MOTION_RECOVERY_NODES = ('Spin', 'Wait', 'BackUp',
                         'DriveOnHeading', 'AssistedTeleop')
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


def _bt_path(name=BT_NAME):
    return _pkg_dir() / 'behavior_trees' / name


def _params():
    path = _pkg_dir() / 'config' / 'nav2_params.yaml'
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _strip_comments(xml_text):
    return re.sub(r'<!--.*?-->', '', xml_text, flags=re.DOTALL)


def _spin_dists():
    """BT에 있는 모든 Spin의 spin_dist를 순서대로 돌려준다.

    속성 순서에 의존하지 않는다. 2026-07-31에 name 속성을 붙이면서
    `<Spin spin_dist=` 고정 패턴이 깨진 적이 있다.
    """
    body = _strip_comments(_bt_path().read_text(encoding='utf-8'))
    out = []
    for tag in re.findall(r'<Spin\b[^>]*>', body):
        m = re.search(r'spin_dist="(-?[0-9.]+)"', tag)
        assert m is not None, f'Spin에 spin_dist가 없다: {tag}'
        out.append(float(m.group(1)))
    return out


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
    """측정용 트리에는 '로봇을 움직이는' 복구가 없어야 한다.

    2026-07-31에 전제를 바꿨다. 풀어야 하는 문제는 "갇힌 데서 빠져나오기"가
    아니라 "못 움직일 자리에 애초에 안 들어가기"인데, recovery가 실패를 전부
    흡수해서 순수 주행 실력이 한 번도 측정되지 않았다. 2026-07-30 12회차
    (155.4 s, 3/3)의 최장정지 19.4 s는 Spin 3회가 만든 시간이고, DWB 무효
    59회·lethal space 6회도 전부 흡수되고 있었다.

    Wait는 실패를 시간으로 덮는다 -- 종전 복구 시간의 79 %가 이것이었다.
    """
    body = _strip_comments(_bt_path(ACTIVE_BT).read_text(encoding='utf-8'))
    assert f'<{node_name}' not in body, (
        f'{ACTIVE_BT}에 {node_name}이 남아 있다. 실패가 흡수되어'
        ' 순수 주행 실력을 측정할 수 없다'
    )


def test_active_bt_keeps_costmap_clearing():
    """ClearEntireCostmap은 남긴다. 로봇을 움직이지 않기 때문이다.

    회전·후진과 달리 자세를 바꾸지 않으므로 '순수 주행 실력' 측정을 오염시키는
    정도가 다르다. 그리고 유령 장애물(p95 43~46 s)이 아직 미해결이라, 이것까지
    빼면 유령 실패와 진짜 실패가 섞여 매 회차가 유령으로 조기 종료될 수 있다.

    대가는 기록해 둔다 -- ComputePathToPose 실패 시 도는 global 초기화는
    "planner가 무엇을 보고 거부했는가"라는 증거를 지운다. 실패 분석 때
    초기화 시각을 먼저 확인할 것.
    """
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
    """복구만 빼고 주행 골격과 재계획 정책은 그대로여야 한다.

    Humble 기성 트리 navigate_w_replanning_only_if_path_becomes_invalid.xml은
    recovery만 빼는 것이 아니라 IsPathValid로 재계획 정책까지 바꾼다(경로가
    무효해질 때만 재계획). 그러면 변수가 둘 섞인다. 현재 트리는 1 Hz 무조건
    재계획을 유지해야 한다.
    """
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

    # 허용되는 변경은 Spin 계열과 BackUp 제거뿐이다.
    #   (1) <BackUp/> 제거      -- 핸들 뒤 사람. 근거는 이 파일 상단.
    #   (2) <Spin spin_dist> 값 -- 2026-07-30 의자 충돌. 근거는 BT 안 주석.
    #   (3) <Spin> 우회전 추가  -- 2026-07-31. 좌만 있으면 막힌 쪽으로만 돈다.
    #                              근거는 test_recovery_can_spin_both_ways.
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
    dists = _spin_dists()
    assert dists, 'BT에서 Spin spin_dist를 찾을 수 없다'
    for spin_dist in dists:
        mag = abs(spin_dist)
        assert mag <= MAX_SPIN_DIST_RAD, (
            f'spin_dist {spin_dist} rad({math.degrees(mag):.0f}도)가'
            f' 상한 {MAX_SPIN_DIST_RAD} rad'
            f'({math.degrees(MAX_SPIN_DIST_RAD):.0f}도)를 넘는다.'
            ' 253 밴드에서 회전하면 후방 0.675 m가 쓸린다'
        )
        # 0이면 복구 수단을 잃는다. 누적을 쓰려면 한 번에 최소한은 돌아야 한다.
        assert mag >= 0.15, (
            f'spin_dist {spin_dist} rad이 너무 작아 자세 전환에 기여하지 못한다'
        )


def test_recovery_can_spin_both_ways():
    """복구 회전은 좌·우 두 방향이 다 있어야 한다.

    nav2 Spin은 BT가 준 부호대로만 돈다 -- onCycleUpdate가
    copysign(vel, cmd_yaw)로 방향을 정한다(libnav2_spin_behavior.so 0x1e4bc의
    bit v8.8b, v0.8b, v2.8b). "빈 쪽으로 돌기" 판단은 없다.

    2026-07-30 12회차 화장실 구간에서 양수 하나뿐이던 Spin이 막힌 쪽으로만
    돌았다. 첫 Spin이 yaw 64.4 -> 94.4도로 돌면서 footprint 안 LETHAL 겹침이
    1 -> 4 -> 6개로 늘었고, 재시도 3회 모두 같은 방향으로 즉시 Collision Ahead가
    났다. 19.4초 뒤 탈출한 첫 궤적은 wz -0.400, 곧 반대 방향이었다.

    부호가 반대인 Spin이 하나라도 없으면 그 상황이 그대로 재발한다.
    """
    dists = _spin_dists()
    assert any(d > 0 for d in dists), (
        f'좌회전(양수) Spin이 없다: {dists}'
    )
    assert any(d < 0 for d in dists), (
        f'우회전(음수) Spin이 없다. 좌가 막힌 자리에서 대안이 사라진다: {dists}'
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
    assert value.endswith(f'behavior_trees/{ACTIVE_BT}'), (
        f'BT 경로가 활성 트리({ACTIVE_BT})를 가리키지 않는다: {value}'
    )
    # 나머지 하나는 되돌릴 자리로 남아 있어야 한다.
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
