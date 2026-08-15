import importlib.util
import re
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node, SetRemap


def _load_launch_module():
    launch_path = (
        Path(__file__).parents[1] / 'launch' / 'nav2_map_test.launch.py'
    )
    spec = importlib.util.spec_from_file_location(
        'vica_nav2_map_test_launch',
        launch_path,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _remap_pairs(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    launch_description = _load_launch_module().generate_launch_description()
    groups = [
        entity
        for entity in launch_description.entities
        if isinstance(entity, GroupAction)
    ]

    assert len(groups) == 1

    actions = groups[0].get_sub_entities()
    remaps = [action for action in actions if isinstance(action, SetRemap)]
    includes = [
        action
        for action in actions
        if isinstance(action, IncludeLaunchDescription)
    ]

    assert len(includes) == 1

    context = LaunchContext()
    return {
        perform_substitutions(context, remap.src):
            perform_substitutions(context, remap.dst)
        for remap in remaps
    }


def _group_nodes(monkeypatch, tmp_path):
    """GroupAction 안에 launch 가 직접 띄우는 Node 를 이름으로 모은다."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    launch_description = _load_launch_module().generate_launch_description()
    groups = [
        entity
        for entity in launch_description.entities
        if isinstance(entity, GroupAction)
    ]
    assert len(groups) == 1

    context = LaunchContext()

    def text(value):
        # launch_ros 는 리터럴 문자열을 Substitution 으로 감싸지 않고 그대로
        # 둔다. 둘 다 올 수 있으므로 여기서 흡수한다.
        if isinstance(value, str):
            return value
        if isinstance(value, (list, tuple)):
            return ''.join(text(item) for item in value)
        return perform_substitutions(context, [value])

    nodes = {}
    for action in groups[0].get_sub_entities():
        if not isinstance(action, Node):
            continue
        nodes[text(action.node_executable)] = text(action.node_package)
    return nodes


def _params():
    config = (
        Path(__file__).parents[1] / 'config' / 'nav2_params.yaml'
    ).read_text(encoding='utf-8')
    return yaml.safe_load(config)


def _smoother_output_topic():
    """velocity_smoother 가 '실제로' 발행하는 토픽 이름을 알아낸다.

    노드 기본 출력은 cmd_vel_smoothed 지만 그 이름으로 나가지 않는다.
    nav2_bringup 이 띄우면서 remap 을 걸기 때문이다.

        navigation_launch.py:183
        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]

    즉 입력이 cmd_vel_nav, 출력이 cmd_vel 이다. 이 값을 상수로 박아 두면
    nav2 를 올릴 때 조용히 어긋난다. 그래서 그쪽 launch 파일에서 직접 읽는다.

    2026-08-15 에 이 계약이 'cmd_vel_smoothed' 를 상수로 기대하고 있었고,
    그 값을 monitor 입력에 넣었더니 시험은 통과했는데 실기에서 배선이 끊겼다.
    controller 는 /cmd_vel_nav 로 23.8 Hz 를 내는데 monitor 는 아무것도 받지
    못했고, /cmd_vel_req 와 /cmd_vel_safe 가 비어 로봇이 못 움직였다.
    앱에는 '주행 중' 으로 보였다.
    """
    launch_file = Path(
        '/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py'
    )
    if not launch_file.is_file():
        pytest.skip(f'nav2_bringup launch 없음: {launch_file}')
    text = launch_file.read_text(encoding='utf-8')
    # ('cmd_vel_smoothed', '<출력이름>') 을 찾는다.
    match = re.search(
        r"\(\s*['\"]cmd_vel_smoothed['\"]\s*,\s*['\"]([^'\"]+)['\"]\s*\)", text
    )
    if not match:
        # remap 이 없으면 노드 기본 이름 그대로 나간다.
        return 'cmd_vel_smoothed'
    return match.group(1)


def test_velocity_smoother_output_reaches_safety_through_collision_monitor(
    monkeypatch,
    tmp_path,
):
    """속도 명령의 마지막 구간이 Collision Monitor 를 거쳐야 한다.

    2026-08-13 이전 배선은 이랬다.

        velocity_smoother --cmd_vel_smoothed--> [SetRemap] --> /cmd_vel_req

    그때 이 시험은 그 SetRemap 이 있는지만 봤다. 지금은 그 자리에
    collision_monitor 가 들어가 /scan 원본으로 마지막 판정을 한다.

        velocity_smoother --cmd_vel_smoothed--> collision_monitor --> /cmd_vel_req

    **SetRemap 이 남아 있으면 안 된다.** 남으면 velocity_smoother 가 monitor 를
    건너뛰고 /cmd_vel_req 로 직접 발행해 감시가 통째로 무의미해진다. 그런데
    노드는 그대로 떠 있어서 로그만 봐서는 알 수 없다 — 이 시험이 그 조용한
    실패를 막는다.

    발행자 수로는 구별되지 않는다는 점도 기록해 둔다. /cmd_vel_req 발행자는
    velocity_smoother 1 + behavior_server 5 = 6 이었는데, monitor 를 끼워도
    collision_monitor 1 + behavior_server 5 = 6 으로 같다. 확인하려면
    /cmd_vel_smoothed 의 구독자가 1(monitor)인지를 봐야 한다.
    """
    pairs = _remap_pairs(monkeypatch, tmp_path)
    assert 'cmd_vel_smoothed' not in pairs, (
        'cmd_vel_smoothed 에 remap 이 남아 있다. velocity_smoother 가'
        ' collision_monitor 를 건너뛰고 /cmd_vel_req 로 직접 발행하게 된다'
    )

    nodes = _group_nodes(monkeypatch, tmp_path)
    assert nodes.get('collision_monitor') == 'nav2_collision_monitor', (
        f'launch 가 collision_monitor 를 띄우지 않는다: {nodes}'
    )

    expected_in = _smoother_output_topic()
    monitor = _params()['collision_monitor']['ros__parameters']
    assert monitor['cmd_vel_in_topic'].lstrip('/') == expected_in.lstrip('/'), (
        f'monitor 입력 {monitor["cmd_vel_in_topic"]} 이 velocity_smoother 의 실제'
        f' 출력 /{expected_in} 과 다르다. 다른 토픽을 보면 아무것도 받지 못하고,'
        ' 그러면 /cmd_vel_req 가 조용히 비어 로봇이 한 발도 못 움직인다'
        ' (2026-08-15 실기에서 실제로 발생)'
    )
    assert monitor['cmd_vel_out_topic'] == '/cmd_vel_req', (
        f'monitor 출력 {monitor["cmd_vel_out_topic"]} 이 Safety 입력이 아니다.'
        ' CLAUDE.md 의 /cmd_vel_req 계약이 깨진다'
    )


def test_collision_monitor_is_lifecycle_managed(monkeypatch, tmp_path):
    """관리자가 없으면 노드가 unconfigured 로 남아 아무 일도 하지 않는다.

    nav2_bringup 의 lifecycle_nodes 목록(navigation_launch.py 43행)에
    collision_monitor 가 없다. 그래서 별도 lifecycle_manager 를 붙인다.
    빠뜨리면 노드는 떠 있고 로그도 정상인데 속도 명령을 한 번도 통과시키지
    않는다 — /cmd_vel_req 가 조용히 끊긴다.
    """
    nodes = _group_nodes(monkeypatch, tmp_path)
    assert nodes.get('lifecycle_manager') == 'nav2_lifecycle_manager', (
        f'collision_monitor 를 올려 줄 lifecycle_manager 가 없다: {nodes}'
    )


def test_collision_monitor_has_no_rear_polygon():
    """후방 감시 영역은 절대 두지 않는다.

    사용자가 손잡이를 잡고 로봇 뒤에 선다. 후방 폴리곤을 두면 사람이 상시로
    검출되어 로봇이 영영 못 움직인다. docs/nav2_backlog.md §9 에
    "다시 제안하지 말 것"으로 등재돼 있다(devlog/2026-07-30.md:152).

    points 는 [x1, y1, x2, y2, ...] 평면 배열이고 +x 가 진행 방향이다.
    padded footprint 앞단이 +0.355 m 이므로 모든 x 가 그보다 커야 차체와도
    겹치지 않는다.
    """
    monitor = _params()['collision_monitor']['ros__parameters']
    front_edge = 0.355  # padded footprint 앞단

    for name in monitor['polygons']:
        points = monitor[name]['points']
        xs = points[0::2]
        assert min(xs) >= front_edge, (
            f'{name} 이 x={min(xs)} 까지 뻗어 차체 앞단 {front_edge} 안으로'
            ' 들어온다. 뒤쪽이면 손잡이를 잡은 사용자가 상시 검출된다'
        )


def test_recovery_behaviors_reach_the_safety_path(monkeypatch, tmp_path):
    """behavior_server의 속도 명령도 /cmd_vel_req로 나가야 한다.

    Nav2 humble navigation_launch.py는 controller_server에만
    ('cmd_vel', 'cmd_vel_nav')를 준다. behavior_server는 remap을 못 받아
    /cmd_vel로 발행하는데 VICA에는 그 토픽 구독자가 없다.

    2026-07-29 실측: /cmd_vel 발행자 5(전부 behavior_server), 구독자 0.
    Spin·BackUp이 한 번도 로봇을 움직이지 못했고, planner가 실패하면 복구
    6회가 전부 헛돌아 즉시 Goal failed가 됐다. 갇히면 빠져나올 수단이 없었다.
    """
    pairs = _remap_pairs(monkeypatch, tmp_path)
    assert pairs['behavior_server:cmd_vel'] == '/cmd_vel_req'


def test_no_unscoped_cmd_vel_remap_hijacks_the_controller(
    monkeypatch,
    tmp_path,
):
    """`cmd_vel` 전역 remap은 절대 넣으면 안 된다.

    launch_ros node.py(468~477행)가 global remap을 node-level보다 **먼저**
    붙이고 rcl은 첫 일치 규칙을 쓴다. 따라서 접두사 없는 cmd_vel remap은
    controller_server의 cmd_vel:=cmd_vel_nav를 덮어써 velocity_smoother를
    건너뛰고, velocity_smoother는 자기 출력을 구독하는 자기루프가 된다.
    노드 지정 문법(node_name:from:=to)만 허용한다.
    """
    pairs = _remap_pairs(monkeypatch, tmp_path)
    assert 'cmd_vel' not in pairs
    for src in pairs:
        if src.endswith(':cmd_vel'):
            assert ':' in src, f'{src}는 노드 지정 remap이어야 한다'
