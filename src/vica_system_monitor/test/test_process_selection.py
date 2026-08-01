"""cmdline 패턴에 걸린 후보 중 실제 노드를 고르는 계약.

2026-08-01 Jetson 실측에서 감시 노드가 `imu_adapter` CPU를 0.0 %로 보고했다. 실제는
45 %였다. 같은 패턴에 세 프로세스가 걸렸고 그중 노드가 아닌 것을 골랐다.

    pid 60999  /bin/bash -c ... imu_base_link_adapter ...          0.0 %
    pid 61069  python3 /opt/ros/humble/bin/ros2 run ... adapter    0.1 %
    pid 61085  python3 .../vica_sensor_adapters/imu_base_link_...  45.5 %

셸 래핑은 그 세션의 실행 방식 탓이지만 `ros2 run` 런처는 아니다. 매뉴얼대로 터미널에서
띄워도 남으며 PID가 노드보다 작다.

**핵심 계약은 마지막 절이다 — 후보가 둘 이상이면 그 사실을 호출자가 알 수 있어야 한다.**
조용히 하나 고르는 것이 이 결함의 본질이었다.
"""

from vica_system_monitor.process_cpu import select_probe_pid


NODE = (
    '/usr/bin/python3\x00/home/x/ws/install/pkg/lib/pkg/imu_base_link_adapter'
    '\x00--ros-args\x00'
)
LAUNCHER = (
    '/usr/bin/python3\x00/opt/ros/humble/bin/ros2\x00run\x00pkg'
    '\x00imu_base_link_adapter\x00'
)
SHELL = '/bin/bash\x00-c\x00source /x/y.sh && ros2 run pkg imu_adapter\x00'
ROS2_LAUNCH = (
    '/usr/bin/python3\x00/opt/ros/humble/bin/ros2\x00launch\x00pkg'
    '\x00thing.launch.py\x00'
)
OTHER_NODE = (
    '/usr/bin/python3\x00/home/x/ws/install/pkg/lib/pkg/imu_base_link_adapter'
    '\x00-p\x00x:=1\x00'
)


def test_real_node_is_selected():
    """실제 노드 하나면 그것을 고른다."""
    pid, kept = select_probe_pid([('61085', NODE)])

    assert pid == '61085'
    assert kept == ['61085']


def test_shell_wrapper_is_excluded():
    """셸 래퍼는 노드가 아니다."""
    pid, kept = select_probe_pid([('60999', SHELL), ('61085', NODE)])

    assert pid == '61085'
    assert kept == ['61085']


def test_ros2_run_launcher_is_excluded():
    """`ros2 run` 런처는 노드가 아니다. PID가 더 작아도 고르지 않는다."""
    pid, kept = select_probe_pid([('61069', LAUNCHER), ('61085', NODE)])

    assert pid == '61085'


def test_ros2_launch_launcher_is_excluded():
    """`ros2 launch` 런처도 마찬가지다."""
    pid, kept = select_probe_pid([('100', ROS2_LAUNCH), ('200', NODE)])

    assert pid == '200'


def test_the_observed_jetson_case():
    """2026-08-01 실측 3종 조합에서 45 %짜리 진짜 노드를 골라야 한다."""
    pid, kept = select_probe_pid([
        ('60999', SHELL),
        ('61069', LAUNCHER),
        ('61085', NODE),
    ])

    assert pid == '61085'
    assert kept == ['61085']


def test_path_containing_ros2_is_not_a_launcher():
    """경로에 'ros2'가 들어간 노드를 런처로 오인하면 안 된다.

    이 저장소의 실제 경로가 `.../vica_ros2_ws/install/...`이다.
    """
    cmdline = (
        '/usr/bin/python3\x00'
        '/home/x/vica_ros2_ws/install/pkg/lib/pkg/ekf_node\x00'
    )
    pid, kept = select_probe_pid([('7', cmdline)])

    assert pid == '7'


def test_no_candidates_returns_none():
    """후보가 아예 없으면 고를 것이 없다."""
    assert select_probe_pid([]) == (None, [])


def test_only_launchers_returns_none():
    """런처만 남으면 노드가 없는 것이다. 런처 CPU를 노드 값으로 보고하지 않는다."""
    pid, kept = select_probe_pid([('1', SHELL), ('2', LAUNCHER)])

    assert pid is None
    assert kept == []


def test_multiple_real_candidates_are_all_reported():
    """후보가 둘 이상이면 전부 돌려준다. 호출자가 진단에 남길 수 있어야 한다."""
    pid, kept = select_probe_pid([('900', NODE), ('300', OTHER_NODE)])

    assert len(kept) == 2
    assert set(kept) == {'900', '300'}


def test_multiple_candidates_pick_is_deterministic():
    """후보가 여럿이어도 매 tick 같은 것을 골라야 CPU 델타가 깨지지 않는다."""
    first, _ = select_probe_pid([('900', NODE), ('300', OTHER_NODE)])
    second, _ = select_probe_pid([('300', OTHER_NODE), ('900', NODE)])

    assert first == second


def test_empty_cmdline_is_ignored():
    """커널 스레드는 cmdline이 비어 있다."""
    pid, kept = select_probe_pid([('2', ''), ('61085', NODE)])

    assert pid == '61085'
