"""Contract tests for the mapping bringup launch."""

from pathlib import Path

LAUNCH = (
    Path(__file__).resolve().parent.parent / 'launch'
    / 'vica_mapping_bringup.launch.py'
)


def source() -> str:
    return LAUNCH.read_text(encoding='utf-8')


def test_launch_file_exists():
    assert LAUNCH.is_file(), f'{LAUNCH} 가 없습니다.'


def test_nav2_is_never_included():
    """SLAM 과 Nav2 는 둘 다 wheel_ekf 를 include 해서 /odom 이 이중 발행된다."""
    text = source()
    for forbidden in ('nav2_map_test', 'vica_nav2', 'navigation_launch'):
        assert forbidden not in text, f'{forbidden} 이 들어 있습니다.'


def test_safety_is_never_included():
    """Safety 칸은 터미네이터에서 AUTO 라 창을 띄우는 순간 이미 떠 있다."""
    assert 'safety_bringup.launch.py' not in source()


def test_motor_is_included_but_switchable():
    """엔코더 피드백을 요청하는 쪽이 motor node 라 없으면 /wheel/odom 이 안 나온다."""
    text = source()
    assert 'motor_bringup.launch.py' in text
    assert "'start_motor'" in text, 'start_motor 인자로 끌 수 있어야 합니다.'
    assert 'IfCondition(start_motor)' in text


def test_slam_and_preview_are_included():
    text = source()
    assert 'vica_slam_bringup.launch.py' in text
    assert 'map_preview_node' in text


def test_camera_and_imu_are_not_launched_here():
    """d455 는 Docker, imu 는 20초 정지가 필요해 사람이 한다."""
    text = source()
    assert 'realsense' not in text.lower()
    assert 'imu_base_link_adapter' not in text


def test_teleop_is_not_launched_here():
    """키보드 입력을 받는 대화형 프로세스라 띄워도 사람이 그 창에 키를 눌러야 한다."""
    assert 'teleop_twist_keyboard' not in source()
