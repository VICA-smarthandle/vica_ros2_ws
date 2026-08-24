"""Contract tests for the mapping bringup launch.

launch 를 실제로 실행하지 않고 파일을 읽어 규약만 확인한다. 여기서 잠그는 것은
"무엇을 띄우는가"가 아니라 **"무엇을 띄우면 안 되는가"**다.
"""

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
    """SLAM 과 Nav2 는 둘 다 wheel_ekf 를 include 해서 /odom 이 이중 발행된다.

    vica_map 프로파일 설명: "nav2 는 뺀 것이 아니라 넣으면 안 되는 것이다".
    여기에 nav2 가 들어오면 mapping_supervisor_node 의 중복 검사가 자기가 만든
    충돌을 잡게 된다.
    """
    text = source()
    for forbidden in ('nav2_map_test', 'vica_nav2', 'navigation_launch'):
        assert forbidden not in text, f'{forbidden} 이 들어 있습니다.'


def test_safety_comes_before_motor():
    """docs/vica_robot_bringup_manual.md 4절: motor 보다 먼저 띄운다."""
    text = source()
    safety = text.index('safety_bringup.launch.py')
    motor = text.index('motor_bringup.launch.py')
    assert safety < motor, 'safety 를 motor 보다 먼저 두어야 합니다.'


def test_motor_is_included():
    """엔코더 피드백을 요청하는 쪽이 motor node 라 없으면 /wheel/odom 이 안 나온다."""
    assert 'motor_bringup.launch.py' in source()


def test_slam_and_preview_are_included():
    text = source()
    assert 'vica_slam_bringup.launch.py' in text
    assert 'map_preview_node' in text


def test_camera_and_imu_are_not_launched_here():
    """d455 는 Docker, imu 는 20초 정지가 필요해 사람이 한다.

    자동으로 띄우면 자이로 보정을 건너뛴 무효 회차가 만들어진다.
    """
    text = source()
    assert 'realsense' not in text.lower()
    assert 'imu_base_link_adapter' not in text


def test_teleop_is_not_launched_here():
    """키보드 입력을 받는 대화형 프로세스라 띄워도 사람이 그 창에 키를 눌러야 한다."""
    assert 'teleop_twist_keyboard' not in source()
