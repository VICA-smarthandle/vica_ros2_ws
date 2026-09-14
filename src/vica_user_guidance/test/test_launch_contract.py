"""launch·config·setup 계약 테스트.

파일을 문자열로 읽어 검사한다. rclpy나 launch를 import하지 않으므로 ROS 환경 없이
실행된다. 특히 안전 경계 위반을 자동으로 잡는 유일한 검사다.
"""

from pathlib import Path

import pytest
import yaml

PKG_ROOT = Path(__file__).resolve().parents[1]

LAUNCH_FILE = PKG_ROOT / "launch" / "user_guidance.launch.py"
CONFIG_FILE = PKG_ROOT / "config" / "user_guidance.yaml"
SETUP_FILE = PKG_ROOT / "setup.py"
PACKAGE_XML = PKG_ROOT / "package.xml"
NODE_DIR = PKG_ROOT / "vica_user_guidance"

FIRMWARE_INO = (
    PKG_ROOT / "firmware" / "smart_handle_firmware" / "smart_handle_firmware.ino"
)

# guidance 계층이 절대 건드리면 안 되는 심볼들.
FORBIDDEN_SYMBOLS = [
    "cmd_vel_req",
    "cmd_vel_safe",
    "NavigateToPose",
    "estop_reset",
    "safety_reset",
]


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def load_config() -> dict:
    return yaml.safe_load(read(CONFIG_FILE))


# ── 안전 경계 ─────────────────────────────────────────


def strip_comments_and_docstrings(source: str) -> str:
    """주석과 문자열 리터럴을 제거해 실행 코드만 남긴다.

    안전 경계를 설명하는 주석("/cmd_vel_req를 발행하지 않는다")까지 금지 심볼로
    잡히면 문서화를 못 하게 된다. 검사 대상은 실제 실행되는 코드다.
    """
    import io
    import tokenize

    kept = []
    tokens = tokenize.generate_tokens(io.StringIO(source).readline)
    for tok_type, tok_str, _, _, _ in tokens:
        if tok_type in (tokenize.COMMENT, tokenize.STRING):
            continue
        kept.append(tok_str)
    return " ".join(kept)


def test_sources_never_touch_drive_command_path():
    """guidance 실행 코드에 주행 명령·goal·reset 심볼이 없어야 한다.

    이 계층은 순수 출력이며 Safety/Nav2 경로를 우회하지 않는다. 주석·문자열은
    제외하므로 "발행하지 않는다"는 설명은 남길 수 있다.
    """
    sources = list(NODE_DIR.glob("*.py")) + [LAUNCH_FILE]
    for path in sources:
        code = strip_comments_and_docstrings(read(path))
        for symbol in FORBIDDEN_SYMBOLS:
            assert symbol not in code, f"{path.name} 실행 코드에 {symbol!r}이 있다"


def test_driver_subscribes_estop_state_without_publishing():
    """/estop_state를 구독만 한다. 발행하지 않는다."""
    text = read(NODE_DIR / "user_guidance_driver_node.py")
    assert 'create_subscription(Bool, "/estop_state"' in text
    assert 'create_publisher(Bool, "/estop_state"' not in text


def test_no_publisher_or_service_on_forbidden_paths():
    """금지 경로에 publisher/service client를 만들지 않는다.

    토픽 이름은 문자열이라 실행 코드 검사만으로는 부족하다. create_publisher와
    create_client 호출 자체를 훑어 금지 심볼이 인자로 들어가는지 본다.
    """
    import re

    pattern = re.compile(
        r"create_(publisher|client)\s*\([^)]*", re.MULTILINE | re.DOTALL
    )
    for path in NODE_DIR.glob("*.py"):
        for call in pattern.finditer(read(path)):
            snippet = call.group(0)
            for symbol in FORBIDDEN_SYMBOLS:
                assert symbol not in snippet, (
                    f"{path.name}의 {snippet[:60]!r}에 금지 심볼 {symbol!r}"
                )


# ── launch 계약 ───────────────────────────────────────


def test_node_names_match_config_keys():
    """launch의 name=과 YAML 최상위 키가 일치해야 한다.

    다르면 파라미터가 조용히 무시되고 전부 기본값으로 동작한다.
    """
    launch_text = read(LAUNCH_FILE)
    for key in load_config():
        assert f'name="{key}"' in launch_text, f"launch에 name={key!r}가 없다"


def test_launch_exposes_enable_serial_argument():
    """하드웨어 없이 mock 실행이 가능해야 한다."""
    text = read(LAUNCH_FILE)
    assert 'DeclareLaunchArgument(\n                "enable_serial"' in text or (
        '"enable_serial"' in text and "DeclareLaunchArgument" in text
    )


def test_launch_sets_colorized_output():
    """심각도별 색상 로그는 이 워크스페이스의 UX 계약이다."""
    assert 'SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")' in read(
        LAUNCH_FILE
    )


# ── setup / package.xml ───────────────────────────────


def test_both_executables_registered():
    text = read(SETUP_FILE)
    assert "turn_guide_node = vica_user_guidance.turn_guide_node:main" in text
    assert "vica_user_guidance.user_guidance_driver_node:main" in text


def test_config_and_launch_are_installed():
    """share에 설치되지 않으면 launch가 YAML을 찾지 못한다."""
    text = read(SETUP_FILE)
    assert 'glob("launch/*.launch.py")' in text
    assert 'glob("config/*.yaml")' in text


def test_depends_on_interfaces_not_on_safety():
    """vica_safety에 역방향 의존을 만들지 않는다."""
    text = read(PACKAGE_XML)
    assert "<depend>vica_interfaces</depend>" in text
    assert "vica_safety" not in text.replace(
        "<!--", "\n<!--"
    ).split("<!--")[0] or "<depend>vica_safety</depend>" not in text


# ── 펌웨어 결합 ───────────────────────────────────────


def test_config_baudrate_matches_firmware():
    """config baudrate가 펌웨어 Serial.begin과 일치해야 한다."""
    config = load_config()
    baud = config["user_guidance_driver_node"]["ros__parameters"]["baudrate"]
    assert baud == 115200
    if FIRMWARE_INO.exists():
        assert f"Serial.begin({baud})" in read(FIRMWARE_INO)


def test_arrival_hold_covers_firmware_animation():
    """arrival_hold_sec가 펌웨어 재생 시간(3.5초) 이상이어야 한다.

    짧으면 ROS가 먼저 코드 0을 보내 마지막 점멸 프레임이 잘린다.
    2026-07-28에 실제로 발견된 결함의 재발 방지 장치다.
    """
    from vica_user_guidance import protocol

    config = load_config()
    hold = config["user_guidance_driver_node"]["ros__parameters"]["arrival_hold_sec"]
    assert hold >= protocol.firmware_arrival_duration_sec(), (
        f"arrival_hold_sec={hold}s는 펌웨어 애니메이션 "
        f"{protocol.firmware_arrival_duration_sec()}s보다 짧다"
    )


def test_hysteresis_thresholds_are_ordered():
    """exit 임계값이 enter보다 작아야 채터링이 없다."""
    params = load_config()["turn_guide_node"]["ros__parameters"]
    assert params["exit_threshold_deg"] < params["enter_threshold_deg"]


def test_send_rate_is_well_within_firmware_watchdog():
    """전송 주기가 펌웨어 워치독보다 충분히 짧아야 오탐이 없다."""
    from vica_user_guidance import protocol

    params = load_config()["user_guidance_driver_node"]["ros__parameters"]
    period_ms = 1000.0 / params["send_rate_hz"]
    assert period_ms * 3 < protocol.FIRMWARE_WATCHDOG_TIMEOUT_MS


def test_write_timeout_shorter_than_send_period():
    """write_timeout이 전송 주기보다 길면 콜백이 밀린다."""
    params = load_config()["user_guidance_driver_node"]["ros__parameters"]
    period_sec = 1.0 / params["send_rate_hz"]
    assert params["write_timeout_sec"] < period_sec


@pytest.mark.parametrize("key", ["turn_guide_node", "user_guidance_driver_node"])
def test_config_has_ros_parameters_block(key):
    """ros__parameters 블록이 없으면 파라미터가 로드되지 않는다."""
    assert "ros__parameters" in load_config()[key]


def test_serial_port_default_matches_config():
    """노드가 선언한 기본 포트와 config 값이 같아야 한다.

    launch로 띄우면 YAML이 덮어쓰지만 `ros2 run`으로 직접 띄우면 declare_parameter
    기본값이 쓰인다. 두 값이 어긋나면 실행 방식에 따라 다른 포트를 열게 되고,
    번호가 밀린 다른 USB 장치를 스마트핸들로 오인해 열 수도 있다.
    """
    configured = load_config()["user_guidance_driver_node"]["ros__parameters"][
        "serial_port"
    ]
    source = read(NODE_DIR / "user_guidance_driver_node.py")
    assert f'declare_parameter("serial_port", "{configured}")' in source


def test_serial_port_is_not_an_enumeration_dependent_path():
    """/dev/ttyUSB* 같은 열거 순서 의존 경로를 기본값으로 쓰지 않는다.

    다른 USB-시리얼 장치를 먼저 꽂으면 번호가 밀린다. udev 규칙이 부여하는 고정
    이름을 쓰는 것이 이 규칙의 존재 이유다.
    """
    configured = load_config()["user_guidance_driver_node"]["ros__parameters"][
        "serial_port"
    ]
    assert not configured.startswith("/dev/ttyUSB")
    assert not configured.startswith("/dev/ttyACM")


def test_driver_haptic_request_is_manual_only():
    """/vica/haptic_request 를 구독해 send_raw 로만 보낸다. 스스로 발행하지 않는다.

    자동 트리거(ESTOP·ARRIVED 진입 시 진동)는 별도 결정 사항이다. 이 시험은
    '노드가 햅틱을 제 판단으로 쏘는 경로가 없다'를 고정한다 — 그 경로가 생기면
    사용자 요구("내가 신호 줄 때만 울린다")가 깨진다.
    """
    text = read(NODE_DIR / "user_guidance_driver_node.py")
    assert 'create_subscription(\n            String, "/vica/haptic_request"' in text
    assert 'create_publisher(String, "/vica/haptic_request"' not in text
    # 햅틱 바이트는 send() 가 아니라 send_raw() 로만 나간다 — last_state_code 보호
    assert "self.link.send_raw(code" in text
    # strip_comments_and_docstrings 는 토큰 사이에 공백을 넣어 되붙이므로
    # "send_raw(" 가 아니라 토큰 "send_raw" 를 센다.
    code = strip_comments_and_docstrings(text)
    assert code.count("send_raw") == 1, (
        "send_raw 호출이 한 곳(cb_haptic_request)이 아니다 — 자동 트리거 의심"
    )


def test_touch_state_is_initialised_before_any_enable_branch():
    """diag_loop 가 읽는 터치 필드는 touch_enabled 값과 무관하게 __init__ 에서 먼저 만든다.

    2026-09-04 사고: touch_enabled=false 면 _setup_touch() 를 건너뛰는데 diag_loop 가
    그 안에서만 만들어지던 touch_contact 를 읽어 첫 진단 tick 에 노드가 죽었다
    (AttributeError) → 펌웨어 워치독 1.5초 → 빨간불. 진단이 읽는 필드는 어떤 설정에서도
    존재해야 한다. 이 시험은 그 초기화가 enable 분기보다 **앞에** 있음을 고정한다.
    """
    text = read(NODE_DIR / "user_guidance_driver_node.py")
    init_contact = text.index("self.touch_contact = False")
    init_last = text.index("self.touch_last_frame_ns = None")
    enable_branch = text.index("if self.touch_enabled:\n            self._setup_touch()")
    assert init_contact < enable_branch, "touch_contact 초기화가 enable 분기 뒤에 있다"
    assert init_last < enable_branch, "touch_last_frame_ns 초기화가 enable 분기 뒤에 있다"
    # diag_loop 는 이 둘만 읽는다 — _setup_touch 안에서 새 필드를 만들어 diag_loop 가
    # 읽게 하면 같은 사고가 재발한다.
    setup_body = text.split("def _setup_touch(self)")[1].split("def ")[0]
    assert "self.touch_contact" not in setup_body
    assert "self.touch_last_frame_ns" not in setup_body
