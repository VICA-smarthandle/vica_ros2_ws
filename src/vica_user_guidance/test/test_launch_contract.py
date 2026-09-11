"""launch·config·setup 계약 테스트."""

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


def strip_comments_and_docstrings(source: str) -> str:
    """주석과 문자열 리터럴을 제거해 실행 코드만 남긴다."""
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
    """guidance 실행 코드에 주행 명령·goal·reset 심볼이 없어야 한다."""
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
    """금지 경로에 publisher/service client를 만들지 않는다."""
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


def test_node_names_match_config_keys():
    """launch의 name=과 YAML 최상위 키가 일치해야 한다."""
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


def test_config_baudrate_matches_firmware():
    """config baudrate가 펌웨어 Serial.begin과 일치해야 한다."""
    config = load_config()
    baud = config["user_guidance_driver_node"]["ros__parameters"]["baudrate"]
    assert baud == 115200
    if FIRMWARE_INO.exists():
        assert f"Serial.begin({baud})" in read(FIRMWARE_INO)


def test_arrival_hold_covers_firmware_animation():
    """arrival_hold_sec가 펌웨어 재생 시간(3.5초) 이상이어야 한다."""
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
    """노드가 선언한 기본 포트와 config 값이 같아야 한다."""
    configured = load_config()["user_guidance_driver_node"]["ros__parameters"][
        "serial_port"
    ]
    source = read(NODE_DIR / "user_guidance_driver_node.py")
    assert f'declare_parameter("serial_port", "{configured}")' in source


def test_serial_port_is_not_an_enumeration_dependent_path():
    """/dev/ttyUSB* 같은 열거 순서 의존 경로를 기본값으로 쓰지 않는다."""
    configured = load_config()["user_guidance_driver_node"]["ros__parameters"][
        "serial_port"
    ]
    assert not configured.startswith("/dev/ttyUSB")
    assert not configured.startswith("/dev/ttyACM")


def test_driver_haptic_request_is_manual_only():
    """/vica/haptic_request 를 구독해 send_raw 로만 보낸다. 스스로 발행하지 않는다."""
    text = read(NODE_DIR / "user_guidance_driver_node.py")
    assert 'create_subscription(\n            String, "/vica/haptic_request"' in text
    assert 'create_publisher(String, "/vica/haptic_request"' not in text
    assert "self.link.send_raw(code" in text
    code = strip_comments_and_docstrings(text)
    assert code.count("send_raw") == 1, (
        "send_raw 호출이 한 곳(cb_haptic_request)이 아니다 — 자동 트리거 의심"
    )


def test_touch_state_is_initialised_before_any_enable_branch():
    """diag_loop 가 읽는 터치 필드는 touch_enabled 값과 무관하게 __init__ 에서 먼저 만든다."""
    text = read(NODE_DIR / "user_guidance_driver_node.py")
    init_contact = text.index("self.touch_contact = False")
    init_last = text.index("self.touch_last_frame_ns = None")
    enable_branch = text.index("if self.touch_enabled:\n            self._setup_touch()")
    assert init_contact < enable_branch, "touch_contact 초기화가 enable 분기 뒤에 있다"
    assert init_last < enable_branch, "touch_last_frame_ns 초기화가 enable 분기 뒤에 있다"
    setup_body = text.split("def _setup_touch(self)")[1].split("def ")[0]
    assert "self.touch_contact" not in setup_body
    assert "self.touch_last_frame_ns" not in setup_body
