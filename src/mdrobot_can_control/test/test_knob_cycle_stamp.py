"""같은 제어 사이클에서 읽은 knob 입력의 시간 역전 오판정 회귀 테스트.

`control_loop`는 판정 기준 시각 `now`를 먼저 캡처한 뒤 CAN을 drain한다.
drain이 자체적으로 더 나중 시각을 찍으면 `last_knob_ns > now`가 되어, 정상
수신한 knob 입력이 음수 age(시간 역전)로 stale 판정된다. 그 결과 매 사이클
motor 출력이 0으로 떨어진다. 사이클 기준 시각을 drain에 주입해 stamp와 판정이
같은 순간을 가리키도록 한다.
"""

from mdrobot_can_control.freshness import sec_to_ns
from mdrobot_can_control.mdrobot_can_keyboard_knob_node import (
    MdrobotCanKeyboardKnobNode,
)
from mdrobot_can_control.motor_watchdog import motor_speed_ratio


CMD_TIMEOUT_NS = sec_to_ns(0.5)
KNOB_TIMEOUT_NS = sec_to_ns(0.8)
T0 = 1_000_000_000
KNOB_PCT = 48


def knob_frame(knob1, knob2):
    """Build an F1 PNT I/O monitor frame carrying both knob values."""
    return bytes([0xF1, 0x00, 0x68, 0x08, 0x01, 0x03, knob1, knob2])


class FakeMessage:
    """Minimal stand-in for a python-can message."""

    def __init__(self, data):
        """Store the raw 8-byte CAN payload."""
        self.data = data


class FakeBus:
    """Deliver a fixed frame list once, then behave as an empty bus."""

    def __init__(self, frames):
        """Queue the frames this fake bus will hand out."""
        self.frames = list(frames)

    def recv(self, timeout=0.0):
        """Return the next queued frame, or None when drained."""
        del timeout
        if not self.frames:
            return None
        return FakeMessage(self.frames.pop(0))


class FakeKnobNode:
    """Hold only the attributes `drain_can_rx` touches."""

    def __init__(self, frames):
        """Start with no knob reading and an unstamped timestamp."""
        self.bus = FakeBus(frames)
        self.knob1 = 0
        self.knob2 = 0
        self.last_knob_ns = None


def test_drain_stamps_knob_with_the_injected_cycle_instant():
    """drain은 사이클 기준 시각으로 stamp해야 한다(자체 시각 조회 금지)."""
    node = FakeKnobNode([knob_frame(KNOB_PCT, KNOB_PCT)])

    MdrobotCanKeyboardKnobNode.drain_can_rx(node, T0)

    assert node.knob1 == KNOB_PCT
    assert node.last_knob_ns == T0


def test_knob_read_in_this_cycle_is_not_judged_time_reversed():
    """같은 사이클에서 읽은 knob은 stale이 아니라 knob 비율을 통과시킨다."""
    node = FakeKnobNode([knob_frame(KNOB_PCT, KNOB_PCT)])
    now = T0

    MdrobotCanKeyboardKnobNode.drain_can_rx(node, now)

    ratio = motor_speed_ratio(
        cmd_last_ns=now,
        knob_last_ns=node.last_knob_ns,
        knob_pct=node.knob1,
        now_ns=now,
        cmd_timeout_ns=CMD_TIMEOUT_NS,
        knob_timeout_ns=KNOB_TIMEOUT_NS,
        deadzone_pct=5,
    )

    assert ratio == KNOB_PCT / 100.0


def test_drain_without_knob_frame_keeps_previous_stamp():
    """자격 프레임이 없으면 stamp를 갱신하지 않는다."""
    node = FakeKnobNode([])
    node.last_knob_ns = T0

    MdrobotCanKeyboardKnobNode.drain_can_rx(node, T0 + sec_to_ns(1.0))

    assert node.last_knob_ns == T0
