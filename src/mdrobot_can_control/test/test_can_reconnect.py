"""CAN 재연결 경로 회귀 테스트.

`try_reconnect_can`은 재개방 전에 기존 bus를 shutdown한다. 재개방 자체가
실패하면 `self.bus`에 이미 닫힌 핸들이 남고, 같은 사이클의 `drain_can_rx`·
`send_vel_cmd`가 그 핸들을 건드린다. 닫힌 socketcan bus는 `CanError`가 아니라
`ValueError('file descriptor cannot be a negative integer (-1)')`를 던지므로
`except can.CanError`를 통과해 타이머 콜백 밖으로 탈출하고 노드가 죽는다.
CAN 장애에서 노드를 살려 두는 것이 이 변경의 목적이므로 회귀로 고정한다.

`ip link set can1 down`으로는 재현되지 않는다. 인터페이스가 존재하면 down
상태여도 bind가 성공해 재개방이 실패하지 않기 때문이다. 재개방이 실패하는
것은 인터페이스가 사라진 경우(모듈 언로드, USB-CAN 탈거)다.
"""

import can

from mdrobot_can_control import mdrobot_can_keyboard_knob_node as node_mod
from mdrobot_can_control.can_link import CanLink
from mdrobot_can_control.freshness import sec_to_ns
from mdrobot_can_control.mdrobot_can_keyboard_knob_node import (
    MdrobotCanKeyboardKnobNode,
)


T0 = 1_000_000_000
RETRY_NS = sec_to_ns(1.0)


class ClosedBus:
    """socketcan bus가 닫힌 뒤의 거동을 그대로 흉내낸다."""

    def __init__(self):
        """Start open; shutdown() flips it to the closed behaviour."""
        self.closed = False

    def shutdown(self):
        """Close the socket like SocketcanBus.shutdown does."""
        self.closed = True

    def recv(self, timeout=0.0):
        """Raise ValueError on a closed socket, as python-can 4.6.1 does."""
        del timeout
        if self.closed:
            raise ValueError(
                'file descriptor cannot be a negative integer (-1)'
            )
        return None

    def send(self, msg):
        """Raise ValueError on a closed socket, as python-can 4.6.1 does."""
        del msg
        if self.closed:
            raise ValueError(
                'file descriptor cannot be a negative integer (-1)'
            )


class FakeLogger:
    """Collect log lines instead of touching rclpy."""

    def __init__(self):
        """Start with no recorded lines."""
        self.lines = []

    def info(self, text):
        """Record an info line."""
        self.lines.append(('info', text))

    def error(self, text):
        """Record an error line."""
        self.lines.append(('error', text))

    def warn(self, text):
        """Record a warn line."""
        self.lines.append(('warn', text))


class FakeReconnectNode:
    """Hold only the attributes the CAN paths touch."""

    def __init__(self, bus):
        """Start with a failed link so reconnect is due."""
        self.bus = bus
        self.can_iface = 'can1'
        self.driver_id = 0x01
        self.max_rpm = 1000
        self.can_reconnect_interval_ns = RETRY_NS
        self.can_link = CanLink(retry_interval_ns=RETRY_NS)
        self.can_link.record_error(can.CanError('gone'), T0)
        self.last_can_error_log_ns = None
        self.last_knob_ns = None
        self.knob1 = 0
        self.knob2 = 0
        self.logger = FakeLogger()

    def get_logger(self):
        """Return the collecting logger."""
        return self.logger

    def now_ns(self):
        """Return a fixed instant; these tests inject time explicitly."""
        return T0 + RETRY_NS

    def log_can_error_throttled(self, phase, exc):
        """Record the throttled error without rclpy."""
        self.logger.error(f'[CAN FAULT] phase={phase} error={exc}')

    def send_pnt_io_monitor_on(self):
        """Run the real probe against whatever bus is installed."""
        return MdrobotCanKeyboardKnobNode.send_pnt_io_monitor_on(self)


def reconnect(node, now_ns):
    """Call the real reconnect path against the fake node."""
    return MdrobotCanKeyboardKnobNode.try_reconnect_can(node, now_ns)


def make_node_with_failing_reopen(monkeypatch, exc):
    """Fail every reopen attempt with the given exception."""
    node = FakeReconnectNode(ClosedBus())

    def boom(*args, **kwargs):
        del args, kwargs
        raise exc

    monkeypatch.setattr(node_mod.can.interface, 'Bus', boom)
    return node


def test_failed_reopen_leaves_no_bus_handle(monkeypatch):
    """재개방이 실패하면 닫힌 핸들을 남기지 않는다."""
    node = make_node_with_failing_reopen(
        monkeypatch,
        OSError('No such device'),
    )

    reconnect(node, T0 + RETRY_NS)

    assert node.bus is None
    assert not node.can_link.is_ok()


def test_drain_survives_after_failed_reopen(monkeypatch):
    """재개방 실패 뒤 drain_can_rx가 예외를 밖으로 던지지 않는다."""
    node = make_node_with_failing_reopen(
        monkeypatch,
        OSError('No such device'),
    )
    reconnect(node, T0 + RETRY_NS)

    MdrobotCanKeyboardKnobNode.drain_can_rx(node, T0 + RETRY_NS)

    assert not node.can_link.is_ok()


def test_send_survives_after_failed_reopen(monkeypatch):
    """재개방 실패 뒤 send_vel_cmd가 예외를 밖으로 던지지 않는다."""
    node = make_node_with_failing_reopen(
        monkeypatch,
        OSError('No such device'),
    )
    reconnect(node, T0 + RETRY_NS)

    MdrobotCanKeyboardKnobNode.send_vel_cmd(node, 0, 0)

    assert not node.can_link.is_ok()


def test_reopen_failing_with_valueerror_is_handled(monkeypatch):
    """socketcan이 ValueError를 던져도 재연결 경로가 이를 상태로 바꾼다."""
    node = make_node_with_failing_reopen(
        monkeypatch,
        ValueError('file descriptor cannot be a negative integer (-1)'),
    )

    reconnect(node, T0 + RETRY_NS)

    assert node.bus is None
    assert not node.can_link.is_ok()


def test_probe_failure_keeps_link_failed(monkeypatch):
    """버스는 열렸지만 프로브 송신이 실패하면 링크는 여전히 비정상이다.

    socketcan은 상대가 없어도 bind에 성공한다. 프로브 예외를 삼키면
    `/motor/can_ok`가 거짓으로 true가 되어 관리자 reset이 수락된다.
    """
    node = FakeReconnectNode(ClosedBus())

    class DeadDriverBus:
        """Open successfully but reject every transmission."""

        def shutdown(self):
            """Accept shutdown without doing anything."""

        def send(self, msg):
            """Reject the probe like a driver-less bus does."""
            del msg
            raise can.CanOperationError('Network is down')

        def recv(self, timeout=0.0):
            """Return nothing."""
            del timeout
            return None

    monkeypatch.setattr(
        node_mod.can.interface,
        'Bus',
        lambda *args, **kwargs: DeadDriverBus(),
    )

    reconnect(node, T0 + RETRY_NS)

    assert not node.can_link.is_ok()


def test_successful_reopen_restores_link(monkeypatch):
    """재개방과 프로브가 모두 성공하면 링크가 정상으로 돌아온다."""
    node = FakeReconnectNode(ClosedBus())

    class HealthyBus:
        """Accept the probe and report an empty receive queue."""

        def shutdown(self):
            """Accept shutdown without doing anything."""

        def send(self, msg):
            """Accept the probe frame."""
            del msg

        def recv(self, timeout=0.0):
            """Return nothing."""
            del timeout
            return None

    monkeypatch.setattr(
        node_mod.can.interface,
        'Bus',
        lambda *args, **kwargs: HealthyBus(),
    )

    reconnect(node, T0 + RETRY_NS)

    assert node.can_link.is_ok()
    assert any('[CAN RECOVERED]' in text for _, text in node.logger.lines)


def test_closed_handle_never_escapes_on_recv():
    """닫힌 핸들이 남아 있어도 recv 경로가 예외를 밖으로 던지지 않는다."""
    node = FakeReconnectNode(ClosedBus())
    node.bus.shutdown()

    MdrobotCanKeyboardKnobNode.drain_can_rx(node, T0)

    assert not node.can_link.is_ok()


def test_closed_handle_never_escapes_on_send():
    """닫힌 핸들이 남아 있어도 send 경로가 예외를 밖으로 던지지 않는다."""
    node = FakeReconnectNode(ClosedBus())
    node.bus.shutdown()

    MdrobotCanKeyboardKnobNode.send_vel_cmd(node, 0, 0)

    assert not node.can_link.is_ok()
