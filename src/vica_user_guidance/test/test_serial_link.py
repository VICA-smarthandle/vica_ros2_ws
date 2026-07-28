"""시리얼 전송 래퍼 테스트.

포트가 없어도 예외를 밖으로 던지지 않아야 한다. 개발 PC와 하드웨어 미연결 운영
양쪽에서 노드가 계속 살아 있어야 하기 때문이다.
"""

import pytest

from vica_user_guidance import protocol
from vica_user_guidance.serial_link import SerialLink

SEC = 1_000_000_000
NOW = 10 * SEC


class FakePort:
    """pyserial Serial의 최소 대역. write 실패를 흉내낼 수 있다."""

    def __init__(self, fail_write=False):
        self.written = []
        self.fail_write = fail_write
        self.closed = False

    def write(self, data):
        if self.fail_write:
            raise OSError("write failed")
        self.written.append(data)
        return len(data)

    def flush(self):
        pass

    def close(self):
        self.closed = True


def test_disabled_link_reports_not_configured():
    """enable_serial=False면 pyserial을 쓰지 않고 fault만 보고한다."""
    link = SerialLink(port="/dev/null", baudrate=115200, enabled=False)
    assert link.connected is False
    assert link.fault_code == protocol_fault("NOT_CONFIGURED")
    assert link.send(protocol.STATE_NORMAL, NOW) is False


def protocol_fault(name):
    """SmartHandleState.msg의 fault_code 상수값과 일치시킨다."""
    return {"NONE": 0, "PORT_OPEN": 1, "WRITE_FAIL": 2, "NOT_CONFIGURED": 3}[name]


def test_port_open_failure_does_not_raise():
    """포트 open 실패는 예외가 아니라 fault로 보고한다."""

    def failing_factory(**kwargs):
        raise OSError("no such device")

    link = SerialLink(
        port="/dev/nonexistent",
        baudrate=115200,
        enabled=True,
        serial_factory=failing_factory,
    )
    assert link.connected is False
    assert link.fault_code == protocol_fault("PORT_OPEN")


def test_successful_send_writes_one_byte():
    """정상 전송은 1바이트를 쓰고 last_state_code를 갱신한다."""
    port = FakePort()
    link = SerialLink(
        port="/dev/fake",
        baudrate=115200,
        enabled=True,
        serial_factory=lambda **kw: port,
    )
    assert link.connected is True
    assert link.send(protocol.STATE_LEFT, NOW) is True
    assert port.written == [bytes([protocol.STATE_LEFT])]
    assert link.last_state_code == protocol.STATE_LEFT
    assert link.fault_code == protocol_fault("NONE")


def test_write_failure_sets_fault_and_counts():
    """write 실패는 fault_code와 누적 카운터로 보고한다."""
    port = FakePort(fail_write=True)
    link = SerialLink(
        port="/dev/fake",
        baudrate=115200,
        enabled=True,
        serial_factory=lambda **kw: port,
    )
    assert link.send(protocol.STATE_NORMAL, NOW) is False
    assert link.fault_code == protocol_fault("WRITE_FAIL")
    assert link.write_error_count == 1
    assert link.connected is False


def test_link_lost_code_is_rejected():
    """코드 4 전송은 프로그래밍 오류다.

    보내면 펌웨어 워치독이 갱신되어 실제 단절을 감지하지 못하게 된다.
    """
    port = FakePort()
    link = SerialLink(
        port="/dev/fake",
        baudrate=115200,
        enabled=True,
        serial_factory=lambda **kw: port,
    )
    with pytest.raises(ValueError):
        link.send(protocol.STATE_LINK_LOST, NOW)
    assert port.written == []


def test_unimplemented_charging_codes_rejected():
    """펌웨어 미구현 코드도 거부한다."""
    port = FakePort()
    link = SerialLink(
        port="/dev/fake",
        baudrate=115200,
        enabled=True,
        serial_factory=lambda **kw: port,
    )
    with pytest.raises(ValueError):
        link.send(protocol.STATE_CHARGING, NOW)


def test_reconnect_respects_backoff_interval():
    """backoff 간격 이전에는 재연결을 시도하지 않는다."""
    attempts = []

    def counting_factory(**kwargs):
        attempts.append(1)
        raise OSError("still gone")

    link = SerialLink(
        port="/dev/fake",
        baudrate=115200,
        enabled=True,
        serial_factory=counting_factory,
        reconnect_interval_ns=2 * SEC,
    )
    first = len(attempts)

    link.maybe_reconnect(NOW + SEC)          # 간격 미달 → 시도 안 함
    assert len(attempts) == first

    link.maybe_reconnect(NOW + 3 * SEC)      # 간격 초과 → 1회 시도
    assert len(attempts) == first + 1


def test_reconnect_recovers_connection():
    """재연결에 성공하면 connected가 복구된다."""
    port = FakePort()
    state = {"fail": True}

    def flaky_factory(**kwargs):
        if state["fail"]:
            raise OSError("not yet")
        return port

    link = SerialLink(
        port="/dev/fake",
        baudrate=115200,
        enabled=True,
        serial_factory=flaky_factory,
        reconnect_interval_ns=SEC,
    )
    assert link.connected is False

    # 첫 호출은 backoff 기준점만 잡는다 (생성자는 now_ns를 모른다)
    link.maybe_reconnect(NOW)
    assert link.connected is False

    state["fail"] = False
    link.maybe_reconnect(NOW + 2 * SEC)
    assert link.connected is True
    assert link.fault_code == protocol_fault("NONE")


def test_close_is_safe_when_never_opened():
    """열린 적 없어도 close가 예외를 던지지 않는다."""
    link = SerialLink(port="/dev/null", baudrate=115200, enabled=False)
    link.close()


def test_close_closes_underlying_port():
    port = FakePort()
    link = SerialLink(
        port="/dev/fake",
        baudrate=115200,
        enabled=True,
        serial_factory=lambda **kw: port,
    )
    link.close()
    assert port.closed is True
    assert link.connected is False
