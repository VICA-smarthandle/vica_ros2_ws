"""CAN 링크 상태 모델 단위 테스트.

주행 중 CAN 장애에서 프로세스를 유지하되 출력을 0으로 막기 위한 판정이다.
시각은 모두 정수 나노초(STEADY_TIME)이며 모델이 스스로 조회하지 않는다.
"""

from mdrobot_can_control.can_link import CanLink
from mdrobot_can_control.freshness import sec_to_ns


T0 = 1_000_000_000
RETRY_NS = sec_to_ns(1.0)


def test_new_link_is_ok():
    """Bus 개방 성공 뒤에만 생성하므로 초기 상태는 정상이다."""
    link = CanLink(retry_interval_ns=RETRY_NS)

    assert link.is_ok() is True
    assert link.last_error is None


def test_error_marks_link_failed():
    """CAN 예외를 기록하면 즉시 비정상으로 전환한다."""
    link = CanLink(retry_interval_ns=RETRY_NS)

    link.record_error(OSError('Network is down'), T0)

    assert link.is_ok() is False
    assert 'Network is down' in link.last_error


def test_success_restores_link():
    """재연결 성공 뒤에는 정상으로 복귀한다."""
    link = CanLink(retry_interval_ns=RETRY_NS)
    link.record_error(OSError('boom'), T0)

    link.record_success()

    assert link.is_ok() is True


def test_healthy_link_never_retries():
    """정상 상태에서는 재연결을 시도하지 않는다."""
    link = CanLink(retry_interval_ns=RETRY_NS)

    assert link.should_retry(T0) is False


def test_retry_waits_for_interval():
    """실패 직후에는 재시도하지 않고 간격이 지나야 시도한다."""
    link = CanLink(retry_interval_ns=RETRY_NS)
    link.record_error(OSError('boom'), T0)

    assert link.should_retry(T0 + RETRY_NS - 1) is False
    assert link.should_retry(T0 + RETRY_NS) is True


def test_retry_attempt_restarts_interval():
    """시도했으면 다음 간격까지 다시 기다린다."""
    link = CanLink(retry_interval_ns=RETRY_NS)
    link.record_error(OSError('boom'), T0)
    link.mark_retry_attempted(T0 + RETRY_NS)

    assert link.should_retry(T0 + RETRY_NS + 1) is False
    assert link.should_retry(T0 + RETRY_NS * 2) is True


def test_time_reversal_allows_retry():
    """시간이 뒤로 가도 재연결이 영구히 막히면 안 된다(fail-safe 방향)."""
    link = CanLink(retry_interval_ns=RETRY_NS)
    link.record_error(OSError('boom'), T0)

    assert link.should_retry(T0 - sec_to_ns(3600)) is True
