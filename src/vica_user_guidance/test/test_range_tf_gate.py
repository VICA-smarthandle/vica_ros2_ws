"""초음파 Range 발행 게이트 순수 로직 테스트."""

import pytest

from vica_user_guidance.range_tf_gate import RangeTfGate


def test_blocks_when_tf_missing():
    """TF 를 못 찾으면 그 채널은 발행하지 않는다 — 이 게이트의 존재 이유."""
    gate = RangeTfGate(channels=2)
    assert gate.allow(0, tf_ok=False) is False


def test_allows_when_tf_available():
    """평시에는 그대로 통과시킨다. 게이트가 기능을 줄이면 안 된다."""
    gate = RangeTfGate(channels=2)
    assert gate.allow(0, tf_ok=True) is True


def test_resumes_immediately_after_tf_returns():
    """TF 가 돌아오면 다음 프레임에서 곧바로 재개한다."""
    gate = RangeTfGate(channels=2)
    gate.allow(0, tf_ok=False)
    gate.allow(0, tf_ok=False)
    assert gate.allow(0, tf_ok=True) is True


def test_channels_are_independent():
    """한 채널의 TF 실패가 다른 채널을 막지 않는다."""
    gate = RangeTfGate(channels=2)
    assert gate.allow(0, tf_ok=False) is False
    assert gate.allow(1, tf_ok=True) is True


def test_transition_reported_once_per_state_change():
    """로그는 상태가 바뀔 때만 1회다."""
    gate = RangeTfGate(channels=1)

    gate.allow(0, tf_ok=True)
    assert gate.take_transition(0) is None

    gate.allow(0, tf_ok=False)
    assert gate.take_transition(0) is False
    gate.allow(0, tf_ok=False)
    assert gate.take_transition(0) is None

    gate.allow(0, tf_ok=True)
    assert gate.take_transition(0) is True
    gate.allow(0, tf_ok=True)
    assert gate.take_transition(0) is None


def test_blocked_count_is_kept_for_diagnosis():
    """막은 횟수를 센다. 진단에서 '조용히 안 나간 것'을 알 수 있어야 한다."""
    gate = RangeTfGate(channels=1)
    gate.allow(0, tf_ok=False)
    gate.allow(0, tf_ok=False)
    gate.allow(0, tf_ok=True)
    assert gate.blocked_count(0) == 2


def test_rejects_bad_channel_index():
    """채널 수를 벗어난 접근은 조용히 통과시키지 않는다."""
    gate = RangeTfGate(channels=2)
    with pytest.raises(IndexError):
        gate.allow(2, tf_ok=True)


def test_channels_must_be_positive():
    """채널 0개는 설정 실수다. 조용히 굴러가느니 기동 때 죽는 편이 낫다."""
    with pytest.raises(ValueError):
        RangeTfGate(channels=0)
