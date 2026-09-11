"""ApproachRequestThrottle 시험 — 호출 폭주 억제가 계약을 지키는가."""
import pytest

from vica_perception.approach_request_policy import (
    DEFAULT_MIN_INTERVAL_NS,
    ApproachRequestThrottle,
)

S = 1_000_000_000


class TestShouldSend:
    def test_첫_호출은_항상_허용된다(self):
        th = ApproachRequestThrottle()
        assert th.should_send(7, now_ns=0) is True

    def test_간격_안의_재호출은_막힌다(self):
        th = ApproachRequestThrottle()
        assert th.should_send(7, 0) is True
        assert th.should_send(7, DEFAULT_MIN_INTERVAL_NS - 1) is False

    def test_간격이_지나면_다시_허용된다(self):
        th = ApproachRequestThrottle()
        assert th.should_send(7, 0) is True
        assert th.should_send(7, DEFAULT_MIN_INTERVAL_NS) is True

    def test_track_이_다르면_서로_안_막는다(self):
        th = ApproachRequestThrottle()
        assert th.should_send(7, 0) is True
        assert th.should_send(8, 1) is True

    def test_거부된_호출은_기록을_안_남긴다(self):
        th = ApproachRequestThrottle(min_interval_ns=10)
        assert th.should_send(7, 0) is True
        assert th.should_send(7, 5) is False
        assert th.should_send(7, 10) is True

    def test_5Hz_판정에서_초당_1건으로_준다(self):
        th = ApproachRequestThrottle()
        sent = sum(
            1 for i in range(15)
            if th.should_send(7, i * S // 5)
        )
        assert sent == 3


class TestHousekeeping:
    def test_forget_뒤에는_즉시_다시_보낼_수_있다(self):
        th = ApproachRequestThrottle()
        th.should_send(7, 0)
        th.forget(7)
        assert th.should_send(7, 1) is True

    def test_prune_은_오래된_track_만_지운다(self):
        th = ApproachRequestThrottle()
        th.should_send(1, 0)
        th.should_send(2, 61 * S)
        th.prune(now_ns=62 * S)
        assert th.should_send(1, 62 * S) is True
        assert th.should_send(2, 61 * S + S // 2) is False

    def test_음수_간격은_거부한다(self):
        with pytest.raises(ValueError):
            ApproachRequestThrottle(min_interval_ns=-1)
