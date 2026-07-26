"""최종 구동단(motor driver) steady-clock watchdog 단위 테스트.

timeout 판정이 단일 STEADY_TIME clock과 정수 나노초를 쓰고, 미수신·시간
역전·timeout에서 반드시 0 출력으로 정지하는지 검증한다.
"""

from mdrobot_can_control.freshness import is_fresh_ns, sec_to_ns
from mdrobot_can_control.motor_watchdog import motor_speed_ratio


CMD_TIMEOUT_NS = sec_to_ns(0.5)
KNOB_TIMEOUT_NS = sec_to_ns(0.8)
T0 = 1_000_000_000


def test_never_received_cmd_or_knob_is_stale():
    """한 번도 수신하지 않으면(None) stale로 판정한다."""
    assert is_fresh_ns(None, now_ns=T0, timeout_ns=CMD_TIMEOUT_NS) is False


def test_cmd_just_before_timeout_is_fresh():
    """경계값 age == timeout_ns는 fresh다."""
    last = T0
    now = T0 + CMD_TIMEOUT_NS
    assert is_fresh_ns(last, now_ns=now, timeout_ns=CMD_TIMEOUT_NS) is True


def test_cmd_just_after_timeout_is_stale():
    """경계 초과 age == timeout_ns + 1은 stale이다."""
    last = T0
    now = T0 + CMD_TIMEOUT_NS + 1
    assert is_fresh_ns(last, now_ns=now, timeout_ns=CMD_TIMEOUT_NS) is False


def test_time_reversal_is_stale():
    """시간 역전(now < last, 음수 age)은 stale이다."""
    assert is_fresh_ns(T0, now_ns=T0 - 1, timeout_ns=CMD_TIMEOUT_NS) is False


def test_both_fresh_forwards_command_scale():
    """cmd·knob 모두 fresh면 knob 비율(여기선 100%)을 통과시킨다."""
    scale = motor_speed_ratio(
        cmd_last_ns=T0,
        knob_last_ns=T0,
        knob_pct=100,
        now_ns=T0,
        cmd_timeout_ns=CMD_TIMEOUT_NS,
        knob_timeout_ns=KNOB_TIMEOUT_NS,
        deadzone_pct=5,
    )
    assert scale == 1.0


def test_cmd_stale_stops_even_if_knob_fresh():
    """cmd만 stale이어도 motor를 정지(0.0)한다."""
    scale = motor_speed_ratio(
        cmd_last_ns=None,
        knob_last_ns=T0,
        knob_pct=100,
        now_ns=T0,
        cmd_timeout_ns=CMD_TIMEOUT_NS,
        knob_timeout_ns=KNOB_TIMEOUT_NS,
        deadzone_pct=5,
    )
    assert scale == 0.0


def test_knob_stale_stops_even_if_cmd_fresh():
    """knob만 stale이어도 motor를 정지(0.0)한다."""
    scale = motor_speed_ratio(
        cmd_last_ns=T0,
        knob_last_ns=None,
        knob_pct=100,
        now_ns=T0,
        cmd_timeout_ns=CMD_TIMEOUT_NS,
        knob_timeout_ns=KNOB_TIMEOUT_NS,
        deadzone_pct=5,
    )
    assert scale == 0.0


def test_knob_time_reversal_stops():
    """미래 knob 시각(음수 age)이면 motor를 정지한다."""
    scale = motor_speed_ratio(
        cmd_last_ns=T0,
        knob_last_ns=T0 + sec_to_ns(1.0),
        knob_pct=100,
        now_ns=T0,
        cmd_timeout_ns=CMD_TIMEOUT_NS,
        knob_timeout_ns=KNOB_TIMEOUT_NS,
        deadzone_pct=5,
    )
    assert scale == 0.0


def test_knob_below_deadzone_stops():
    """knob이 fresh여도 deadzone 이하면 0을 반환한다."""
    scale = motor_speed_ratio(
        cmd_last_ns=T0,
        knob_last_ns=T0,
        knob_pct=3,
        now_ns=T0,
        cmd_timeout_ns=CMD_TIMEOUT_NS,
        knob_timeout_ns=KNOB_TIMEOUT_NS,
        deadzone_pct=5,
    )
    assert scale == 0.0
