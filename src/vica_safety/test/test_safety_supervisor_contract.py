from geometry_msgs.msg import Twist
import pytest

from vica_safety.safety_gate import SafetyState
from vica_safety.safety_supervisor_node import (
    describe_safety_transition,
    is_zero_twist,
    limited_twist,
)


def test_zero_twist_checks_every_axis():
    msg = Twist()
    assert is_zero_twist(msg) is True

    msg.angular.x = 0.1
    assert is_zero_twist(msg) is False


def test_limited_twist_only_forwards_differential_drive_axes():
    msg = Twist()
    msg.linear.x = 2.0
    msg.linear.y = 0.4
    msg.angular.x = 0.3
    msg.angular.z = -3.0

    out = limited_twist(msg, max_linear_mps=1.0, max_angular_radps=2.0)

    assert out.linear.x == pytest.approx(1.0)
    assert out.angular.z == pytest.approx(-2.0)
    assert out.linear.y == 0.0
    assert out.angular.x == 0.0


@pytest.mark.parametrize(
    "state,severity,marker",
    [
        (SafetyState.ESTOP_ACTIVE, "error", "[ESTOP ACTIVE]"),
        (SafetyState.FAULT, "error", "[FAULT]"),
        (SafetyState.ESTOP_RELEASED_WAIT_RESET, "warn", "[WAIT RESET]"),
        (SafetyState.READY_TO_GO, "info", "[SAFETY READY]"),
        (SafetyState.RUNNING, "info", "[RUNNING]"),
        (SafetyState.IDLE, "warn", "[WAIT RESET]"),
    ],
)
def test_safety_transition_has_expected_severity(state, severity, marker):
    assert describe_safety_transition(state) == (severity, marker)
