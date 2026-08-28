import threading
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal

from vica_safety.app_emergency_node import (
    AppEmergencyNode,
    build_app_state,
    has_active_navigation_goals,
)

# 모든 harness 시각은 정수 나노초(steady clock) 기준이다.
NOW_NS = 1_000_000_000_000
OLD_NS = NOW_NS - 60_000_000_000  # 60초 전


class UnexpectedCancelClient:
    """Fail a test if an idle or unknown state reaches the cancel service."""

    def wait_for_service(self, timeout_sec):
        del timeout_sec
        raise AssertionError("cancel service must not be called")


class NavigationCheckHarness:
    """Minimal state used to exercise the real navigation check method."""

    def __init__(self, statuses, last_status_ns):
        self.nav_statuses = statuses
        self.last_nav_status_ns = last_status_ns
        self.state_condition = threading.Condition()
        self.cancel_client = UnexpectedCancelClient()

    def now_ns(self):
        return NOW_NS


class RecordingCancelClient:
    """Record whether an active-goal check reached the cancel service."""

    def __init__(self):
        self.wait_calls = 0

    def wait_for_service(self, timeout_sec):
        del timeout_sec
        self.wait_calls += 1
        return True


class AvailabilityCancelClient:
    """Report whether the Nav2 action cancel service currently exists."""

    def __init__(self, available):
        self.available = available
        self.wait_calls = 0

    def wait_for_service(self, timeout_sec):
        del timeout_sec
        self.wait_calls += 1
        return self.available


class ActiveNavigationHarness(NavigationCheckHarness):
    """Simulate an active goal becoming terminal after cancel."""

    def __init__(self):
        super().__init__([GoalStatus.STATUS_EXECUTING], NOW_NS)
        self.cancel_client = RecordingCancelClient()
        self.call_count = 0

    def call_sync(self, client, request):
        del client, request
        self.call_count += 1
        return SimpleNamespace(
            return_code=CancelGoal.Response.ERROR_NONE,
            goals_canceling=[SimpleNamespace()],
        )

    def wait_for_condition(self, predicate):
        self.nav_statuses = [GoalStatus.STATUS_CANCELED]
        # cancel 요청 시각(now_ns) 이후에 도착한 새 terminal 상태를 모사.
        self.last_nav_status_ns = self.now_ns() + 1
        return predicate()


def test_active_nav_statuses_include_canceling():
    for status in (
        GoalStatus.STATUS_ACCEPTED,
        GoalStatus.STATUS_EXECUTING,
        GoalStatus.STATUS_CANCELING,
    ):
        assert has_active_navigation_goals([status]) is True


def test_terminal_nav_statuses_are_not_active():
    statuses = [
        GoalStatus.STATUS_UNKNOWN,
        GoalStatus.STATUS_SUCCEEDED,
        GoalStatus.STATUS_CANCELED,
        GoalStatus.STATUS_ABORTED,
    ]

    assert has_active_navigation_goals(statuses) is False


def test_fresh_idle_nav_status_skips_cancel_service():
    harness = NavigationCheckHarness([], NOW_NS)

    accepted, message = AppEmergencyNode.ensure_no_active_nav_goal(harness)

    assert accepted is True
    assert message == "no active Nav2 goal"


def test_old_terminal_nav_status_skips_cancel_service():
    harness = NavigationCheckHarness(
        [GoalStatus.STATUS_CANCELED],
        OLD_NS,
    )

    accepted, message = AppEmergencyNode.ensure_no_active_nav_goal(harness)

    assert accepted is True
    assert message == "no active Nav2 goal"


def test_nav2_never_seen_and_server_absent_skips_goal_check():
    harness = NavigationCheckHarness([], None)
    harness.cancel_client = AvailabilityCancelClient(False)

    accepted, message = AppEmergencyNode.ensure_no_active_nav_goal(harness)

    assert accepted is True
    assert message == "Nav2 is not running; goal check skipped"
    assert harness.cancel_client.wait_calls == 1


def test_nav2_server_present_without_status_skips_goal_check():
    harness = NavigationCheckHarness([], None)
    harness.cancel_client = AvailabilityCancelClient(True)

    accepted, message = AppEmergencyNode.ensure_no_active_nav_goal(harness)

    assert accepted is True
    assert message == "Nav2 has no goal status history; goal check skipped"
    assert harness.cancel_client.wait_calls == 1


def test_active_nav_goal_is_canceled_and_confirmed_terminal():
    harness = ActiveNavigationHarness()

    accepted, message = AppEmergencyNode.ensure_no_active_nav_goal(harness)

    assert accepted is True
    assert message == "all Nav2 goals canceled"
    assert harness.cancel_client.wait_calls == 1
    assert harness.call_count == 1


def test_old_active_nav_goal_is_still_canceled_and_confirmed_terminal():
    harness = ActiveNavigationHarness()
    harness.last_nav_status_ns = OLD_NS

    accepted, message = AppEmergencyNode.ensure_no_active_nav_goal(harness)

    assert accepted is True
    assert message == "all Nav2 goals canceled"
    assert harness.cancel_client.wait_calls == 1
    assert harness.call_count == 1


def test_old_active_status_accepts_fresh_cancel_response_with_no_goals():
    harness = ActiveNavigationHarness()
    harness.last_nav_status_ns = OLD_NS

    def no_active_goals(client, request):
        del client, request
        return SimpleNamespace(
            return_code=CancelGoal.Response.ERROR_NONE,
            goals_canceling=[],
        )

    harness.call_sync = no_active_goals

    accepted, message = AppEmergencyNode.ensure_no_active_nav_goal(harness)

    assert accepted is True
    assert message == "no active Nav2 goal at cancel time"
    assert harness.cancel_client.wait_calls == 1


def test_app_state_active_uses_authoritative_emergency_state():
    payload = build_app_state(
        app_active=False,
        emergency_active=True,
        safety_state="ESTOP_ACTIVE",
        message="physical estop",
        timestamp="2026-07-22T00:00:00+00:00",
    )

    assert payload["active"] is True
    assert payload["app_active"] is False
    assert payload["emergency_active"] is True
    assert payload["safety_state"] == "ESTOP_ACTIVE"
    assert payload["reset_allowed"] is False


def test_app_state_marks_cleared_waiting_supervisor_as_resettable():
    payload = build_app_state(
        app_active=False,
        emergency_active=False,
        safety_state="ESTOP_RELEASED_WAIT_RESET",
        message="waiting",
        timestamp="2026-07-22T00:00:00+00:00",
    )

    assert payload["reset_allowed"] is True


# ---------------------------------------------------------------------------
# 자동 복구 표시
# ---------------------------------------------------------------------------
#
# 앱은 원인 문자열을 보지 않는다. 이 JSON이 앱의 유일한 창구이므로, 로봇이
# 스스로 풀렸다는 사실도 여기에 실어야 관리자가 화면에서 알 수 있다.


def test_app_state_reports_automatic_recovery():
    payload = build_app_state(
        app_active=False,
        emergency_active=False,
        safety_state="READY_TO_GO",
        message="통신 복구로 자동 해제",
        timestamp="2026-08-28T00:00:00+00:00",
        auto_recovered=True,
    )

    assert payload["auto_recovered"] is True


def test_app_state_defaults_to_manual_recovery():
    # 기존 호출부(인자 5개)는 그대로 동작해야 한다.
    payload = build_app_state(
        app_active=False,
        emergency_active=False,
        safety_state="READY_TO_GO",
        message="관리자 reset",
        timestamp="2026-08-28T00:00:00+00:00",
    )

    assert payload["auto_recovered"] is False
