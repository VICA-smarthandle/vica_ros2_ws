import threading
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal

from vica_safety.app_emergency_node import (
    AppEmergencyNode,
    TIP_RESET_REJECT_MESSAGE,
    build_app_state,
    has_active_navigation_goals,
    primary_cause,
    reset_rejected_by_tip,
)
from vica_safety.auto_recovery import AutoRecoveryPolicy
from vica_safety.reset_sequence import ResetSequence

NOW_NS = 1_000_000_000_000
OLD_NS = NOW_NS - 60_000_000_000


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


def test_app_state_carries_estop_sources_for_the_app():
    """앱이 원인을 볼 수 있어야 한다(2026-08-31)."""
    payload = build_app_state(
        app_active=False,
        emergency_active=True,
        safety_state="ESTOP_ACTIVE",
        message="physical estop",
        timestamp="2026-07-22T00:00:00+00:00",
        sources=("physical_f1",),
    )

    assert payload["sources"] == ["physical_f1"]


def test_app_state_without_sources_stays_backward_compatible():
    """sources를 안 주는 호출부도 그대로 동작한다."""
    payload = build_app_state(
        app_active=True,
        emergency_active=True,
        safety_state="ESTOP_ACTIVE",
        message="app estop",
        timestamp="2026-07-22T00:00:00+00:00",
    )

    assert payload["sources"] == []


def test_app_state_marks_cleared_waiting_supervisor_as_resettable():
    payload = build_app_state(
        app_active=False,
        emergency_active=False,
        safety_state="ESTOP_RELEASED_WAIT_RESET",
        message="waiting",
        timestamp="2026-07-22T00:00:00+00:00",
    )

    assert payload["reset_allowed"] is True


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
    payload = build_app_state(
        app_active=False,
        emergency_active=False,
        safety_state="READY_TO_GO",
        message="관리자 reset",
        timestamp="2026-08-28T00:00:00+00:00",
    )

    assert payload["auto_recovered"] is False


class _Logger:
    def info(self, text):
        del text

    warn = warning = error = info


class PopupHarness:
    """The real popup-message methods over the minimum node state."""

    estop_sources_callback = AppEmergencyNode.estop_sources_callback
    update_cause_message = AppEmergencyNode.update_cause_message
    reject_reset = AppEmergencyNode.reject_reset

    def __init__(self, last_message="Safety 상태 확인 대기 중"):
        self.state_condition = threading.Condition()
        self.auto_recovery = AutoRecoveryPolicy(settle_ns=1_000_000_000)
        self.last_estop_sources = ()
        self.last_cause = None
        self.event_message = ""
        self.last_message = last_message
        self.sequence = ResetSequence()

    def now_ns(self):
        return NOW_NS

    def get_logger(self):
        return _Logger()

    def sources(self, text):
        self.estop_sources_callback(SimpleNamespace(data=text))


AUTO_RECOVERED = "통신이 복구되어 비상정지를 자동 해제했습니다 (원인: motor_can_waiting)"


def test_physical_button_replaces_the_previous_message():
    h = PopupHarness(last_message=AUTO_RECOVERED)
    h.sources("physical_f1")
    assert h.last_message == "비상정지 버튼이 눌렸습니다."


def test_voice_and_comm_causes_have_their_own_line():
    h = PopupHarness()
    h.sources("voice")
    assert h.last_message == "이용자가 음성으로 멈춤을 요청해 비상정지되었습니다."
    h = PopupHarness()
    h.sources("motor_can_stale")
    assert h.last_message == "모터 드라이버 통신이 끊겨 비상정지되었습니다."
    h = PopupHarness()
    h.sources("physical_stale")
    assert h.last_message == "모터 드라이버 통신이 끊겨 비상정지되었습니다."


def test_tip_over_message_wins_over_other_causes():
    h = PopupHarness()
    h.sources("motor_can_stale")
    h.sources("motor_can_stale,tipped")
    assert h.last_message == "넘어짐이 감지되어 비상정지되었습니다."
    assert primary_cause(("physical_f1", "tipped", "voice")) == "tipped"


def test_app_button_and_boot_wait_keep_their_message():
    h = PopupHarness(last_message="앱 E-stop이 활성화되었습니다.")
    h.sources("app")
    assert h.last_message == "앱 E-stop이 활성화되었습니다."
    h = PopupHarness()
    h.sources("motor_can_waiting,physical_waiting")
    assert h.last_message == "Safety 상태 확인 대기 중"


def test_reset_rejection_is_not_overwritten_by_the_same_cause():
    h = PopupHarness()
    h.sources("physical_f1")
    h.reject_reset(SimpleNamespace(), "estop_reset", "active sources: physical_f1")
    rejected = h.last_message
    assert rejected.startswith("Safety reset 거부:")
    h.sources("physical_f1")
    assert h.last_message == rejected


def test_reset_before_standing_up_says_what_to_do():
    h = PopupHarness()
    h.sources("tipped")
    response = h.reject_reset(
        SimpleNamespace(), "estop_reset", "active sources: tipped")
    assert response.success is False
    assert response.message == TIP_RESET_REJECT_MESSAGE
    assert h.last_message == TIP_RESET_REJECT_MESSAGE


def test_tip_rejection_text_goes_back_after_standing_up():
    h = PopupHarness()
    h.sources("tipped")
    h.reject_reset(SimpleNamespace(), "estop_reset", "active sources: tipped")
    h.sources("")
    assert h.last_message == "넘어짐이 감지되어 비상정지되었습니다."


def test_released_button_after_rejection_shows_the_cause_again():
    h = PopupHarness()
    h.sources("physical_f1")
    h.reject_reset(SimpleNamespace(), "estop_reset", "active sources: physical_f1")
    h.sources("")
    assert h.last_message == "비상정지 버튼이 눌렸습니다."


def test_reset_success_message_is_kept_when_sources_stay_empty():
    h = PopupHarness()
    h.sources("physical_f1")
    h.sources("")
    h.last_message = "Safety reset 완료: Nav2 Goal 없음, 중앙 래치 해제, Supervisor READY_TO_GO 확인"
    h.sources("")
    assert h.last_message.startswith("Safety reset 완료")


def test_tip_rejection_is_recognised_only_for_the_latch_step():
    assert reset_rejected_by_tip("estop_reset", "active sources: tipped") is True
    assert reset_rejected_by_tip(
        "estop_reset", "active sources: physical_waiting,tipped") is True
    assert reset_rejected_by_tip("estop_reset", "active sources: physical_f1") is False
    assert reset_rejected_by_tip("nav_goal_check", "tipped") is False
    assert reset_rejected_by_tip("estop_reset", "estop_reset service is unavailable") is False


class ActivateHarness:
    """handle_activate 의 순서만 본다: 앱 원인 → 본문 → 상태 즉시 발행 → Nav2 확인."""

    handle_activate = AppEmergencyNode.handle_activate

    def __init__(self):
        self.app_active = False
        self.last_message = "Safety reset 완료: 옛 문장"
        self.calls = []

    def publish_app_estop(self):
        self.calls.append(("app_estop", self.app_active))

    def publish_state(self):
        self.calls.append(("state", self.app_active, self.last_message))

    def ensure_no_active_nav_goal(self):
        self.calls.append(("nav_check",))
        return True, "no active Nav2 goal"

    def get_logger(self):
        return _Logger()


def test_app_button_state_goes_out_before_the_nav2_check():
    h = ActivateHarness()
    response = h.handle_activate(None, SimpleNamespace())
    assert h.calls[0] == ("app_estop", True)
    assert h.calls[1] == ("state", True, "앱 E-stop이 활성화되었습니다.")
    assert h.calls[2] == ("nav_check",)
    assert response.success is True
