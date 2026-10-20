"""배송 도착 표시 — 배송 요청으로 간 목적지의 도착에만 "delivery": true 를 싣는지 본다."""
import json
import threading
from types import SimpleNamespace

import pytest

mm = pytest.importorskip("vica_mission_manager.mission_manager_node")

from vica_mission_manager.mission_logic import (  # noqa: E402
    Destination,
    GateReason,
    Navigate,
    Pose2D,
    State,
)

DELIVERY_ID = "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa"
OTHER_ID = "bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb"


def _dest(dest_id=DELIVERY_ID, name="408호"):
    return Destination(id=dest_id, name=name, pose=Pose2D(1.0, 2.0, 90.0))


class _Pub:
    def __init__(self):
        self.payloads = []

    def publish(self, msg):
        self.payloads.append(json.loads(msg.data))


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(mm, "apply_goal_event", lambda *a, **k: False)
    n = mm.MissionManagerNode.__new__(mm.MissionManagerNode)
    n._ledger = object()
    n._map_id = "m1"
    n.pub_goal_event = _Pub()
    n._delivery_dest_id = None
    return n


def _last(n):
    return n.pub_goal_event.payloads[-1]


class TestArrivalFlag:
    def test_delivery_arrival_carries_flag_once(self, node):
        node._delivery_dest_id = DELIVERY_ID
        node._publish_goal_event("goal_succeeded", _dest())
        assert _last(node)["delivery"] is True
        node._publish_goal_event("goal_succeeded", _dest())
        assert "delivery" not in _last(node)

    def test_plain_arrival_has_no_flag(self, node):
        node._publish_goal_event("goal_succeeded", _dest())
        assert "delivery" not in _last(node)

    def test_other_destination_arrival_has_no_flag(self, node):
        node._delivery_dest_id = DELIVERY_ID
        node._publish_goal_event("goal_succeeded", _dest(OTHER_ID, "407호"))
        assert "delivery" not in _last(node)
        assert node._delivery_dest_id == DELIVERY_ID

    def test_pause_then_arrival_keeps_flag(self, node):
        node._delivery_dest_id = DELIVERY_ID
        node._publish_goal_event("goal_paused", _dest())
        node._publish_goal_event("goal_sent", _dest())
        node._publish_goal_event("goal_succeeded", _dest())
        assert _last(node)["delivery"] is True

    @pytest.mark.parametrize("ending", ["goal_canceled", "goal_failed", "goal_rejected"])
    def test_ended_delivery_is_forgotten(self, node, ending):
        node._delivery_dest_id = DELIVERY_ID
        node._publish_goal_event(ending, _dest())
        assert "delivery" not in _last(node)
        node._publish_goal_event("goal_succeeded", _dest())
        assert "delivery" not in _last(node)


class _FakeNavigator:
    def goToPose(self, goal, behavior_tree=''):  # noqa: N802 - nav2_simple_commander 이름
        return True

    def isTaskComplete(self, timeout_sec=0.10):  # noqa: N802
        return True


class _FakeClock:
    class _Now:
        def to_msg(self):
            from builtin_interfaces.msg import Time
            return Time()

    def now(self):
        return self._Now()


def _nav_node(n):
    n._nav_lock = threading.Lock()
    n._nav_lock_timeout_sec = 2.0
    n._nav_active = False
    n._nav_gen = 0
    n.navigator = _FakeNavigator()
    n._approach_bt = ""
    n._tree_files = {}
    n._guided_no_rail_bt = ""
    n._route_server_up = lambda: True
    n._nav2_ready = lambda: True
    n._task_bt = ""
    n._clear_before_next_nav = False
    n._nav_task_is_spin = False
    n.logic = SimpleNamespace(state=State.NAVIGATING)
    log = SimpleNamespace(info=lambda m: None, warn=lambda m: None, error=lambda m: None)
    n.get_logger = lambda: log
    n.get_clock = lambda: _FakeClock()
    n._publish_goal_event = lambda event, dest, reason="", extra=None: None
    return n


class TestDepartureClears:
    def test_leaving_for_another_place_ends_the_delivery(self, node):
        _nav_node(node)
        node._delivery_dest_id = DELIVERY_ID
        node._start_nav(Navigate(destination=_dest(OTHER_ID, "407호")))
        assert node._delivery_dest_id is None

    def test_resending_the_delivery_keeps_it(self, node):
        _nav_node(node)
        node._delivery_dest_id = DELIVERY_ID
        node._start_nav(Navigate(destination=_dest()))
        assert node._delivery_dest_id == DELIVERY_ID


class TestRequestRemembers:
    def _request_node(self, node, accept):
        dest = _dest()
        node.destinations = {DELIVERY_ID: dest}
        node.map_bounds = None
        node._nav2_ready = lambda: True
        node._now = lambda: 0.0
        node._check_destination_request = lambda req, resp: DELIVERY_ID
        node.logic = SimpleNamespace(
            state=State.IDLE,
            on_app_destination=lambda *a, **k: (["go"], GateReason.OK))

        def run(actions):
            node._nav_active = accept
        node._run_actions = run
        node.get_logger = lambda: SimpleNamespace(info=lambda m: None, warn=lambda m: None)
        return SimpleNamespace(map_id="m1"), SimpleNamespace(accepted=False, message="")

    def test_accepted_delivery_is_remembered(self, node):
        req, resp = self._request_node(node, accept=True)
        node._handle_destination_request(req, resp, allow_private=True, label="배송",
                                         is_delivery=True)
        assert node._delivery_dest_id == DELIVERY_ID

    def test_rejected_delivery_is_not_remembered(self, node):
        req, resp = self._request_node(node, accept=False)
        node._handle_destination_request(req, resp, allow_private=True, label="배송",
                                         is_delivery=True)
        assert node._delivery_dest_id is None

    def test_plain_request_forgets_a_delivery(self, node):
        req, resp = self._request_node(node, accept=True)
        node._delivery_dest_id = OTHER_ID
        node._handle_destination_request(req, resp, allow_private=False, label="목적지")
        assert node._delivery_dest_id is None
