"""주행 번호표(_nav_gen) — 늦은 취소가 새 goal 을 죽이지 않는지 본다."""
import threading
from types import SimpleNamespace

import pytest

mm = pytest.importorskip("vica_mission_manager.mission_manager_node")

from builtin_interfaces.msg import Time  # noqa: E402

from vica_mission_manager.mission_logic import (  # noqa: E402
    Destination,
    NAV_TREE_GUIDED,
    NAV_TREE_WAIT,
    Navigate,
    Pose2D,
    SpinInPlace,
    State,
)


class _FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(("info", msg))

    def warn(self, msg):
        self.lines.append(("warn", msg))

    def error(self, msg):
        self.lines.append(("error", msg))


class _FakeNavigator:
    def __init__(self, accept=True, task_done=True):
        self.accept = accept
        self.canceled = 0
        self.goals_sent = 0
        self.spins_sent = 0
        self.trees = []
        self.task_done = task_done
        self.calls = []

    def goToPose(self, goal, behavior_tree=''):  # noqa: N802 - nav2_simple_commander 이름
        self.goals_sent += 1
        self.trees.append(behavior_tree)
        self.calls.append('goal')
        return self.accept

    def spin(self, spin_dist):
        self.spins_sent += 1
        return self.accept

    def cancelTask(self):  # noqa: N802
        self.canceled += 1
        self.calls.append('cancel')
        self.task_done = True

    def isTaskComplete(self, timeout_sec=0.10):  # noqa: N802
        return self.task_done


class _FakeClock:
    class _Now:
        def to_msg(self):
            return Time()

    def now(self):
        return self._Now()


def _bare_node(gen=5, accept=True):
    """__init__ 없이 노드를 만든다 — 번호표 로직에 필요한 속성만 채운다."""
    node = mm.MissionManagerNode.__new__(mm.MissionManagerNode)
    node._nav_lock = threading.Lock()
    node._nav_lock_timeout_sec = 2.0
    node._nav_active = True
    node._nav_gen = gen
    node.navigator = _FakeNavigator(accept=accept)
    node._approach_bt = ""
    node._tree_files = {}
    node._guided_no_rail_bt = ""
    node._route_server_up = lambda: True
    node._nav2_ready = lambda: True
    node._task_bt = ""
    node._clear_before_next_nav = False
    node._nav_task_is_spin = False
    node._delivery_dest_id = None
    node.logic = SimpleNamespace(state=State.NAVIGATING)
    logger = _FakeLogger()
    node.get_logger = lambda: logger
    node.get_clock = lambda: _FakeClock()
    node._published_events = []
    node._publish_goal_event = lambda event, dest, reason="": (
        node._published_events.append(event)
    )
    return node


def _dest():
    return Destination(id="cafeteria", name="탕비실", pose=Pose2D(1.0, 2.0, 90.0))


class TestCancelNote:
    def test_cancel_runs_when_no_new_goal_started(self):
        """취소 스레드가 먼저 도는 정상 순서 — 옛 goal 을 취소한다."""
        node = _bare_node(gen=5)
        node._cancel_nav_blocking(5)
        assert node.navigator.canceled == 1

    def test_cancel_skips_when_new_goal_overtook(self):
        """새 goal 이 먼저 출발한 경합 순서 — 취소를 건너뛴다."""
        node = _bare_node(gen=6)
        node._cancel_nav_blocking(5)
        assert node.navigator.canceled == 0

    def test_cancel_note_carries_current_generation(self, monkeypatch):
        """_cancel_nav 가 스레드 쪽지에 '지금' 번호를 적는지 본다."""
        node = _bare_node(gen=5)
        recorded = {}

        class _FakeThread:
            def __init__(self, target=None, args=(), name="", daemon=False):
                recorded["target"] = target
                recorded["args"] = args

            def start(self):
                pass

        monkeypatch.setattr(mm.threading, "Thread", _FakeThread)
        node._cancel_nav(destination=None)
        assert recorded["args"] == (5,)
        assert node._nav_active is False


class TestGenerationBump:
    def test_accepted_goal_bumps_generation(self):
        node = _bare_node(gen=5, accept=True)
        node._start_nav(Navigate(destination=_dest()))
        assert node._nav_gen == 6
        assert node._nav_active is True
        assert node._published_events == ["goal_sent", "goal_accepted"]

    def test_rejected_goal_keeps_generation(self):
        """거부되면 옛 goal 이 그대로다 — 번호를 올리면 대기 중인 취소가"""
        node = _bare_node(gen=5, accept=False)

        class _FakeLogic:
            state = State.NAVIGATING

            def on_tick(self, now, status):
                return []

        node.logic = _FakeLogic()
        node._now = lambda: 0.0
        node._run_actions = lambda actions: None
        node._start_nav(Navigate(destination=_dest()))
        assert node._nav_gen == 5
        assert node._nav_active is False
        assert "goal_rejected" in node._published_events

    def test_accepted_spin_bumps_generation(self):
        """회전도 Nav2 task 다 — 늦은 취소가 회전을 죽여도 같은 결함이다."""
        node = _bare_node(gen=5, accept=True)
        node._start_spin(SpinInPlace(yaw_rad=1.57))
        assert node._nav_gen == 6


APPROACH_BT = "/share/vica_nav2/behavior_trees/vica_navigate_to_pose_approach.xml"


def _approach_dest():
    return Destination(id="approach:7", name="접근 대상", pose=Pose2D(1.0, 0.5, 30.0))


class TestBehaviorTreeSwitch:
    """사람 접근 goal 만 레일 없는 트리로 보낸다(2026-10-02 run60)."""

    def test_approach_goal_sends_the_approach_tree(self):
        node = _bare_node()
        node._approach_bt = APPROACH_BT
        node._start_nav(Navigate(destination=_approach_dest()))
        assert node.navigator.trees == [APPROACH_BT]
        assert node._task_bt == APPROACH_BT

    def test_destination_goal_keeps_the_default_tree(self):
        node = _bare_node()
        node._approach_bt = APPROACH_BT
        node._start_nav(Navigate(destination=_dest()))
        assert node.navigator.trees == [""]

    def test_switch_cancels_a_running_task_first(self):
        """레일 goal 이 아직 도는 중 접근 goal — 취소가 보내기보다 먼저다."""
        node = _bare_node()
        node._approach_bt = APPROACH_BT
        node.navigator.task_done = False
        node._start_nav(Navigate(destination=_approach_dest()))
        assert node.navigator.calls == ["cancel", "goal"]

    def test_switch_back_after_a_finished_task_does_not_cancel(self):
        node = _bare_node()
        node._approach_bt = APPROACH_BT
        node._task_bt = APPROACH_BT
        node._start_nav(Navigate(destination=_dest()))
        assert node.navigator.calls == ["goal"]
        assert node._task_bt == ""

    def test_same_tree_preempts_without_cancel(self):
        """접근 중 사람이 움직여 접근 goal 을 다시 보낼 때는 Nav2 선점을 그대로 쓴다."""
        node = _bare_node()
        node._approach_bt = APPROACH_BT
        node._task_bt = APPROACH_BT
        node.navigator.task_done = False
        node._start_nav(Navigate(destination=_approach_dest()))
        assert node.navigator.calls == ["goal"]

    def test_rejected_goal_keeps_the_old_tree(self):
        node = _bare_node(accept=False)
        node._approach_bt = APPROACH_BT

        class _FakeLogic:
            state = State.NAVIGATING

            def on_tick(self, now, status):
                return []

        node.logic = _FakeLogic()
        node._now = lambda: 0.0
        node._start_nav(Navigate(destination=_approach_dest()))
        assert node._task_bt == ""


GUIDED_BT = "/share/vica_nav2/behavior_trees/vica_navigate_to_pose_guided.xml"
GUIDED_NO_RAIL_BT = "/share/vica_nav2/behavior_trees/vica_navigate_to_pose_guided_no_rail.xml"
WAIT_BT = "/share/vica_nav2/behavior_trees/vica_navigate_to_pose_wait_spot.xml"


class TestGuidedAndWaitTrees:
    """Navigate.tree 가 고른 트리 파일을 보낸다(2026-10-07 대기 장소)."""

    def _node(self, state=State.NAVIGATING):
        node = _bare_node()
        node._approach_bt = APPROACH_BT
        node._tree_files = {NAV_TREE_GUIDED: GUIDED_BT, NAV_TREE_WAIT: WAIT_BT}
        node._guided_no_rail_bt = GUIDED_NO_RAIL_BT
        node.logic = SimpleNamespace(state=state)
        return node

    def test_guided_goal_sends_the_guided_tree(self):
        node = self._node()
        node._start_nav(Navigate(destination=_dest(), tree=NAV_TREE_GUIDED))
        assert node.navigator.trees == [GUIDED_BT]
        assert node._published_events == ["goal_sent", "goal_accepted"]

    def test_guided_goal_without_rail_server_uses_the_no_rail_tree(self):
        node = self._node()
        node._route_server_up = lambda: False
        node._start_nav(Navigate(destination=_dest(), tree=NAV_TREE_GUIDED))
        assert node.navigator.trees == [GUIDED_NO_RAIL_BT]

    def test_wait_tree_ignores_the_rail_server(self):
        node = self._node(state=State.MOVING_TO_WAIT_SPOT)
        node._route_server_up = lambda: False
        node._start_nav(Navigate(destination=_dest(), tree=NAV_TREE_WAIT))
        assert node.navigator.trees == [WAIT_BT]

    def test_default_goal_keeps_the_default_tree(self):
        node = self._node()
        node._start_nav(Navigate(destination=_dest()))
        assert node.navigator.trees == [""]

    def test_approach_id_wins_over_the_tree_kind(self):
        node = self._node()
        node._start_nav(Navigate(destination=_approach_dest(), tree=NAV_TREE_GUIDED))
        assert node.navigator.trees == [APPROACH_BT]

    def test_moving_to_wait_spot_is_quiet_to_the_app(self):
        node = self._node(state=State.MOVING_TO_WAIT_SPOT)
        node._start_nav(Navigate(destination=_dest(), tree=NAV_TREE_WAIT))
        assert node.navigator.trees == [WAIT_BT]
        assert node._published_events == []

    def test_switch_from_guided_to_wait_cancels_first(self):
        node = self._node(state=State.MOVING_TO_WAIT_SPOT)
        node._task_bt = GUIDED_BT
        node.navigator.task_done = False
        node._start_nav(Navigate(destination=_dest(), tree=NAV_TREE_WAIT))
        assert node.navigator.calls == ["cancel", "goal"]


class TestNav2NotReady:
    """Nav2 액션 서버가 없으면 goal 을 보내지 않는다(7789aed) — goToPose 무한 대기 방지."""

    def test_goal_is_not_sent_and_reported_as_rejected(self):
        node = _bare_node()
        node._nav2_ready = lambda: False

        class _FakeLogic:
            state = State.NAVIGATING

            def on_tick(self, now, status):
                return []

        node.logic = _FakeLogic()
        node._now = lambda: 0.0
        node._start_nav(Navigate(destination=_dest()))
        assert node.navigator.goals_sent == 0
        assert node._published_events == ["goal_rejected"]

class TestIdleCancelSync:
    """취소를 눌렀는데 취소할 주행이 없을 때 앱에 사실을 알리는지."""

    def _node_with_logic(self, state):
        node = _bare_node()
        node._map_id = 'm1'
        node._publish_goal_event = (
            mm.MissionManagerNode._publish_goal_event.__get__(node)
        )

        class _FakeLogic:
            def __init__(self, st):
                self.state = st

            def on_app_cancel(self, now):
                from vica_mission_manager.mission_logic import GateReason
                return [], GateReason.OK

        node.logic = _FakeLogic(state)
        node._now = lambda: 0.0
        node._published = []

        class _FakePub:
            def __init__(self, sink):
                self.sink = sink

            def publish(self, msg):
                self.sink.append(msg.data)

        node.pub_goal_event = _FakePub(node._published)
        return node

    def test_idle_cancel_publishes_sync_event(self):
        from vica_mission_manager.mission_logic import State
        node = self._node_with_logic(State.IDLE)
        node._run_mission_command('cancel')
        assert any('state_idle' in data for data in node._published)

    def test_driving_cancel_does_not_publish_sync_event(self):
        """주행 중 취소는 종전 경로가 goal_canceled 를 낸다 — 두 번 알리면 안 된다."""
        from vica_mission_manager.mission_logic import State
        node = self._node_with_logic(State.NAVIGATING)
        node._run_mission_command('cancel')
        assert not any('state_idle' in data for data in node._published)

    def test_sync_event_survives_missing_destination(self):
        """목적지 없이 발행해도 죽지 않는다 — 이 이벤트에는 goal 이 없다."""
        from vica_mission_manager.mission_logic import State
        node = self._node_with_logic(State.IDLE)
        node._run_mission_command('cancel')
        import json
        payload = json.loads(node._published[-1])
        assert payload['event'] == 'state_idle'
        assert payload['name'] == ''
        assert payload['x'] == 0.0
