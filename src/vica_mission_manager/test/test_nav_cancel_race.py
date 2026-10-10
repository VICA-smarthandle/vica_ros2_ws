"""주행 번호표(_nav_gen) — 늦은 취소가 새 goal 을 죽이지 않는지 본다.

배경(2026-09-01): 취소는 콜백을 막지 않으려고 별도 스레드에 맡긴다(8/21 수리).
그런데 앱 전권화(8/31, dev)가 주행 중 새 목적지를 [취소 -> 즉시 출발] 로
처리하면서, 취소 스레드가 lock 을 잡기 전에 새 goal 이 먼저 나가는 순서가
생겼다. cancelTask() 는 '지금 잡혀 있는' task 를 취소하므로 그대로 부르면
방금 보낸 새 goal 이 취소된다 — 앱 명령이 조용히 사라지는 증상.

이 시험은 노드를 rclpy 없이 __new__ 로 맨몸 생성해 순수 로직만 본다.
mission_manager_node 는 rclpy 를 import 하므로 없는 환경(개발 노트북)에서는
모듈 전체를 건너뛴다. 젯슨 colcon test 에서 실제로 돈다.
"""
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
        self.trees = []           # goToPose 가 받은 behavior_tree 순서
        self.task_done = task_done
        self.calls = []           # 'cancel' / 'goal' 순서

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
        self.task_done = True     # 취소 응답 뒤 task 가 끝난다

    def isTaskComplete(self):  # noqa: N802
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
    node._approach_bt = ""       # 사람 접근 트리 끔 — 종전 동작
    node._tree_files = {}        # 안내·대기 장소 트리 끔 — 종전 동작(2026-10-07)
    node._guided_no_rail_bt = ""
    node._route_server_up = lambda: True
    # Nav2 준비 확인(7789aed)은 액션 서버를 본다 — 가짜 노드에는 없으니 준비됨으로 둔다.
    node._nav2_ready = lambda: True
    node._task_bt = ""
    # 출발 전 local costmap 비우기(2026-10-09)는 끈 상태로 둔다 — 시험은
    # test_clear_before_departure.py 에 있다.
    node._clear_before_next_nav = False
    node._nav_task_is_spin = False
    node._delivery_dest_id = None   # 배송 도착 표시(2026-10-10) — 배송 아님
    node.logic = SimpleNamespace(state=State.NAVIGATING)   # 대기 장소로 가는 중이 아니다
    logger = _FakeLogger()
    node.get_logger = lambda: logger  # 클래스 메서드를 인스턴스 속성으로 가린다
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
        """새 goal 이 먼저 출발한 경합 순서 — 취소를 건너뛴다.

        옛 goal 은 Nav2 선점으로 이미 내려갔고, 여기서 cancelTask() 를 부르면
        방금 보낸 새 goal 이 죽는다. 이 파일에서 가장 중요한 시험이다.
        """
        node = _bare_node(gen=6)  # 쪽지(5) 이후 새 goal 이 번호를 올렸다
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
                pass  # 시험에서는 돌리지 않는다 — 쪽지 내용만 본다

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
        """거부되면 옛 goal 이 그대로다 — 번호를 올리면 대기 중인 취소가
        멀쩡한 옛 goal 을 두고 그냥 돌아가 버린다."""
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
    """사람 접근 goal 만 레일 없는 트리로 보낸다(2026-10-02 run60).

    Humble bt_navigator 는 실행 중 goal 과 다른 트리 파일의 선점을 거절하므로,
    트리가 바뀔 때 앞 task 가 돌고 있으면 먼저 취소하고 끝난 뒤 보낸다.
    """

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
        node._task_bt = APPROACH_BT          # 접근을 마쳤다(task 끝남)
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
        # 레일 파일이 없는 지도 — 레일 안내 트리는 route_server 가 없어 통째로 실패한다.
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
    """취소를 눌렀는데 취소할 주행이 없을 때 앱에 사실을 알리는지.

    2026-09-02 실기: 홈 도착 뒤 앱이 계속 '주행 중'으로 굳었다. 로봇은 이미
    IDLE 이라 취소가 조용히 수락되고 아무 이벤트도 안 나가, 앱의 유령 표시를
    되맞출 길이 없었다. 이제 대기 사실을 한 번 알린다.
    """

    def _node_with_logic(self, state):
        node = _bare_node()
        node._map_id = 'm1'
        # _bare_node 는 _publish_goal_event 를 이름만 기록하는 가짜로 덮는다.
        # 여기서는 **실제로 나가는 JSON** 을 봐야 하므로 진짜 메서드를 되돌린다.
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
