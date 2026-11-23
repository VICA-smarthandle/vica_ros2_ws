"""도킹 — 미션 노드 쪽 배선(2026-11-19 독립 검토 반영)."""
import threading
from types import SimpleNamespace

import pytest

mm = pytest.importorskip("vica_mission_manager.mission_manager_node")

from vica_mission_manager.dock_flow import CALL_CANCEL, CALL_UNDOCK, DockFlow  # noqa: E402
from vica_mission_manager.dock_spot import DockSpot  # noqa: E402
from vica_mission_manager.mission_logic import (  # noqa: E402
    Destination, MissionLogic, Navigate, Pose2D, SpinInPlace, State,
)


class _Logger:
    def info(self, msg):
        pass

    warn = error = info


class _Navigator:
    def __init__(self):
        self.goals = []
        self.spins = 0

    def goToPose(self, goal, behavior_tree=''):  # noqa: N802
        self.goals.append((goal.pose.position.x, goal.pose.position.y))
        return True

    def spin(self, spin_dist):
        self.spins += 1
        return True

    def isTaskComplete(self, timeout_sec=0.10):  # noqa: N802
        return False


class _Clock:
    class _Now:
        nanoseconds = 100 * 10**9

        def to_msg(self):
            from builtin_interfaces.msg import Time
            return Time()

    def now(self):
        return self._Now()


class _DockClient:
    def __init__(self, calls, name):
        self._calls, self._name = calls, name

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self._calls.append(self._name)
        return SimpleNamespace(add_done_callback=lambda cb: None)


DEST_A = Destination(id="a", name="창구", pose=Pose2D(1.0, 2.0, 0.0))
DEST_B = Destination(id="b", name="화장실", pose=Pose2D(5.0, 6.0, 90.0))


def _node():
    node = mm.MissionManagerNode.__new__(mm.MissionManagerNode)
    node._nav_lock = threading.Lock()
    node._nav_lock_timeout_sec = 2.0
    node._nav_active = False
    node._nav_gen = 1
    node.navigator = _Navigator()
    node._approach_bt = ""
    node._tree_files = {}
    node._guided_no_rail_bt = ""
    node._route_server_up = lambda: True
    node._nav2_ready = lambda: True
    node._task_bt = ""
    node._clear_before_next_nav = False
    node._nav_task_is_spin = False
    node._delivery_dest_id = None
    node._estop_active = False
    node.logic = MissionLogic()
    logger = _Logger()
    node.get_logger = lambda: logger
    node.get_clock = lambda: _Clock()
    node.events = []
    node._publish_goal_event = lambda event, dest, reason="", extra=None: node.events.append(event)
    node._publish_robot_state = lambda: None
    node._publish_nav_speed_limit = lambda percent: None
    node._question_safely = lambda fn, *a: None
    node.pub_tts = SimpleNamespace(publish=lambda m: None)
    node.pub_listen_request = SimpleNamespace(publish=lambda m: None)
    node.destinations = {"a": DEST_A, "b": DEST_B}
    node.map_bounds = None
    node._map_id = "m"
    node._check_destination_request = lambda request, response: request.destination_id
    node._dock = DockFlow()
    node._dock_lock = threading.RLock()
    node._dock_far_check_at = 0.0
    node.dock_calls = []
    node._dock_clients = {n: _DockClient(node.dock_calls, n) for n in ("start", "undock", "cancel")}
    return node


def _request(dest_id):
    return SimpleNamespace(destination_id=dest_id, map_id="m")


def _response():
    return SimpleNamespace(accepted=None, message="")


def _docked(node):
    node._dock_apply(node._dock.on_node_state(
        {"seq": 1, "state": "docked", "near_dock": True, "last": {}}))


def test_drive_from_inside_dock_is_accepted_and_held():
    """도크 안에서 보낸 원격 주행: 먼저 나오고(undock 호출) Nav2 로는 아직 안 나간다. 응답은 수락이다."""
    node = _node()
    _docked(node)
    res = node._handle_destination_request(_request("a"), _response(), False, "목적지", is_delivery=True)
    assert res.accepted is True
    assert node.dock_calls == [CALL_UNDOCK]
    assert node.navigator.goals == []
    assert node._dock.held.destination.id == "a"
    assert node._delivery_dest_id == "a"


def test_second_destination_while_leaving_swaps_held_without_stopping():
    """나오는 중에 다른 곳으로 다시 보냄: 나오기를 멈추지 않고(cancel 호출 없음) 잡아 둔 주행만 바꾼다."""
    node = _node()
    _docked(node)
    node._handle_destination_request(_request("a"), _response(), False, "목적지")
    res = node._handle_destination_request(_request("b"), _response(), False, "목적지")
    assert res.accepted is True
    assert node.dock_calls == [CALL_UNDOCK]
    assert node.navigator.goals == []
    assert node._dock.phase == "leaving" and node._dock.held.destination.id == "b"
    node._dock_apply(node._dock.on_node_state(
        {"seq": 2, "state": "idle", "near_dock": False,
         "last": {"action": "undock", "result": "success", "reason": ""}}))
    assert node.navigator.goals == [(5.0, 6.0)]


def test_drive_request_refused_while_docking():
    """가는 중·들어가는 중에는 원격 주행을 받지 않는다 — 로직·Nav2 를 건드리지 않고 거절한다."""
    node = _node()
    node._dock.phase, node._dock.mode = "entering", "docking"
    res = node._handle_destination_request(_request("a"), _response(), False, "목적지")
    assert res.accepted is False and "도킹 중지" in res.message
    assert node.logic.state == State.IDLE and node.navigator.goals == [] and node.dock_calls == []
    node._dock.phase = "going"
    assert node._dock_refuse(_response(), "홈 복귀") is True
    node._dock.phase, node._dock.canceling = "leaving", True
    assert node._dock_refuse(_response(), "홈 복귀") is True
    node._dock.canceling = False
    assert node._dock_refuse(_response(), "홈 복귀") is False


def test_stray_drive_while_entering_never_reaches_nav2():
    """관문을 지나친 주행(로직 타이머 등)도 도킹 중에는 Nav2 로 나가지 않는다."""
    node = _node()
    node._dock.phase, node._dock.mode = "entering", "docking"
    node._run_actions([Navigate(DEST_A), SpinInPlace(1.0, reason="시험")])
    assert node.navigator.goals == [] and node.navigator.spins == 0
    assert node._dock.phase == "entering" and node._dock.held is None


def test_spin_near_dock_is_dropped_not_undocked():
    """도크 바로 앞에서 제자리 회전 하나 때문에 스스로 도크에서 나오지 않는다."""
    node = _node()
    node._dock_apply(node._dock.on_node_state(
        {"seq": 1, "state": "idle", "near_dock": True, "last": {}}))
    assert node.logic.dock_near is True
    node._run_actions([SpinInPlace(1.0, reason="시험")])
    assert node.navigator.spins == 0 and node.dock_calls == []


def test_near_and_pending_marks_clear_when_robot_is_far(monkeypatch):
    """도킹 노드는 일이 끝날 때만 near_dock·undock_pending 을 고친다. 로봇이 도크에서 2 m 넘게 멀어지면 미션이 내린다."""
    node = _node()
    node._destinations_path = "/x/destinations.yaml"
    node._pose_cov_xy = 0.0
    monkeypatch.setattr(mm, "read_dock_spot", lambda path: DockSpot("m", 0.0, 0.0, 0.0))
    node._dock_apply(node._dock.on_node_state(
        {"seq": 1, "state": "idle", "near_dock": True, "undock_pending": True, "last": {}}))
    node._robot_pose = Pose2D(-1.0, 0.0, 0.0)
    node._dock_far_check()
    assert node._dock.near_dock and node._dock.undock_pending and node.dock_calls == []
    node._robot_pose = Pose2D(-3.0, 0.5, 0.0)
    node._dock_far_check()
    assert not node._dock.near_dock and not node._dock.undock_pending
    assert node.dock_calls == [CALL_CANCEL]
    assert node.logic.dock_near is False and node.logic.dock_pending is False


def test_held_drive_not_released_during_estop():
    """나오기가 끝난 순간 비상정지가 걸려 있으면 잡아 둔 주행을 보내지 않는다(비상정지 뒤 자동으로 가지 않는다)."""
    node = _node()
    _docked(node)
    node._handle_destination_request(_request("a"), _response(), False, "목적지")
    node._estop_active = True
    node._dock_apply(node._dock.on_node_state(
        {"seq": 2, "state": "idle", "near_dock": False,
         "last": {"action": "undock", "result": "success", "reason": ""}}))
    assert node.navigator.goals == []
    assert node.logic.state != State.NAVIGATING


def test_estop_callback_does_not_wait_for_dock_lock():
    """비상 콜백은 도킹 잠금을 기다리지 않는다 — 주 콜백이 Nav2 호출로 잠금을 쥔 동안 같이 멈추지 않게."""
    node = _node()
    node._dock.phase, node._dock.mode = "going", "docking"
    node._dock_far_check_at = 1e18
    held = threading.Event()
    release = threading.Event()

    def holder():
        with node._dock_lock:
            held.set()
            release.wait(5.0)

    th = threading.Thread(target=holder)
    th.start()
    held.wait(1.0)
    done = threading.Event()
    canceled = []
    node._cancel_nav = lambda *a, **k: canceled.append(1)
    worker = threading.Thread(target=lambda: (node._on_estop(SimpleNamespace(data=True)), done.set()))
    worker.start()
    assert done.wait(1.0), "비상 콜백이 도킹 잠금을 기다렸다"
    release.set()
    th.join()
    worker.join()
    assert node._dock.phase == "going" and canceled == []
    node._poll_nav_status = lambda: mm.NavStatus.NONE
    node._nav_distance_remaining = lambda: None
    with node._dock_lock:
        if node._estop_active and node._dock.phase == "going":
            node._dock_apply(node._dock.on_estop(True))
    assert node._dock.phase == "" and canceled == [1]
