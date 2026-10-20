"""주행 중 '끝났나' 확인의 기다림 — 라이브러리 기본 0.1 초에서 NAV_POLL_WAIT_SEC(0.03 초)로."""
import inspect
import threading
from types import SimpleNamespace

import pytest

mm = pytest.importorskip("vica_mission_manager.mission_manager_node")

from vica_mission_manager.mission_logic import NavStatus  # noqa: E402

needs_nav2 = pytest.mark.skipif(mm.BasicNavigator is None, reason="nav2_simple_commander 없음")


def test_poll_wait_is_30ms():
    """사용자 결정(2026-10-13): 0.03 초."""
    assert mm.NAV_POLL_WAIT_SEC == pytest.approx(0.03)


class _RecordingNavigator:
    """isTaskComplete 에 넘긴 기다림을 적는다."""

    def __init__(self, done):
        self.done = done
        self.timeouts = []

    def isTaskComplete(self, timeout_sec=0.10):  # noqa: N802 - nav2_simple_commander 이름
        self.timeouts.append(timeout_sec)
        return self.done

    def getResult(self):  # noqa: N802
        return mm.TaskResult.SUCCEEDED if mm.TaskResult is not None else None


def _polling_node(done):
    node = mm.MissionManagerNode.__new__(mm.MissionManagerNode)
    node._nav_lock = threading.Lock()
    node._nav_active = True
    node.navigator = _RecordingNavigator(done)
    node._nav_task_is_spin = False
    node._clear_before_next_nav = False
    node.logic = SimpleNamespace(state=None, active_destination=None)
    node._quiet_goal_events = lambda: False
    node._publish_goal_event = lambda *a, **k: None
    node._record_home_visit = lambda ok: None
    return node


def test_running_poll_waits_only_the_short_time():
    node = _polling_node(done=False)
    assert node._poll_nav_status() == NavStatus.RUNNING
    assert node.navigator.timeouts == [mm.NAV_POLL_WAIT_SEC]


def test_finished_poll_also_uses_the_short_time():
    node = _polling_node(done=True)
    assert node._poll_nav_status() == NavStatus.SUCCEEDED
    assert node.navigator.timeouts == [mm.NAV_POLL_WAIT_SEC]


def test_no_active_task_does_not_ask_nav2():
    node = _polling_node(done=False)
    node._nav_active = False
    assert node._poll_nav_status() == NavStatus.NONE
    assert node.navigator.timeouts == []


def test_bt_switch_wait_keeps_the_library_default():
    """트리 전환 기다림(10-02)은 바꾸지 않는다 — 인자 없이 부른다(라이브러리 기본 0.1 초)."""
    src = inspect.getsource(mm.MissionManagerNode._finish_task_for_bt_switch)
    assert "isTaskComplete()" in src
    assert "NAV_POLL_WAIT_SEC" not in src
    src = inspect.getsource(mm.MissionManagerNode._clear_costmaps_before_departure)
    assert "isTaskComplete()" in src


class _FakeFuture:
    def __init__(self, result=None):
        self._result = result

    def result(self):
        return self._result


def _fake_self(result_future):
    return SimpleNamespace(result_future=result_future, status=None,
                           debug=lambda *a, **k: None, info=lambda *a, **k: None)


@pytest.fixture
def spins(monkeypatch):
    """rclpy.spin_until_future_complete 대신 넘긴 기다림만 적는다."""
    calls = []
    monkeypatch.setattr(mm.rclpy, "spin_until_future_complete",
                        lambda node, future, timeout_sec=None: calls.append(timeout_sec))
    return calls


@needs_nav2
def test_default_wait_equals_the_library():
    """기본값은 라이브러리 원본과 같다 — 다른 곳은 지금과 똑같이 기다린다."""
    default = inspect.signature(mm.MissionNavigator.isTaskComplete).parameters["timeout_sec"].default
    assert default == pytest.approx(0.10)
    assert "timeout_sec=0.10" in inspect.getsource(mm.BasicNavigator.isTaskComplete)


@needs_nav2
@pytest.mark.parametrize("case", ["no_task", "running", "succeeded", "aborted"])
def test_same_answer_as_the_library(spins, case):
    """같은 상태에서 라이브러리 원본 isTaskComplete 와 같은 답·같은 status 를 낸다."""
    def make():
        if case == "no_task":
            return _fake_self(None)
        if case == "running":
            return _fake_self(_FakeFuture(None))
        status = 4 if case == "succeeded" else 6
        return _fake_self(_FakeFuture(SimpleNamespace(status=status)))

    lib_self, ours_self = make(), make()
    lib = mm.BasicNavigator.isTaskComplete(lib_self)
    ours = mm.MissionNavigator.isTaskComplete(ours_self, timeout_sec=mm.NAV_POLL_WAIT_SEC)
    assert ours == lib
    assert ours_self.status == lib_self.status
    expected = [] if case == "no_task" else [0.10, mm.NAV_POLL_WAIT_SEC]
    assert spins == pytest.approx(expected)
