"""주행 중 '끝났나' 확인의 기다림 — 라이브러리 기본 0.1 초에서 NAV_POLL_WAIT_SEC(0.03 초)로.

배경(2026-10-12~13): nav2_simple_commander 의 isTaskComplete 는 Nav2 소식을 0.1 초 동안 기다리며
처리한다. 쉬는 시간 없이 도는 예제 반복문(example_nav_to_pose.py)의 쉼 겸 우편함 확인이다. 미션은
자기 틱(_tick 5 Hz)이 있어 그 위에 0.1 초가 겹치고, 그동안 오는 진행 소식(bt_loop_duration 10 ms →
최대 100 Hz)을 다 처리하느라 CPU 를 쓰고 같은 처리 묶음(_main_group)의 음성·앱 요청이 줄을 섰다.
기다리지 않아도 결과는 대기열에 남아 다음 틱에 읽힌다(0 이면 남은 거리가 최대 3 초 묵는다).

주행 중 확인만 짧게 한다. 트리 전환 기다림(BT_SWITCH_WAIT_SEC, 10-02)·출발 전 확인은
라이브러리 기본 그대로다. test_nav_cancel_race.py 처럼 노드를 rclpy 없이 __new__ 로 만든다.
"""
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


# ---------------------------------------------------------------- 미션이 어디서 얼마나 기다리나

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


# ---------------------------------------------------------------- MissionNavigator = 라이브러리와 같은 판정

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
        status = 4 if case == "succeeded" else 6  # GoalStatus SUCCEEDED / ABORTED
        return _fake_self(_FakeFuture(SimpleNamespace(status=status)))

    lib_self, ours_self = make(), make()
    lib = mm.BasicNavigator.isTaskComplete(lib_self)
    ours = mm.MissionNavigator.isTaskComplete(ours_self, timeout_sec=mm.NAV_POLL_WAIT_SEC)
    assert ours == lib
    assert ours_self.status == lib_self.status
    # 기다림만 다르다: 라이브러리 0.1 초, 미션 주행 중 0.03 초. 할 일이 없으면 둘 다 안 기다린다.
    expected = [] if case == "no_task" else [0.10, mm.NAV_POLL_WAIT_SEC]
    assert spins == pytest.approx(expected)
