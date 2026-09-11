"""주행 번호표(_nav_gen) — 늦은 취소가 새 goal 을 죽이지 않는지 본다."""
import threading

import pytest

mm = pytest.importorskip("vica_mission_manager.mission_manager_node")

from builtin_interfaces.msg import Time  # noqa: E402

from vica_mission_manager.mission_logic import (  # noqa: E402
    Destination,
    Navigate,
    Pose2D,
    SpinInPlace,
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
    def __init__(self, accept=True):
        self.accept = accept
        self.canceled = 0
        self.goals_sent = 0
        self.spins_sent = 0

    def goToPose(self, goal):  # noqa: N802 - nav2_simple_commander 이름
        self.goals_sent += 1
        return self.accept

    def spin(self, spin_dist):
        self.spins_sent += 1
        return self.accept

    def cancelTask(self):  # noqa: N802
        self.canceled += 1


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
