"""출발 전 local costmap 비우기 — 직전 안내가 성공으로 끝난 뒤의 새 출발만 비우는지 본다.

배경(2026-10-09 run81·82): 초음파 표시는 빔 밖으로 나가면 지워지지 않는다. 접근·도착
끝의 회전으로 빔 밖에 남은 표시가 다음 출발의 제자리 회전을 막아, 10초 포기(BT 의
ClearLocalCostmap)가 비울 때까지 섰다 — run81 15:48 홈, run82 18:14 사회 복지창구.
사용자 결정: 직전 안내가 다 끝나고 새로 출발할 때마다 비운다. 일시정지 뒤 재개·실패
뒤 재시도·주행 중 바꾸기는 비우지 않는다(지나온 낮은 물체 기억을 잃지 않게).

test_nav_cancel_race.py 처럼 노드를 rclpy 없이 __new__ 로 맨몸 생성한다.
"""
import threading
from types import SimpleNamespace

import pytest

mm = pytest.importorskip("vica_mission_manager.mission_manager_node")

from builtin_interfaces.msg import Time  # noqa: E402

from vica_mission_manager.mission_logic import (  # noqa: E402
    Destination,
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


class _FakeFuture:
    def __init__(self, done):
        self._done = done

    def done(self):
        return self._done


class _FakeClearClient:
    def __init__(self, calls, ready=True, responds=True):
        self.calls = calls
        self.ready = ready
        self.responds = responds
        self.removed = 0

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.calls.append("clear")
        return _FakeFuture(self.responds)

    def remove_pending_request(self, future):
        self.removed += 1


class _FakeNavigator:
    def __init__(self, accept=True, ready=True, responds=True):
        self.accept = accept
        self.calls = []           # 'clear' / 'goal' / 'spin' / 'cancel' 순서
        self.result = None
        self.task_done = True
        self.clear_costmap_local_srv = _FakeClearClient(self.calls, ready, responds)

    def goToPose(self, goal, behavior_tree=''):  # noqa: N802 - nav2_simple_commander 이름
        self.calls.append("goal")
        return self.accept

    def spin(self, spin_dist):
        self.calls.append("spin")
        return self.accept

    def cancelTask(self):  # noqa: N802
        self.calls.append("cancel")

    def isTaskComplete(self):  # noqa: N802
        return self.task_done

    def getResult(self):  # noqa: N802
        return self.result


class _FakeClock:
    class _Now:
        def to_msg(self):
            return Time()

    def now(self):
        return self._Now()


class _FakeLogic:
    state = State.NAVIGATING     # 대기 장소로 가는 중이 아니다, 복귀 중도 아니다
    active_destination = None

    def on_tick(self, now, status):
        return []


@pytest.fixture(autouse=True)
def _no_spin(monkeypatch):
    """rclpy.spin_until_future_complete 를 막는다 — 가짜 future 의 done() 이 답한다."""
    monkeypatch.setattr(mm.rclpy, "spin_until_future_complete",
                        lambda node, future, timeout_sec=None: None)


def _bare_node(**nav_kwargs):
    node = mm.MissionManagerNode.__new__(mm.MissionManagerNode)
    node._nav_lock = threading.Lock()
    node._nav_lock_timeout_sec = 2.0
    node._nav_active = False
    node._nav_gen = 0
    node.navigator = _FakeNavigator(**nav_kwargs)
    node._approach_bt = ""
    node._tree_files = {}
    node._guided_no_rail_bt = ""
    node._route_server_up = lambda: True
    node._nav2_ready = lambda: True
    node._task_bt = ""
    node._clear_before_next_nav = True     # __init__ 과 같은 시작값 — 켤 때의 첫 출발
    node._nav_task_is_spin = False
    node.logic = _FakeLogic()
    logger = _FakeLogger()
    node.get_logger = lambda: logger
    node.get_clock = lambda: _FakeClock()
    node._now = lambda: 0.0
    node._run_actions = lambda actions: None
    node._publish_goal_event = lambda event, dest, reason="": None
    return node


def _dest(dest_id="cafeteria"):
    return Destination(id=dest_id, name="탕비실", pose=Pose2D(1.0, 2.0, 90.0))


def _finish(node, result):
    """지금 task 가 result 로 끝났다고 보고 한 번 폴링한다."""
    node.navigator.result = result
    node._poll_nav_status()


def _warns(node):
    return [m for level, m in node.get_logger().lines if level == "warn"]


class TestClearsOnFreshDeparture:
    def test_first_departure_clears_before_goal(self):
        node = _bare_node()
        node._start_nav(Navigate(destination=_dest()))
        assert node.navigator.calls == ["clear", "goal"]
        assert node._clear_before_next_nav is False

    def test_departure_after_success_clears(self):
        """도착(성공) 뒤 다음 목적지·홈·대기 장소로 새로 출발 — 비운다."""
        node = _bare_node()
        node._start_nav(Navigate(destination=_dest()))
        _finish(node, mm.TaskResult.SUCCEEDED)
        node.navigator.calls.clear()
        node._start_nav(Navigate(destination=_dest(mm.HOME_DESTINATION_ID)))
        assert node.navigator.calls == ["clear", "goal"]

    def test_failed_handle_turn_after_approach_still_clears(self):
        """run82 18:14: 접근 성공 -> 손잡이 돌리기 Spin 이 사람에 막혀 실패 -> 목적지 출발.

        Spin 의 실패가 '직전 안내 성공'을 지우면 이 출발이 다시 10초 선다."""
        node = _bare_node()
        node._start_nav(Navigate(destination=_dest("approach:47")))
        _finish(node, mm.TaskResult.SUCCEEDED)
        node._start_spin(SpinInPlace(yaw_rad=3.14))
        _finish(node, mm.TaskResult.FAILED)
        node.navigator.calls.clear()
        node._start_nav(Navigate(destination=_dest()))
        assert node.navigator.calls == ["clear", "goal"]


class TestKeepsMemoryWhenTripNotFinished:
    def test_change_while_driving_does_not_clear(self):
        """주행 중 목적지 바꾸기 — 달리는 중 지나온 낮은 물체 기억을 지우면 안 된다."""
        node = _bare_node()
        node._start_nav(Navigate(destination=_dest()))
        node.navigator.calls.clear()
        node._start_nav(Navigate(destination=_dest("other")))
        assert node.navigator.calls == ["goal"]

    def test_resume_after_pause_does_not_clear(self, monkeypatch):
        """일시정지(취소) 뒤 다시 출발 — 앞 goal 이 성공으로 끝나지 않았다."""
        node = _bare_node()
        node._start_nav(Navigate(destination=_dest()))

        class _NoThread:
            def __init__(self, **kwargs):
                pass

            def start(self):
                pass

        monkeypatch.setattr(mm.threading, "Thread", _NoThread)
        node._cancel_nav(destination=None)
        node.navigator.calls.clear()
        node._start_nav(Navigate(destination=_dest()))
        assert node.navigator.calls == ["goal"]

    def test_retry_after_failure_does_not_clear(self):
        node = _bare_node()
        node._start_nav(Navigate(destination=_dest()))
        _finish(node, mm.TaskResult.FAILED)
        node.navigator.calls.clear()
        node._start_nav(Navigate(destination=_dest()))
        assert node.navigator.calls == ["goal"]

    def test_spin_still_running_does_not_clear(self):
        """도착 뒤 회전 중에 앱이 목적지를 보내 취소 스레드보다 먼저 왔다 — 아직 돈다.

        직전 안내는 성공이라 비울 차례지만, 로봇이 움직이는 중이면 비우지 않는다."""
        node = _bare_node()
        node._start_nav(Navigate(destination=_dest()))
        _finish(node, mm.TaskResult.SUCCEEDED)
        node._start_spin(SpinInPlace(yaw_rad=3.14))
        node.navigator.task_done = False          # 회전이 아직 끝나지 않았다
        node.navigator.calls.clear()
        node._start_nav(Navigate(destination=_dest("other")))
        assert node.navigator.calls == ["goal"]
        assert any("아직 돈다" in m for m in _warns(node))

    def test_succeeded_spin_does_not_arm_clear(self):
        """접근 실패 -> '못 찾아 원위치로' 회전 성공 -> 출발: 회전 성공은 안내 성공이 아니다."""
        node = _bare_node()
        node._start_nav(Navigate(destination=_dest("approach:7")))
        _finish(node, mm.TaskResult.FAILED)
        node._start_spin(SpinInPlace(yaw_rad=1.57))
        _finish(node, mm.TaskResult.SUCCEEDED)
        node.navigator.calls.clear()
        node._start_nav(Navigate(destination=_dest(mm.HOME_DESTINATION_ID)))
        assert node.navigator.calls == ["goal"]


class TestNeverBlocksDeparture:
    def test_missing_service_still_departs(self):
        node = _bare_node(ready=False)
        node._start_nav(Navigate(destination=_dest()))
        assert node.navigator.calls == ["goal"]
        assert any("서비스 없음" in m for m in _warns(node))

    def test_no_response_still_departs(self):
        node = _bare_node(responds=False)
        node._start_nav(Navigate(destination=_dest()))
        assert node.navigator.calls == ["clear", "goal"]
        assert node.navigator.clear_costmap_local_srv.removed == 1
        assert any("응답 없음" in m for m in _warns(node))

    def test_rejected_goal_clears_again_on_next_send(self):
        """goal 이 거부되면 주행이 시작되지 않았다 — 다시 보낼 때도 비운다."""
        node = _bare_node(accept=False)
        node._start_nav(Navigate(destination=_dest()))
        assert node._clear_before_next_nav is True
        node.navigator.accept = True
        node.navigator.calls.clear()
        node._start_nav(Navigate(destination=_dest()))
        assert node.navigator.calls == ["clear", "goal"]
