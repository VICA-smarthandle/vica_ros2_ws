"""경비(순찰) 모드 — 노드 배선 (2026-11-14, ~/vica_data/patrol_plan/SPEC.md 3절).

앱 사건 JSON(/vica_goal_event), 기록 파일(events.jsonl), 서비스 응답, /vica/robot_state 칸,
속도 상한, 그리고 "순찰 구간 이동에는 goal_* 가 나가지 않는다"를 본다. 다른 노드 시험처럼
노드를 rclpy 없이 __new__ 로 맨몸 생성한다. 로직은 test_patrol_logic.py 가 본다.
"""
import json
import threading
import uuid
from collections import deque
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

import pytest

mm = pytest.importorskip("vica_mission_manager.mission_manager_node")

from builtin_interfaces.msg import Time  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from vica_mission_manager.ledger import Ledger  # noqa: E402
from vica_mission_manager.mission_logic import (  # noqa: E402
    Destination,
    MapBounds,
    MissionLogic,
    Navigate,
    NavStatus,
    Pose2D,
    patrol_leg_destination,
    SetNavSpeedCap,
    SetNavSpeedLimit,
    State,
)
from vica_mission_manager.patrol_storage import PatrolEventLog, PatrolRouteStorage  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
NODE_SRC = (PKG / "vica_mission_manager" / "mission_manager_node.py").read_text(encoding="utf-8")
LAUNCH_SRC = (PKG / "launch" / "mission_manager.launch.py").read_text(encoding="utf-8")

MAP = "map_1"
ID_A = "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa"
ID_B = "bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb"
ID_C = "cccccccc-cccc-4ccc-8ccc-cccccccccccc"
A = Destination(id=ID_A, name="로비", pose=Pose2D(1.0, 0.0, 10.0, "map"), calibrated=True)
B = Destination(id=ID_B, name="복도 끝", pose=Pose2D(1.0, 2.0, 20.0, "map"), calibrated=True)
C = Destination(id=ID_C, name="창고 앞", pose=Pose2D(-1.0, 2.0, 30.0, "map"), calibrated=True)
BOUNDS = MapBounds(min_x=-50, min_y=-50, max_x=50, max_y=50)
HOME = Destination(id="__home__", name="홈", pose=Pose2D(5.0, 5.0, 90.0, "map"))
DETECTOR_OK = {"src": "detector", "kind": "status", "ready": True, "camera_ok": True}
INTRUDER = {"src": "detector", "kind": "intruder", "conf": 0.8, "hits": 2,
            "clip_id": "20261114_203455_intruder"}
GOAL_EVENTS = {"goal_sent", "goal_accepted", "goal_succeeded", "goal_failed", "goal_canceled",
               "goal_rejected", "goal_paused"}


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(("info", msg))

    def warn(self, msg):
        self.lines.append(("warn", msg))

    error = warn


class _Clock:
    class _Now:
        def to_msg(self):
            return Time()

    def now(self):
        return self._Now()


class _Navigator:
    """goToPose·결과만 흉내 낸다."""

    def __init__(self):
        self.goals = []
        self.canceled = 0
        self.done = False
        self.result = None
        # 출발 전 costmap 비우기는 '서비스 없음'으로 건너뛴다(test_clear_before_departure 가 본다).
        self.clear_costmap_local_srv = SimpleNamespace(service_is_ready=lambda: False)
        self.clear_costmap_global_srv = SimpleNamespace(service_is_ready=lambda: False)

    def goToPose(self, goal, behavior_tree=""):  # noqa: N802 - nav2_simple_commander 이름
        self.goals.append((goal, behavior_tree))
        self.done = False
        return True

    def isTaskComplete(self, timeout_sec=0.10):  # noqa: N802
        return self.done

    def getResult(self):  # noqa: N802
        return self.result

    def getFeedback(self):  # noqa: N802
        return SimpleNamespace(distance_remaining=2.0)

    def cancelTask(self):  # noqa: N802
        self.canceled += 1
        self.done = True


@pytest.fixture
def node(tmp_path, monkeypatch):
    """경로 파일·기록 파일은 임시 폴더, Nav2 는 가짜. 시각은 node.t 로 민다."""
    monkeypatch.setattr(mm, "apply_goal_event", lambda *a, **k: False)

    late = {"queue": None}   # 목록을 넣으면 취소 스레드가 '늦게' 돈다(시험이 직접 돌린다)

    class _NoThread:
        def __init__(self, target=None, args=(), name="", daemon=False):
            self.target, self.args = target, args

        def start(self):
            if late["queue"] is not None:
                late["queue"].append((self.target, self.args))
            else:
                self.target(*self.args)   # 취소를 그 자리에서 끝낸다

    monkeypatch.setattr(mm.threading, "Thread", _NoThread)
    (tmp_path / "destinations" / MAP).mkdir(parents=True)

    n = mm.MissionManagerNode.__new__(mm.MissionManagerNode)
    n.late = late
    n.t = 0.0
    n._now = lambda: n.t
    n._map_id = MAP
    n._ledger = Ledger()
    n._map_meta = SimpleNamespace(building="로봇관", floor=4)
    n.get_parameter = lambda name: SimpleNamespace(value=-1 if name == "current_floor" else "")
    n.destinations = {d.id: d for d in (A, B, C)}
    n.map_bounds = BOUNDS
    n._robot_pose = Pose2D(1.1, 0.2, 0.0, "map")
    n._pose_cov_xy = 0.0
    n._delivery_dest_id = None
    n._wake_reply_seq = ""
    log = _Log()
    n.get_logger = lambda: log
    n.get_clock = lambda: _Clock()
    for name in ("pub_goal_event", "pub_tts", "pub_speed_limit", "pub_state",
                 "pub_listen_request", "pub_wake_reply", "pub_haptic", "_pub_patrol_watch"):
        setattr(n, name, _Pub())
    # Nav2 배선(진짜 _start_nav·_poll_nav_status·_cancel_nav 를 탄다).
    n.navigator = _Navigator()
    n._nav_lock = threading.Lock()
    n._nav_lock_timeout_sec = 2.0
    n._nav_active = False
    n._nav_gen = 0
    n._approach_bt = ""
    n._tree_files = {}
    n._guided_no_rail_bt = ""
    n._route_server_up = lambda: True
    n._nav2_ready = lambda: True
    n._task_bt = ""
    n._clear_before_next_nav = False
    n._nav_task_is_spin = False
    n._record_home_visit = lambda ok: None
    # 순찰 배선.
    n._patrol_routes = PatrolRouteStorage(tmp_path / "destinations")
    n._patrol_log = PatrolEventLog(tmp_path / "media")
    # 진짜 _tick 이 지나는 나머지(대장·장애물 안내)는 꺼 둔다.
    n._ledger_prev_confirming = None
    n._obstacle = None
    n._obstacle_guard = SimpleNamespace(enabled=False)
    n._obstacle_cues = deque()
    n.logic = MissionLogic()
    n.logic.patrol_wall_clock = lambda: datetime(2026, 11, 14, 20, 34, 55)
    return n


def goal_events(node) -> list:
    return [json.loads(m.data) for m in node.pub_goal_event.msgs]


def event_names(node) -> list:
    return [e["event"] for e in goal_events(node)]


def log_lines(node) -> list:
    path = node._patrol_log.path
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def call(handler, request_id=None):
    """순찰 서비스 하나를 부른다. request_id 를 안 주면 앱처럼 UUID 를 넣는다(""는 그대로 "")."""
    response = SimpleNamespace(accepted=None, message="")
    if request_id is None:
        request_id = str(uuid.uuid4())
    return handler(SimpleNamespace(request_id=request_id), response)


def command(node, name):
    """기존 pause·resume·cancel 서비스(_handle_mission_command). request_id 는 UUID 여야 한다."""
    response = SimpleNamespace(accepted=None, message="")
    return node._handle_mission_command(
        SimpleNamespace(request_id=str(uuid.uuid4())), response, name)


def send(node, event: dict) -> None:
    node._on_patrol_event(String(data=json.dumps(event, ensure_ascii=False)))


def tick(node, t, ok=True) -> None:
    """진짜 _tick 한 번. ok 면 그 직전에 감지 노드 정상 신호가 온 것으로 친다."""
    node.t = t
    if ok:
        send(node, DETECTOR_OK)
    node._tick()


def arrive(node, t) -> None:
    node.navigator.done, node.navigator.result = True, mm.TaskResult.SUCCEEDED
    tick(node, t)


def start_patrol(node, ids=(ID_A, ID_B, ID_C), watch_off=()):
    node._patrol_routes.write(MAP, list(ids), list(watch_off))
    response = call(node._on_patrol_start)
    assert response.accepted, response.message
    return response


def patrolling(node):
    start_patrol(node)
    tick(node, 1.0)
    assert node.logic.state == State.PATROLLING
    return node


# ---- 시작 서비스 ------------------------------------------------------------


def test_start_reads_the_route_file_and_announces_it(node):
    response = start_patrol(node)
    assert response.message == "경비를 시작합니다."
    assert node.logic.state == State.PATROL_DWELL
    started = goal_events(node)[-1]
    assert started["event"] == "patrol_started" and started["route"] == ["로비", "복도 끝", "창고 앞"]
    assert started["map_id"] == MAP and started["name"] == "" and "timestamp" in started
    # 감지·음성·손잡이가 바로 따라오게 상태를 곧바로 낸다.
    assert [m.dialog_state for m in node.pub_state.msgs] == ["patrol_dwell"]
    assert node.navigator.goals == []                      # 준비 전에는 출발하지 않는다


TOO_FEW_LINE = "순찰 지점이 2곳 이상이어야 경비를 시작할 수 있습니다."   # 목업 문구 그대로


def test_start_without_a_route_is_refused_with_the_mockup_line(node):
    response = call(node._on_patrol_start)
    assert (response.accepted, response.message) == (False, TOO_FEW_LINE)
    assert node.pub_goal_event.msgs == [] and node.pub_state.msgs == []


def test_start_with_a_one_place_route_uses_the_same_mockup_line(node):
    node._patrol_routes.write(MAP, [ID_A])                 # 손으로 고친 파일 등 — 저장 서비스는 거절한다
    response = call(node._on_patrol_start)
    assert (response.accepted, response.message) == (False, TOO_FEW_LINE)


def test_estop_line_is_the_mockup_line_only_for_patrol_start(node):
    """시작 거절만 목업 문구다 — 순찰 중 재개가 비상정지로 거절될 때는 기존 문구 그대로."""
    patrolling(node)
    send(node, INTRUDER)
    node.logic.estop_active = True                         # 래치는 걸렸고 상태 전이는 아직(다른 스레드)
    response = command(node, "resume")
    assert (response.accepted, response.message) == (
        False, "비상정지 중입니다. 해제한 뒤 다시 시도하세요.")


def test_start_with_a_deleted_place_says_why(node):
    node._patrol_routes.write(MAP, [ID_A, "dddddddd-dddd-4ddd-8ddd-dddddddddddd"])
    response = call(node._on_patrol_start)
    assert (response.accepted, response.message) == (False, "저장되지 않은 장소가 경로에 있습니다.")


@pytest.mark.parametrize("break_it,message", [
    (lambda n: setattr(n, "_robot_pose", None), "로봇 위치가 아직 정해지지 않았습니다."),
    (lambda n: setattr(n, "_pose_cov_xy", 5.0), "로봇 위치가 아직 정해지지 않았습니다."),
    (lambda n: setattr(n, "_nav2_ready", lambda: False), "주행 준비(Nav2)가 아직 안 됐습니다."),
    (lambda n: n.logic.on_estop(True, 0.0), "비상정지 중에는 경비를 시작할 수 없습니다."),
])
def test_start_gate_messages(node, break_it, message):
    node._patrol_routes.write(MAP, [ID_A, ID_B])
    break_it(node)
    response = call(node._on_patrol_start)
    assert (response.accepted, response.message) == (False, message)


def test_start_twice_is_refused(node):
    start_patrol(node)
    response = call(node._on_patrol_start)
    assert (response.accepted, response.message) == (False, "이미 순찰 중입니다.")


def test_request_id_format_is_not_checked_for_patrol_services(node):
    node._patrol_routes.write(MAP, [ID_A, ID_B])
    assert call(node._on_patrol_start, request_id="app-1").accepted
    assert call(node._on_patrol_stop, request_id="").accepted


# ---- 구간 이동: goal_* 없음 -------------------------------------------------


def test_a_whole_leg_publishes_only_patrol_events(node):
    patrolling(node)
    goal, tree = node.navigator.goals[-1]
    assert (goal.pose.position.x, goal.pose.position.y) == (1.0, 0.0) and tree == ""
    arrive(node, 5.0)                                      # 도착 → 3초 정지
    assert node.logic.state == State.PATROL_DWELL
    tick(node, 8.0)                                        # 다음 지점으로
    node.navigator.done, node.navigator.result = True, mm.TaskResult.FAILED
    tick(node, 9.0)                                        # 그 구간 실패 → 건너뛴다
    send(node, INTRUDER)                                   # 이동 중 경보 → 취소
    names = event_names(node)
    assert not GOAL_EVENTS & set(names)
    assert names == ["patrol_started", "patrol_waypoint", "patrol_waypoint", "patrol_waypoint",
                     "patrol_intruder"]
    assert node.navigator.canceled == 1 and len(node.navigator.goals) == 3


def test_waypoint_event_names_the_place_and_counts(node):
    patrolling(node)
    first = goal_events(node)[-1]
    assert first["event"] == "patrol_waypoint"
    assert (first["name"], first["index"], first["lap"], first["count"]) == ("로비", 0, 1, 3)
    assert (first["x"], first["y"], first["destination_id"]) == (1.0, 0.0, ID_A)


def test_state_changes_inside_a_tick_are_broadcast_at_once(node):
    """출발·도착·지점 정지 끝은 틱 안에서 일어난다 — 1 Hz 를 기다리지 않고 바로 낸다."""
    patrolling(node)
    arrive(node, 5.0)
    tick(node, 6.0)                                        # 바뀐 것이 없으면 내지 않는다
    tick(node, 8.0)
    assert [m.dialog_state for m in node.pub_state.msgs] == [
        "patrol_dwell", "patrolling", "patrol_dwell", "patrolling"]
    assert [m.is_moving for m in node.pub_state.msgs] == [False, True, False, True]


def test_a_leg_decided_before_an_estop_is_not_sent(node):
    """로직이 구간을 정한 뒤 실행 전에 비상정지(다른 스레드)가 순찰을 끝냈다 — 보내지 않는다."""
    patrolling(node)
    leg = Navigate(patrol_leg_destination(B, C))
    node._run_actions(node.logic.on_estop(True, 2.0))
    node._start_nav(leg)
    assert len(node.navigator.goals) == 1 and node._nav_active is False
    assert not GOAL_EVENTS & set(event_names(node))


def test_estop_while_a_leg_is_being_sent_cancels_that_goal(node):
    """goal 수락을 기다리는 사이 비상정지가 오면 그때의 취소는 빈손이다(아직 goal 이 없다).
    수락 뒤 다시 보고 거둔다 — 안 거두면 래치가 풀린 뒤 Nav2 가 그 지점으로 혼자 간다."""
    start_patrol(node)
    send_goal = node.navigator.goToPose

    def go_to_pose(goal, behavior_tree=""):
        accepted = send_goal(goal, behavior_tree)
        node._run_actions(node.logic.on_estop(True, node.t))   # 비상정지 스레드가 끼어든다
        assert node.navigator.canceled == 0                    # 이때는 거둘 goal 이 없다
        return accepted

    node.navigator.goToPose = go_to_pose
    tick(node, 1.0)
    assert node.logic.state == State.ESTOPPED
    assert node.navigator.canceled == 1 and node._nav_active is False
    names = event_names(node)
    assert not GOAL_EVENTS & set(names) and names[-1] == "patrol_stopped"


def test_patrol_ending_while_a_leg_is_being_sent_also_cancels_that_goal(node):
    """같은 창에서 상태가 순찰 밖으로 나간 다른 경우(문맥이 사라져 끝난 뒤 등)도 거둔다."""
    patrolling(node)
    arrive(node, 5.0)
    send_goal = node.navigator.goToPose

    def go_to_pose(goal, behavior_tree=""):
        accepted = send_goal(goal, behavior_tree)
        node.logic._patrol = None
        node.logic._to_idle()
        return accepted

    node.navigator.goToPose = go_to_pose
    tick(node, 8.0)
    assert node.navigator.canceled == 1 and node._nav_active is False


def test_quiet_goal_events_covers_patrol_legs(node):
    for state, quiet in ((State.PATROLLING, True), (State.NAVIGATING, False),
                         (State.RETURNING, False), (State.MOVING_TO_WAIT_SPOT, True)):
        node.logic.state = state
        assert node._quiet_goal_events() is quiet


def test_leg_that_cannot_be_sent_ends_the_patrol_without_goal_events(node):
    """Nav2 가 내려가 구간을 못 보내면 실패로 세어 3번 만에 끝낸다(goal_rejected 도 없다)."""
    start_patrol(node)
    node._nav2_ready = lambda: False
    tick(node, 1.0)
    assert node.logic.state == State.IDLE
    names = event_names(node)
    assert not GOAL_EVENTS & set(names)
    assert names.count("patrol_waypoint") == 3 and names[-1] == "patrol_stopped"
    assert goal_events(node)[-1]["reason"] == "nav_failed"


# ---- 속도 상한 --------------------------------------------------------------


def test_patrol_speed_cap_is_an_absolute_speed_limit(node):
    node._run_actions([SetNavSpeedCap(0.6), SetNavSpeedLimit(0.0)])
    cap, release = node.pub_speed_limit.msgs
    assert (cap.percentage, cap.speed_limit) == (False, pytest.approx(0.6))
    assert (release.percentage, release.speed_limit) == (True, 0.0)


def test_each_leg_sets_the_cap_and_arrival_releases_it(node):
    patrolling(node)
    assert [(m.percentage, round(m.speed_limit, 2)) for m in node.pub_speed_limit.msgs] == [(False, 0.6)]
    arrive(node, 5.0)
    assert (node.pub_speed_limit.msgs[-1].percentage, node.pub_speed_limit.msgs[-1].speed_limit) == (True, 0.0)


# ---- 경보: 사건 JSON·음성·기록 파일 ------------------------------------------


def test_alarm_event_carries_place_position_and_clip(node):
    patrolling(node)
    node.t = 3.0
    send(node, INTRUDER)
    alarm = goal_events(node)[-1]
    assert alarm["event"] == "patrol_intruder"
    assert alarm["alarm_id"] == "20261114_203455_intruder"
    assert (alarm["place"], alarm["robot_x"], alarm["robot_y"]) == ("로비", 1.1, 0.2)
    assert alarm["clip_id"] == "20261114_203455_intruder" and alarm["resend"] is False
    assert "photo" not in alarm and "level_db" not in alarm
    assert node.pub_tts.msgs[-1].data == "emergency:침입자 발생"
    assert node.pub_state.msgs[-1].dialog_state == "patrol_alarm"
    assert node.pub_listen_request.msgs == []              # 질문이 아니다 — 듣기 창을 열지 않는다


def test_sound_and_light_events_carry_their_own_keys(node):
    patrolling(node)
    send(node, {"src": "audio", "kind": "sound", "db": 71.5, "threshold": 67.0})
    sound = goal_events(node)[-1]
    assert (sound["event"], sound["level_db"]) == ("patrol_sound", 71.5) and "clip_id" not in sound

    send(node, DETECTOR_OK)
    assert command(node, "resume").accepted
    arrive(node, 5.0)
    send(node, {"src": "detector", "kind": "light", "brightness": 35.0, "photo": "light_1.jpg"})
    light = goal_events(node)[-1]
    assert (light["event"], light["photo"]) == ("patrol_light", "light_1.jpg")
    assert node.pub_tts.msgs[-1].data == "emergency:불 켜짐 감지"
    assert node.pub_state.msgs[-1].dialog_state == "patrol_light"


def test_alarm_without_a_pose_sends_nulls(node):
    patrolling(node)
    node._robot_pose = None
    send(node, INTRUDER)
    alarm = goal_events(node)[-1]
    assert alarm["place"] == "" and alarm["robot_x"] is None and alarm["robot_y"] is None
    assert log_lines(node)[0]["x"] is None


def test_log_file_is_written_only_at_the_alarm_not_on_resends(node):
    patrolling(node)
    assert log_lines(node) == []                           # 순찰만으로는 쓰지 않는다
    arrive(node, 5.0)
    tick(node, 8.0)
    assert not node._patrol_log.path.exists()
    node.t = 9.0
    send(node, INTRUDER)
    assert log_lines(node) == [{
        "type": "alarm", "alarm_id": "20261114_203455_intruder", "kind": "intruder",
        "time": "2026-11-14T20:34:55", "place": "로비", "x": 1.1, "y": 0.2, "map_id": MAP,
        "clip_id": "20261114_203455_intruder", "photo": None, "level_db": None,
    }]
    for t in (19.0, 29.0, 39.0):                           # 10초마다 재전송(확인 전)
        tick(node, t)
    resends = [e for e in goal_events(node) if e.get("resend")]
    assert len(resends) == 3 and {e["event"] for e in resends} == {"patrol_intruder"}
    assert len(log_lines(node)) == 1


def test_clip_learned_later_adds_one_line_and_no_app_event(node):
    patrolling(node)
    send(node, {"src": "audio", "kind": "sound", "db": 70.0, "threshold": 67.0})
    before = len(node.pub_goal_event.msgs)
    clip = {"src": "detector", "kind": "clip", "clip_id": "20261114_203456_sound",
            "state": "started", "alarm": "sound"}
    send(node, clip)
    send(node, dict(clip, state="done", frames=75))
    assert len(node.pub_goal_event.msgs) == before
    assert log_lines(node)[1:] == [{"type": "clip", "alarm_id": "20261114_203455_sound",
                                    "clip_id": "20261114_203456_sound"}]


def test_log_failure_only_warns(node, tmp_path):
    blocker = tmp_path / "blocker"
    blocker.write_text("x", encoding="utf-8")
    node._patrol_log = PatrolEventLog(blocker / "sub")
    patrolling(node)
    send(node, INTRUDER)
    assert node.logic.state == State.PATROL_ALARM
    assert goal_events(node)[-1]["event"] == "patrol_intruder"
    assert any(level == "warn" and "경보 기록" in line for level, line in node.get_logger().lines)


# ---- 확인·재개·일시정지·종료 서비스 -----------------------------------------


def test_ack_stops_the_voice_but_keeps_the_stop(node):
    patrolling(node)
    send(node, INTRUDER)
    response = call(node._on_patrol_ack, request_id="20261114_203455_intruder")
    assert (response.accepted, response.message) == (True, "경보를 확인했습니다.")
    acked = goal_events(node)[-1]
    assert (acked["event"], acked["alarm_id"]) == ("patrol_acked", "20261114_203455_intruder")
    said = len(node.pub_tts.msgs)
    for t in (10.0, 20.0, 30.0):
        tick(node, t)
    assert len(node.pub_tts.msgs) == said and node.logic.state == State.PATROL_ALARM
    assert not [e for e in goal_events(node) if e.get("resend")]


def test_ack_accepts_a_plain_request_id_and_rejects_another_alarm(node):
    patrolling(node)
    assert call(node._on_patrol_ack).message == "확인할 경보가 없습니다."
    send(node, INTRUDER)
    stale = call(node._on_patrol_ack, request_id="20261114_190000_sound")
    assert (stale.accepted, stale.message) == (False, "지금 경보와 다른 경보입니다.")
    assert call(node._on_patrol_ack).accepted              # uuid — 지금 경보를 확인한다


def test_resume_service_acks_and_departs_again(node):
    patrolling(node)
    send(node, INTRUDER)
    node.t = 9.0
    send(node, DETECTOR_OK)
    response = command(node, "resume")
    assert response.accepted and node.logic.state == State.PATROLLING
    assert event_names(node)[-3:] == ["patrol_acked", "patrol_resumed", "patrol_waypoint"]
    assert node.pub_state.msgs[-1].dialog_state == "patrolling"
    assert len(node.navigator.goals) == 2


def test_resume_is_refused_in_korean_while_the_camera_is_down(node):
    patrolling(node)
    send(node, INTRUDER)
    node.t = 9.0                                           # 마지막 정상 신호(t=1)에서 8초
    response = command(node, "resume")
    assert (response.accepted, response.message) == (False, "카메라가 준비되지 않아 재개할 수 없습니다.")
    assert node.logic.state == State.PATROL_ALARM


def test_pause_service_becomes_patrol_paused(node):
    patrolling(node)
    response = command(node, "pause")
    assert response.accepted and node.logic.state == State.PATROL_PAUSED
    assert event_names(node)[-1] == "patrol_paused"
    assert node.navigator.canceled == 1
    state = node.pub_state.msgs[-1]
    assert (state.dialog_state, state.is_moving, state.is_paused) == ("patrol_paused", False, True)


def _without_time(event: dict) -> dict:
    return {k: v for k, v in event.items() if k != "timestamp"}


def test_stop_service_ends_the_patrol_then_returns_home_like_the_app_button(node):
    """경비 종료 → 홈 복귀(2026-11-14 사용자 결정). 앱의 /vica/mission/return_home 과 같은
    상태(returning)·사건(goal_sent·goal_accepted·return_home_sent)·트리(기본)를 탄다."""
    node.logic.return_destination = HOME
    # 먼저 앱 홈 복귀를 그대로 한 번 — 견줄 기준이다.
    assert call(node._on_return_home).accepted
    direct = [_without_time(e) for e in goal_events(node)]
    direct_goal, direct_tree = node.navigator.goals[-1]
    arrive(node, 0.5)
    assert node.logic.state == State.IDLE
    node.pub_goal_event.msgs.clear()

    patrolling(node)
    response = call(node._on_patrol_stop)
    assert (response.accepted, response.message) == (True, "경비를 종료했습니다.")
    assert node.logic.state == State.RETURNING and node.navigator.canceled == 1
    names = event_names(node)
    assert names[-4:] == ["patrol_stopped", "goal_sent", "goal_accepted", "return_home_sent"]
    stopped = goal_events(node)[-4]
    assert (stopped["reason"], stopped["return_home"]) == ("admin", True)
    assert [_without_time(e) for e in goal_events(node)[-3:]] == direct
    goal, tree = node.navigator.goals[-1]
    assert (goal.pose.position.x, goal.pose.position.y, tree) == (5.0, 5.0, direct_tree)
    assert goal.pose.orientation == direct_goal.pose.orientation
    state = node.pub_state.msgs[-1]
    assert (state.dialog_state, state.is_moving) == ("returning", False)   # 기존 복귀와 같은 값
    # 순찰 속도 상한은 풀려 있다.
    assert (node.pub_speed_limit.msgs[-1].percentage, node.pub_speed_limit.msgs[-1].speed_limit) == (True, 0.0)
    # 홈에 도착하면 기존 복귀처럼 끝난다 — 순찰은 다시 돌지 않는다.
    arrive(node, 30.0)
    assert node.logic.state == State.IDLE and event_names(node)[-1] == "return_home_succeeded"
    assert call(node._on_patrol_stop).message == "순찰 중이 아닙니다."


def test_stop_without_a_home_ends_in_place_with_the_existing_refusal_line(node):
    # 기존 홈 복귀 서비스가 홈이 없을 때 돌려주는 문구 — 새로 만들지 않고 이 글자를 그대로 싣는다.
    refused = call(node._on_return_home)
    assert (refused.accepted, refused.message) == (False, "홈 복귀 거부: no_home")
    patrolling(node)
    response = call(node._on_patrol_stop)
    assert (response.accepted, response.message) == (True, refused.message)
    assert node.logic.state == State.IDLE and node.navigator.canceled == 1
    stopped = goal_events(node)[-1]
    assert (stopped["event"], stopped["reason"], stopped["return_home"]) == (
        "patrol_stopped", "admin", False)
    assert "return_home_sent" not in event_names(node)
    assert node.pub_state.msgs[-1].dialog_state == "idle"
    assert call(node._on_patrol_stop).message == "순찰 중이 아닙니다."


def test_stop_when_nav2_is_down_ends_in_place_with_the_existing_refusal_line(node):
    node.logic.return_destination = HOME
    patrolling(node)
    node._nav2_ready = lambda: False
    response = call(node._on_patrol_stop)
    assert (response.accepted, response.message) == (True, "홈 복귀 거부: nav_not_ready")
    assert node.logic.state == State.IDLE and goal_events(node)[-1]["return_home"] is False
    assert len(node.navigator.goals) == 1 and not GOAL_EVENTS & set(event_names(node))


def test_stop_during_an_alarm_silences_it_and_returns_home(node):
    node.logic.return_destination = HOME
    patrolling(node)
    send(node, INTRUDER)
    said = len(node.pub_tts.msgs)
    assert call(node._on_patrol_stop).message == "경비를 종료했습니다."
    assert node.logic.state == State.RETURNING
    for t in (5.0, 12.0, 22.0):
        tick(node, t)
    assert len(node.pub_tts.msgs) == said                  # "침입자 발생" 반복이 멈췄다
    assert not [e for e in goal_events(node) if e.get("resend")]
    assert node.logic.state == State.RETURNING


def test_late_cancel_of_the_leg_does_not_break_the_return_home(node):
    """구간 취소 직후 복귀 goal 이 나간다. 취소 스레드가 늦게 돌아도(일시정지 뒤 재개·앱 선점과
    같은 순서) 주행 번호표가 달라 복귀 goal 을 건드리지 않는다 — 복귀가 실패로 끝나지 않는다."""
    node.logic.return_destination = HOME
    patrolling(node)
    node.late["queue"] = []
    assert call(node._on_patrol_stop).accepted
    assert node.logic.state == State.RETURNING and node._nav_active is True
    assert len(node.late["queue"]) == 1                    # 구간 취소가 아직 안 돌았다
    for target, args in node.late["queue"]:
        target(*args)                                      # 늦은 취소가 이제 돈다
    node.late["queue"] = None
    assert node.navigator.canceled == 0 and node._nav_active is True
    tick(node, 4.0)
    tick(node, 5.0)
    assert node.logic.state == State.RETURNING
    assert not {"return_home_canceled", "return_home_failed"} & set(event_names(node))
    arrive(node, 30.0)
    assert event_names(node)[-1] == "return_home_succeeded" and node.logic.state == State.IDLE


def test_app_cancel_during_patrol_ends_it(node):
    patrolling(node)
    response = command(node, "cancel")
    assert (response.accepted, response.message) == (True, "홈 복귀 거부: no_home")
    assert node.logic.state == State.IDLE
    assert event_names(node)[-1] == "patrol_stopped"
    assert not GOAL_EVENTS & set(event_names(node))
    assert not [m for m in node.pub_tts.msgs if "취소" in m.data]


def test_app_cancel_during_patrol_also_returns_home(node):
    node.logic.return_destination = HOME
    patrolling(node)
    response = command(node, "cancel")
    assert (response.accepted, response.message) == (True, "cancel 요청을 처리했습니다.")
    assert node.logic.state == State.RETURNING
    assert event_names(node)[-4:] == ["patrol_stopped", "goal_sent", "goal_accepted", "return_home_sent"]
    assert goal_events(node)[-4]["return_home"] is True
    assert node.pub_state.msgs[-1].dialog_state == "returning"
    assert not [m for m in node.pub_tts.msgs if "취소" in m.data]


def test_ordinary_cancel_never_reuses_an_old_patrol_result(node):
    """순찰이 아닌 취소는 홈 복귀 마무리를 타지 않는다(지난 경비 종료의 결과가 남아 있어도)."""
    node.logic.return_destination = HOME
    patrolling(node)
    call(node._on_patrol_stop)
    arrive(node, 30.0)
    node.pub_goal_event.msgs.clear()
    actions, _ = node.logic.on_app_destination(A, BOUNDS, True, 31.0)
    node._run_actions(actions)
    response = command(node, "cancel")
    assert (response.accepted, response.message) == (True, "cancel 요청을 처리했습니다.")
    assert "return_home_sent" not in event_names(node) and node.logic.state == State.IDLE


def test_automatic_endings_do_not_return_home(node):
    """구간 실패로 끝난 순찰은 홈으로 가지 않는다 — patrol_stopped 의 return_home 은 false."""
    node.logic.return_destination = HOME
    start_patrol(node)
    node._nav2_ready = lambda: False
    tick(node, 1.0)
    stopped = goal_events(node)[-1]
    assert (stopped["event"], stopped["reason"], stopped["return_home"]) == (
        "patrol_stopped", "nav_failed", False)
    assert node.logic.state == State.IDLE and "return_home_sent" not in event_names(node)


def test_estop_reports_patrol_stopped_with_reason_estop(node):
    patrolling(node)
    node._run_actions(node.logic.on_estop(True, 3.0))
    stopped = goal_events(node)[-1]
    assert (stopped["event"], stopped["reason"], stopped["return_home"]) == (
        "patrol_stopped", "estop", False)
    assert not GOAL_EVENTS & set(event_names(node))
    assert node.navigator.canceled == 1


# ---- /vica/robot_state ------------------------------------------------------


@pytest.mark.parametrize("state,moving,paused", [
    (State.PATROLLING, True, False),
    (State.PATROL_DWELL, False, False),
    (State.PATROL_ALARM, False, False),
    (State.PATROL_LIGHT, False, False),
    (State.PATROL_PAUSED, False, True),
    (State.NAVIGATING, True, False),       # 기존 값은 그대로
    (State.PAUSED, False, True),
    (State.IDLE, False, False),
])
def test_robot_state_fields(node, state, moving, paused):
    node.logic.state = state
    node._publish_robot_state()
    msg = node.pub_state.msgs[-1]
    assert (msg.dialog_state, msg.is_moving, msg.is_paused) == (state.value, moving, paused)


# ---- 순찰이 아닐 때 ---------------------------------------------------------


def test_patrol_events_are_not_even_parsed_outside_a_patrol(node, monkeypatch):
    def _boom(_text):
        raise AssertionError("순찰이 아닌데 JSON 을 풀었다")

    monkeypatch.setattr(mm, "json", SimpleNamespace(loads=_boom, dumps=json.dumps))
    node._on_patrol_event(String(data=json.dumps(INTRUDER)))
    node._on_patrol_event(String(data="{깨진 json"))
    assert node.logic.state == State.IDLE
    assert node.pub_goal_event.msgs == [] and node.pub_tts.msgs == [] and node.pub_state.msgs == []


def test_broken_patrol_event_is_dropped_during_a_patrol(node):
    patrolling(node)
    for text in ("{깨진 json", "[1, 2]", '"intruder"', ""):
        node._on_patrol_event(String(data=text))
    assert node.logic.state == State.PATROLLING


def test_ordinary_guidance_still_publishes_goal_events(node):
    """순찰이 아닌 주행은 그대로 goal_sent·goal_accepted·goal_succeeded 를 낸다."""
    actions, _ = node.logic.on_app_destination(A, BOUNDS, True, 0.0)
    node._run_actions(actions)
    node.navigator.done, node.navigator.result = True, mm.TaskResult.SUCCEEDED
    assert node._poll_nav_status() == NavStatus.SUCCEEDED
    assert event_names(node) == ["goal_sent", "goal_accepted", "goal_succeeded"]
    assert [(m.percentage, m.speed_limit) for m in node.pub_speed_limit.msgs] == [(True, 0.0)]


# ---- 경로 서비스 ------------------------------------------------------------


def save(node, ids, map_id=MAP, watch_off=()):
    response = SimpleNamespace(accepted=None, reason="", message="")
    return node._on_save_patrol_route(
        SimpleNamespace(map_id=map_id, destination_ids=list(ids), watch_off=list(watch_off)),
        response)


def get(node, map_id=""):
    response = SimpleNamespace(found=None, map_id="", destination_ids=None, names=None,
                               watch_off=None, message="")
    return node._on_get_patrol_route(SimpleNamespace(map_id=map_id), response)


def test_save_then_get_returns_ids_and_names_in_tap_order(node):
    saved = save(node, [ID_C, ID_A.upper(), ID_B])
    assert (saved.accepted, saved.reason) == (True, "ok") and "3곳" in saved.message
    got = get(node)
    assert (got.found, got.map_id) == (True, MAP)
    assert got.destination_ids == [ID_C, ID_A, ID_B]
    assert got.names == ["창고 앞", "로비", "복도 끝"] and got.message == ""


def test_save_with_empty_map_id_means_the_current_map(node):
    assert save(node, [ID_A, ID_B], map_id="").accepted
    assert node._patrol_routes.read(MAP) == ([ID_A, ID_B], [])


@pytest.mark.parametrize("ids,reason", [
    ([ID_A], "too_few"),
    ([ID_A, "dddddddd-dddd-4ddd-8ddd-dddddddddddd"], "unknown_destination"),
    ([ID_A, ID_B, ID_B], "duplicate_adjacent"),
    ([ID_A, ID_B, ID_A], "duplicate_adjacent"),
])
def test_save_rejections_use_the_spec_codes_and_keep_the_old_route(node, ids, reason):
    save(node, [ID_A, ID_B])
    refused = save(node, ids)
    assert (refused.accepted, refused.reason) == (False, reason) and refused.message
    assert node._patrol_routes.read(MAP) == ([ID_A, ID_B], [])


def test_save_empty_list_deletes_the_route(node):
    save(node, [ID_A, ID_B])
    deleted = save(node, [])
    assert (deleted.accepted, deleted.reason) == (True, "ok")
    got = get(node)
    assert (got.found, got.destination_ids, got.names) == (False, [], [])
    assert got.message == "저장된 순찰 경로가 없습니다."
    assert save(node, []).accepted                          # 지울 것이 없어도 받는다


def test_save_for_an_unknown_map_is_refused(node):
    for map_id in ("map_other", "../etc"):
        refused = save(node, [ID_A, ID_B], map_id=map_id)
        assert (refused.accepted, refused.reason) == (False, "unknown_map")


def test_save_for_another_map_checks_that_maps_places(node, tmp_path):
    folder = tmp_path / "destinations" / "map_2"
    folder.mkdir()
    (folder / "destinations.yaml").write_text(
        "destinations:\n"
        f"  - {{id: {ID_A}, name: 다른 층 로비, pose: {{x: 1.0, y: 1.0, yaw: 0.0}}}}\n"
        f"  - {{id: {ID_C}, name: 다른 층 창고, pose: {{x: 2.0, y: 1.0, yaw: 0.0}}}}\n",
        encoding="utf-8")
    assert save(node, [ID_A, ID_C], map_id="map_2").accepted
    assert save(node, [ID_A, ID_B], map_id="map_2").reason == "unknown_destination"
    assert get(node, map_id="map_2").names == ["다른 층 로비", "다른 층 창고"]
    assert get(node).found is False                         # 지금 지도의 경로는 따로다


def test_save_io_error_is_reported(node, monkeypatch):
    def _fail(map_id, ids, watch_off=(), saved_at=None):
        raise OSError("디스크가 가득 찼습니다")

    monkeypatch.setattr(node._patrol_routes, "write", _fail)
    refused = save(node, [ID_A, ID_B])
    assert (refused.accepted, refused.reason) == (False, "io_error")


def test_get_marks_deleted_places_with_an_empty_name(node):
    save(node, [ID_A, ID_B, ID_C])
    del node.destinations[ID_B]                             # 저장 뒤 장소가 지워졌다
    got = get(node)
    assert got.found and got.names == ["로비", "", "창고 앞"]
    assert "지워진 장소" in got.message


# ---- 감시 항목 켜고 끄기(2026-11-14 사용자 요청) -----------------------------


def watch_msgs(node) -> list:
    return [json.loads(m.data) for m in node._pub_patrol_watch.msgs]


def test_start_tells_the_watch_setting_before_the_patrol_state(node):
    order = []
    node._pub_patrol_watch.publish = lambda msg: order.append(("watch", json.loads(msg.data)))
    node.pub_state.publish = lambda msg: order.append(("state", msg.dialog_state))
    start_patrol(node, watch_off=["light"])
    assert order == [("watch", {"off": ["light"]}), ("state", "patrol_dwell")]


def test_start_with_everything_watched_says_so(node):
    start_patrol(node)
    assert watch_msgs(node) == [{"off": []}]               # 지난 순찰의 값이 남지 않게 매번 낸다


def test_a_refused_start_does_not_touch_the_watch_setting(node):
    call(node._on_patrol_start)                            # 경로 없음
    assert node._pub_patrol_watch.msgs == []


def test_an_item_switched_off_is_ignored_for_the_whole_patrol(node):
    start_patrol(node, watch_off=["intruder"])
    tick(node, 1.0)
    send(node, INTRUDER)
    assert node.logic.state == State.PATROLLING            # 꺼 둔 종류는 버린다
    told = watch_msgs(node)
    save(node, [ID_A, ID_B, ID_C])                         # 순찰 중 다시 켜 저장해도 이번 순찰은 그대로
    assert watch_msgs(node) == told                        # 감지·호출어 노드에도 순찰 중에는 알리지 않는다
    assert node._patrol_routes.read(MAP) == ([ID_A, ID_B, ID_C], [])   # 파일에는 적혔다 — 다음 경비부터
    send(node, INTRUDER)
    assert node.logic.state == State.PATROLLING
    send(node, {"src": "audio", "kind": "sound", "db": 70.0, "threshold": 67.0})
    assert node.logic.state == State.PATROL_ALARM          # 켜 둔 종류는 그대로 받는다


def test_save_keeps_the_watch_setting_and_get_returns_it(node):
    saved = save(node, [ID_A, ID_B], watch_off=["sound", "LIGHT", "nonsense"])
    assert (saved.accepted, saved.reason) == (True, "ok")
    assert node._patrol_routes.read(MAP) == ([ID_A, ID_B], ["light", "sound"])
    got = get(node)
    assert got.found and got.watch_off == ["light", "sound"]
    assert get(node, map_id=MAP).watch_off == ["light", "sound"]
    save(node, [ID_A, ID_B])                               # 다시 모두 켬
    assert get(node).watch_off == []


def test_all_three_cannot_be_switched_off(node):
    save(node, [ID_A, ID_B], watch_off=["light"])
    refused = save(node, [ID_A, ID_B], watch_off=["intruder", "light", "sound"])
    assert (refused.accepted, refused.reason) == (False, "nothing_watched")
    assert refused.message == "감시 항목은 하나 이상 켜 두어야 합니다."
    assert node._patrol_routes.read(MAP) == ([ID_A, ID_B], ["light"])


def test_the_watch_setting_outlives_a_deleted_route(node):
    save(node, [ID_A, ID_B], watch_off=["sound"])
    deleted = save(node, [], watch_off=["sound"])
    assert (deleted.accepted, deleted.message) == (True, "순찰 경로를 지웠습니다.")
    got = get(node)
    assert (got.found, got.destination_ids, got.watch_off) == (False, [], ["sound"])
    assert save(node, [], watch_off=["sound"]).message == "지울 순찰 경로가 없었습니다."
    save(node, [])                                         # 경로도 끈 항목도 없으면 파일을 지운다
    assert not node._patrol_routes.path_for(MAP).exists()


def test_saving_for_the_current_map_tells_the_watch_setting_at_once(node):
    save(node, [ID_A, ID_B], watch_off=["light"])
    assert watch_msgs(node) == [{"off": ["light"]}]
    save(node, [ID_A], watch_off=["sound"])                # 거절된 저장은 알리지 않는다
    assert watch_msgs(node) == [{"off": ["light"]}]


def test_saving_for_another_map_does_not_tell(node, tmp_path):
    folder = tmp_path / "destinations" / "map_2"
    folder.mkdir()
    (folder / "destinations.yaml").write_text(
        "destinations:\n"
        f"  - {{id: {ID_A}, name: 다른 층 로비, pose: {{x: 1.0, y: 1.0, yaw: 0.0}}}}\n"
        f"  - {{id: {ID_C}, name: 다른 층 창고, pose: {{x: 2.0, y: 1.0, yaw: 0.0}}}}\n",
        encoding="utf-8")
    assert save(node, [ID_A, ID_C], map_id="map_2", watch_off=["sound"]).accepted
    assert node._pub_patrol_watch.msgs == []
    assert get(node, map_id="map_2").watch_off == ["sound"]


def test_startup_tells_the_saved_setting_of_the_current_map(node):
    node._patrol_routes.write(MAP, [ID_A, ID_B], ["intruder"])
    node._publish_patrol_watch()                           # __init__ 이 발행자를 만든 직후 부른다
    assert watch_msgs(node) == [{"off": ["intruder"]}]
    assert NODE_SRC.count("self._publish_patrol_watch()") == 1


def test_get_for_a_missing_file_says_everything_is_watched(node):
    got = get(node)
    assert (got.found, got.watch_off) == (False, [])


# ---- 배선 계약(소스) --------------------------------------------------------


def test_patrol_adds_exactly_one_subscription_and_no_timer_or_thread():
    assert NODE_SRC.count("create_subscription(") == 25      # 순찰 전 24 + /vica/patrol/event
    assert NODE_SRC.count('"/vica/patrol/event"') == 1
    assert NODE_SRC.count('"/vica/patrol/watch"') == 1       # 감시 항목 알림 — 발행자 하나, 구독 없음
    assert NODE_SRC.count("create_timer(") == 3              # 틱·robot_state·장애물 판정 — 그대로
    assert NODE_SRC.count("threading.Thread(") == 2          # 장애물 입력·주행 취소 — 그대로


def test_services_use_the_spec_names():
    for name in ("patrol_start", "patrol_stop", "patrol_ack"):
        assert f'("{name}", self._on_{name})' in NODE_SRC
    assert 'f"/vica/mission/{name}"' in NODE_SRC
    assert 'SavePatrolRoute, "/vica/patrol/route/save"' in NODE_SRC
    assert 'GetPatrolRoute, "/vica/patrol/route/get"' in NODE_SRC
    assert mm.SavePatrolRoute is not None and mm.GetPatrolRoute is not None


def test_route_service_types_match_the_spec():
    request, response = mm.SavePatrolRoute.Request(), mm.SavePatrolRoute.Response()
    assert request.get_fields_and_field_types() == {
        "map_id": "string", "destination_ids": "sequence<string>",
        "watch_off": "sequence<string>"}
    assert response.get_fields_and_field_types() == {
        "accepted": "boolean", "reason": "string", "message": "string"}
    assert mm.GetPatrolRoute.Request().get_fields_and_field_types() == {"map_id": "string"}
    assert mm.GetPatrolRoute.Response().get_fields_and_field_types() == {
        "found": "boolean", "map_id": "string", "destination_ids": "sequence<string>",
        "names": "sequence<string>", "watch_off": "sequence<string>", "message": "string"}


PATROL_PARAMS = {
    "patrol_dwell_sec": "3.0", "patrol_intruder_repeat_sec": "3.0",
    "patrol_light_repeat_sec": "10.0", "patrol_alarm_resend_sec": "10.0",
    "patrol_detector_timeout_sec": "3.0", "patrol_arming_timeout_sec": "15.0",
    "patrol_speed_mps": "0.6", "patrol_max_leg_failures": "3",
}


def test_node_and_launch_expose_the_patrol_parameters():
    for name, default in PATROL_PARAMS.items():
        assert f'self.declare_parameter("{name}"' in NODE_SRC
        assert f'DeclareLaunchArgument("{name}", default_value="{default}")' in LAUNCH_SRC
        assert f'"{name}"' in LAUNCH_SRC.split("parameters=[")[1]
    assert 'self.declare_parameter("patrol_media_dir", "~/workspaces/isaac_ros-dev/vica_patrol")' in NODE_SRC
    assert '"patrol_media_dir": ParameterValue(' in LAUNCH_SRC


def test_launch_file_still_loads():
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "mission_manager_launch", PKG / "launch" / "mission_manager.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    from launch.actions import DeclareLaunchArgument

    names = [e.name for e in module.generate_launch_description().entities
             if isinstance(e, DeclareLaunchArgument)]
    for name in (*PATROL_PARAMS, "patrol_media_dir"):
        assert name in names
