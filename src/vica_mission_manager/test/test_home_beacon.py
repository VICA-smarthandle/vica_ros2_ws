"""Home 알림 (2026-10-08 사용자 요청).

Home 에서 쉬고 있으면(IDLE · 홈 0.5 m 안) 1분마다 M3 "비카가 대기 중입니다."를 가장 낮은
ambient 등급으로 말한다 — 사람이 소리를 듣고 로봇이 어디 있는지 알게 한다. 대화 중이면
건너뛰고(박자는 유지), 홈을 벗어나거나 IDLE 이 아니게 되면 박자를 처음부터 다시 잰다.
"""
from vica_mission_manager.mission_logic import (
    Destination, MissionLogic, NavStatus, Pose2D, Say, State,
    HOME_BEACON_INTERVAL_SEC, HOME_BEACON_RADIUS_M, MSG_WAIT_BEACON,
)

HOME = Destination(id="__home__", name="홈", pose=Pose2D(2.0, -1.0, 0.0, "map"))
AT_HOME = Pose2D(2.1, -0.9, 30.0, "map")          # 홈에서 약 0.14 m
AWAY = Pose2D(2.0 + HOME_BEACON_RADIUS_M + 0.1, -1.0, 0.0, "map")


def _logic(**kw):
    logic = MissionLogic(return_destination=HOME, **kw)
    logic.robot_pose = AT_HOME
    return logic


def _beacons(actions):
    return [(a.text, a.priority) for a in actions if isinstance(a, Say)]


def _run(logic, t0, t1, step=1.0):
    """t0..t1 을 1초 간격으로 돌리며 말한 시각을 모은다."""
    said = []
    t = t0
    while t <= t1 + 1e-9:
        if _beacons(logic.on_tick(t, NavStatus.NONE)):
            said.append(t)
        t += step
    return said


def test_speaks_m3_every_minute_at_home_as_ambient():
    logic = _logic()
    said = _run(logic, 0.0, 185.0)
    # 첫 tick 에 박자를 잡고, 1분 뒤부터 1분마다.
    assert said == [60.0, 120.0, 180.0]
    acts = logic.on_tick(240.0, NavStatus.NONE)
    assert _beacons(acts) == [(MSG_WAIT_BEACON, "ambient")]
    assert HOME_BEACON_INTERVAL_SEC == 60.0
    assert MSG_WAIT_BEACON == "비카가 대기 중입니다."


def test_silent_when_away_from_home():
    logic = _logic()
    logic.robot_pose = AWAY
    assert _run(logic, 0.0, 200.0) == []


def test_silent_without_pose_or_home():
    logic = _logic()
    logic.robot_pose = None
    assert _run(logic, 0.0, 200.0) == []
    no_home = MissionLogic()
    no_home.robot_pose = AT_HOME
    assert _run(no_home, 0.0, 200.0) == []


def test_silent_when_not_idle():
    logic = _logic()
    logic.on_tick(0.0, NavStatus.NONE)
    logic.state = State.CONFIRMING
    logic._confirm_deadline = None
    assert _run(logic, 1.0, 200.0) == []


def test_rhythm_restarts_after_leaving_home():
    logic = _logic()
    _run(logic, 0.0, 50.0)
    # 50 s 에 홈을 잠깐 벗어났다가 70 s 에 돌아온다 — 60 s 에는 말하지 않고,
    # 돌아온 70 s 부터 다시 1분을 잰다.
    logic.robot_pose = AWAY
    assert _run(logic, 51.0, 69.0) == []
    logic.robot_pose = AT_HOME
    assert _run(logic, 70.0, 135.0) == [130.0]


def test_skips_while_listening_but_keeps_rhythm():
    logic = _logic()
    _run(logic, 0.0, 55.0)
    logic.on_listen_state("speech", 56.0)
    # 60 s 차례는 대화 중이라 건너뛴다. 몰아서 말하지 않고 다음 박자(120 s)에 말한다.
    assert _run(logic, 56.0, 65.0) == []
    logic.on_listen_state("closed", 66.0)
    assert _run(logic, 66.0, 125.0) == [120.0]


def test_interval_zero_turns_it_off():
    logic = _logic(home_beacon_interval_sec=0.0)
    assert _run(logic, 0.0, 300.0) == []
