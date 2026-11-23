"""도킹 흐름(dock_flow.py)과 미션 상태 기계의 도킹 막기(2026-11-18)."""
import math
from pathlib import Path

from vica_mission_manager.dock_flow import (
    CALL_CANCEL, CALL_START, CALL_UNDOCK, MODE_DOCKED, MODE_DOCKING, MODE_NONE, MODE_UNDOCKING,
    DockFlow, classify_start, staging_relation,
)
from vica_mission_manager.dock_spot import (
    BUMPER_X, DOCK_DESTINATION_ID, read_dock_spot, staging_destination,
)
from vica_mission_manager.mission_logic import (
    MSG_DOCK_GO, MSG_DOCK_WAIT, MSG_DOCK_WAKE, MissionLogic, Navigate, NavStatus, Say, SpinInPlace,
    State, WakeReply,
)

STAGING = object()


def _st(seq, state="idle", action="", result="", reason="", near=False):
    return {"seq": seq, "state": state, "near_dock": near,
            "last": {"action": action, "result": result, "reason": reason}}


def _kinds(cmds):
    return [(c.kind, c.arg) for c in cmds]


def test_start_goes_to_staging_then_enters_then_docked():
    f = DockFlow()
    ok, reason, cmds = f.request_start(True, False, True, STAGING)
    assert ok and reason == "ok" and _kinds(cmds) == [("nav", STAGING)]
    assert f.mode == MODE_DOCKING and f.owns_nav()
    assert _kinds(f.on_nav_status("running")) == []
    assert _kinds(f.on_nav_status("succeeded")) == [("call", CALL_START)]
    assert f.on_node_state(_st(1, "docking", "dock")) == []
    cmds = f.on_node_state(_st(2, "docked", "dock", "success"))
    assert _kinds(cmds) == [("event", "dock_succeeded")]
    assert f.mode == MODE_DOCKED and not f.busy() and f.blocks_motion()


def test_start_near_dock_skips_nav():
    f = DockFlow()
    f.on_node_state(_st(1, "idle", near=True))
    ok, _, cmds = f.request_start(True, False, False, STAGING)
    assert ok and _kinds(cmds) == [("call", CALL_START)]


def test_not_found_at_staging_renavigates_once_then_fails():
    f = DockFlow()
    f.request_start(True, False, True, STAGING)
    f.on_nav_status("succeeded")
    f.on_node_state(_st(1, "docking", "dock"))
    cmds = f.on_node_state(_st(2, "idle", "dock", "failed", "not_found"))
    assert _kinds(cmds) == [("restage", None)]
    cmds = f.restage(STAGING, None)
    assert _kinds(cmds) == [("nav", STAGING)]
    f.on_nav_status("succeeded")
    f.on_node_state(_st(3, "docking", "dock"))
    cmds = f.on_node_state(_st(4, "idle", "dock", "failed", "not_found"))
    assert [c.kind for c in cmds] == ["event"] and cmds[0].arg == "dock_failed"
    assert cmds[0].message == "not_found" and f.mode == MODE_NONE


def test_rejections():
    f = DockFlow()
    assert f.request_start(True, True, True, STAGING)[1] == "estop"
    assert f.request_start(True, False, True, None)[1] == "no_dock"
    assert f.request_start(False, False, True, STAGING)[1] == "busy"
    assert f.request_undock(True, False)[1] == "not_docked"
    assert f.request_cancel()[1] == "idle"


def test_docked_at_boot_then_remote_job_undocks_first_then_releases():
    f = DockFlow()
    f.on_node_state(_st(1, "docked", near=True))
    assert f.mode == MODE_DOCKED and f.blocks_motion()
    job = object()
    cmds = f.hold(job)
    assert _kinds(cmds) == [("call", CALL_UNDOCK)] and f.mode == MODE_UNDOCKING
    f.on_node_state(_st(2, "undocking", "undock"))
    cmds = f.on_node_state(_st(3, "idle", "undock", "success"))
    assert _kinds(cmds) == [("release_held", job)] and f.mode == MODE_NONE
    assert not f.blocks_motion()


def test_undock_blocked_drops_held_job_and_stays_docked():
    f = DockFlow()
    f.on_node_state(_st(1, "docked", near=True))
    job = object()
    f.hold(job)
    f.on_node_state(_st(2, "undocking", "undock", near=True))
    cmds = f.on_node_state(_st(3, "docked", "undock", "failed", "blocked", near=True))
    assert _kinds(cmds) == [("drop_held", job), ("event", "undock_failed")]
    assert f.mode == MODE_DOCKED


def test_cancel_while_going_cancels_nav_and_while_entering_calls_dock_cancel():
    f = DockFlow()
    f.request_start(True, False, True, STAGING)
    ok, _, cmds = f.request_cancel()
    assert ok and _kinds(cmds) == [("cancel_nav", None)] and f.mode == MODE_NONE
    f.request_start(True, False, True, STAGING)
    f.on_nav_status("succeeded")
    ok, _, cmds = f.request_cancel()
    assert ok and _kinds(cmds) == [("call", CALL_CANCEL)]
    f.on_node_state(_st(5, "docking", "dock"))
    cmds = f.on_node_state(_st(6, "idle", "dock", "failed", "canceled", near=True))
    assert cmds == [] and f.mode == MODE_NONE
    assert f.blocks_motion()


def test_mission_cancel_during_held_undock_cancels_undock():
    f = DockFlow()
    f.on_node_state(_st(1, "docked", near=True))
    f.hold(object())
    assert _kinds(f.on_mission_cancel()) == [("call", CALL_CANCEL)]


def test_dock_node_rejection_folds_flow():
    f = DockFlow()
    f.request_start(True, False, True, STAGING)
    f.on_nav_status("succeeded")
    cmds = f.on_call_result(CALL_START, False, "busy")
    assert [c.arg for c in cmds] == ["dock_failed"] and not f.busy()


def test_staging_destination_from_dock_yaml(tmp_path: Path):
    p = tmp_path / "dock.yaml"
    p.write_text("map_id: m1\napex_x: 2.0\napex_y: 1.0\nfwd_yaw: 1.5708\nstage_cm: 100.0\n"
                 "target_cm: 14.0\nvisited_ok: false\n", encoding="utf-8")
    spot = read_dock_spot(p)
    d = staging_destination(spot)
    assert d.id == DOCK_DESTINATION_ID
    assert math.isclose(d.pose.x, 2.0, abs_tol=1e-3)
    assert math.isclose(d.pose.y, 1.0 - (BUMPER_X + 1.0), abs_tol=1e-3)
    assert math.isclose(d.pose.yaw_deg, 90.0, abs_tol=0.01)
    assert read_dock_spot(tmp_path / "none.yaml") is None


def test_wake_while_docked_says_dock_ment_and_does_not_listen():
    logic = MissionLogic()
    logic.dock_mode = MODE_DOCKED
    acts = logic.on_wake_call(0.0)
    assert Say(MSG_DOCK_WAKE, priority="response") in acts
    assert WakeReply(listen=False) in acts
    assert MSG_DOCK_WAKE == "비카가 밥 먹고 있어요. 밥 먹을때는 개도 안 건드린다는 속담이 있죠"
    logic.dock_mode = MODE_UNDOCKING
    assert logic.on_wake_call(0.0) == [WakeReply(listen=False)]


def test_dialog_state_shows_dock_mode():
    logic = MissionLogic()
    assert logic.dialog_state == State.IDLE.value
    logic.dock_mode = MODE_DOCKED
    assert logic.dialog_state == "docked"
    logic.dock_mode = MODE_DOCKING
    assert logic.dialog_state == "docking"
    logic.dock_mode = MODE_UNDOCKING
    logic.state = State.NAVIGATING
    assert logic.dialog_state == "undocking"


def test_wake_direction_turn_blocked_while_docked():
    logic = MissionLogic()
    logic.dock_mode = MODE_DOCKED
    assert logic.on_wake_doa(90.0, True, 0.0) == []
    assert logic.on_wake(0.0) == []


def test_no_navigate_leaks_from_wake_when_docked():
    logic = MissionLogic()
    logic.dock_mode = MODE_DOCKED
    acts = logic.on_wake_call(0.0) + logic.on_wake_doa(170.0, True, 0.1)
    assert not any(isinstance(a, Navigate) for a in acts)


def test_near_staging_spins_instead_of_nav():
    """11-19 실기: 도크에서 나온 직후(준비 위치에서 0.16 m, 방향만 반대) Nav2 는 'Path is empty' 로 실패했다."""
    f = DockFlow()
    ok, reason, cmds = f.request_start(True, False, True, STAGING, near_spin=3.1)
    assert ok and reason == "ok"
    assert [c.kind for c in cmds] == ["spin"] and cmds[0].arg == 3.1
    assert f.mode == "docking" and f.owns_nav()
    cmds = f.on_nav_status("succeeded")
    assert [(c.kind, c.arg) for c in cmds] == [("call", "start")]
    cmds = f.on_node_state({"seq": 1, "state": "idle", "near_dock": False,
                            "last": {"action": "dock", "result": "failed", "reason": "out_of_range"}})
    assert [(c.kind, c.arg) for c in cmds] == [("restage", None)]
    assert [(c.kind, c.arg) for c in f.restage(STAGING, 0.1)] == [("call", "start")]
    cmds = f.on_node_state({"seq": 2, "state": "idle", "near_dock": False,
                            "last": {"action": "dock", "result": "failed", "reason": "out_of_range"}})
    assert [(c.kind, c.arg, c.message) for c in cmds] == [("event", "dock_failed", "out_of_range")]
    assert f.mode == "" and not f.busy()


def test_near_staging_already_facing_enters_directly():
    f = DockFlow()
    ok, _, cmds = f.request_start(True, False, False, STAGING, near_spin=0.2)
    assert ok and [(c.kind, c.arg) for c in cmds] == [("call", "start")]


def test_near_staging_spin_failure_reports_nav_failed():
    f = DockFlow()
    f.request_start(True, False, True, STAGING, near_spin=-2.0)
    cmds = f.on_nav_status("failed")
    assert [(c.kind, c.arg, c.message) for c in cmds] == [("event", "dock_failed", "nav_failed")]
    assert f.request_start(True, False, False, STAGING, near_spin=-2.0)[1] == "nav_not_ready"


def test_undock_pending_allows_resume_and_blocks_voice():
    """11-19 사용자: 나오다 멈추면(중지·비상정지) '도크에서 나오기'로 이어서 한다. 그 상태에서 음성은"""
    f = DockFlow()
    f.on_node_state({"seq": 1, "state": "idle", "near_dock": False, "undock_pending": True,
                     "last": {"action": "undock", "result": "failed", "reason": "estop"}})
    assert f.undock_pending and not f.blocks_motion()
    ok, reason, cmds = f.request_undock(True, False)
    assert ok and [(c.kind, c.arg) for c in cmds] == [("call", "undock")]
    assert f.mode == "undocking"
    cmds = f.on_node_state({"seq": 2, "state": "idle", "near_dock": False, "undock_pending": False,
                            "last": {"action": "undock", "result": "success", "reason": ""}})
    assert [(c.kind, c.arg) for c in cmds] == [("event", "undock_succeeded")]
    assert not f.undock_pending
    assert f.request_undock(True, False)[1] == "not_docked"

    logic = MissionLogic()
    logic.dock_pending = True
    acts = logic.on_wake_call(0.0)
    assert Say(MSG_DOCK_WAIT, priority="response") in acts
    assert WakeReply(listen=False) in acts
    assert MSG_DOCK_WAIT == "비카가 아직 밥을 덜 먹었어요. 조금만 기다려주세요"
    assert logic.dialog_state == "idle"


def test_cancel_then_success_does_not_release_held_drive():
    """11-19: 회전이 10° 안으로 남았을 때 멈추면 도킹 노드는 '성공'으로 끝낸다. 중지를 눌렀다면 잡아 둔 주행은 버린다."""
    f = DockFlow()
    f.on_node_state({"seq": 1, "state": "docked", "near_dock": True, "last": {}})
    f.hold("주행")
    ok, _, cmds = f.request_cancel()
    assert ok and [(c.kind, c.arg) for c in cmds] == [("call", "cancel")]
    cmds = f.on_node_state({"seq": 2, "state": "idle", "near_dock": False, "undock_pending": False,
                            "last": {"action": "undock", "result": "success", "reason": ""}})
    assert [(c.kind, c.arg) for c in cmds] == [("drop_held", "주행")]
    assert f.mode == "" and not f.busy()


def test_dock_go_phrase_is_fixed():
    """11-19 사용자: 도크에 들어갈 때 '도킹 위치로 복귀합니다' 안내."""
    assert MSG_DOCK_GO == "도킹 위치로 복귀합니다"


def test_retry_rereads_dock_spot_and_spins_when_near():
    """11-19: 준비 위치에서 V자가 범위 밖이면 도킹 노드가 본 값으로 도크 위치를 고쳐 둔다. 미션은 고친 준비 위치로"""
    f = DockFlow()
    f.request_start(True, False, True, STAGING)
    f.on_nav_status("succeeded")
    cmds = f.on_node_state({"seq": 1, "state": "idle", "near_dock": False,
                            "last": {"action": "dock", "result": "failed", "reason": "out_of_range"}})
    assert [(c.kind, c.arg) for c in cmds] == [("restage", None)]
    new_staging = object()
    cmds = f.restage(new_staging, 1.2)
    assert [(c.kind, c.arg) for c in cmds] == [("spin", 1.2)]
    assert f.staging is new_staging and f.owns_nav() and f.mode == "docking"
    assert [(c.kind, c.arg) for c in f.on_nav_status("succeeded")] == [("call", "start")]
    cmds = f.on_node_state({"seq": 2, "state": "idle", "near_dock": False,
                            "last": {"action": "dock", "result": "failed", "reason": "out_of_range"}})
    assert [(c.kind, c.arg, c.message) for c in cmds] == [("event", "dock_failed", "out_of_range")]

    g = DockFlow()
    g.request_start(True, False, True, STAGING)
    g.on_nav_status("succeeded")
    g.on_node_state({"seq": 1, "state": "idle", "near_dock": False,
                     "last": {"action": "dock", "result": "failed", "reason": "not_found"}})
    assert [(c.kind, c.arg) for c in g.restage(STAGING, 0.1)] == [("call", "start")]


def test_intercept_drops_mission_drive_while_docking():
    """가는 중·들어가는 중에 온 미션 주행은 보내지 않는다 — 예전에는 그대로 Nav2 로 나가 도킹 노드와 다퉜다."""
    f = DockFlow()
    f.request_start(True, False, True, STAGING)
    assert f.phase == "going"
    assert f.intercept("주행", False) == ("drop", [])
    f.on_nav_status("succeeded")
    assert f.phase == "entering"
    assert f.intercept("주행", False) == ("drop", [])
    assert f.intercept("회전", True) == ("drop", [])
    assert f.held is None and f.phase == "entering"


def test_intercept_while_leaving_holds_latest_and_drops_spin():
    f = DockFlow()
    f.on_node_state({"seq": 1, "state": "docked", "near_dock": True, "last": {}})
    verdict, cmds = f.intercept("A", False)
    assert verdict == "held" and _kinds(cmds) == [("call", CALL_UNDOCK)]
    assert f.intercept("B", False) == ("held", []) and f.held == "B"
    assert f.intercept("회전", True) == ("drop", []) and f.held == "B"
    f.request_cancel()
    assert f.intercept("C", False) == ("drop", [])


def test_intercept_does_not_undock_for_a_spin():
    """제자리 회전 하나 때문에 도크에서 나오지 않는다(예: 도크 앞에서 '비카야' 방향 회전)."""
    f = DockFlow()
    f.on_node_state({"seq": 1, "state": "idle", "near_dock": True, "last": {}})
    assert f.intercept("회전", True) == ("drop", [])
    assert not f.busy()
    f2 = DockFlow()
    assert f2.intercept("주행", False) == ("pass", [])


def test_mission_cancel_followed_by_new_drive_keeps_leaving():
    """도크 안에서 A 로 보냄(나오는 중) → B 로 다시 보냄: 나오기는 멈추지 않고 잡아 둔 주행만 바꾼다."""
    f = DockFlow()
    f.on_node_state({"seq": 1, "state": "docked", "near_dock": True, "last": {}})
    f.intercept("A", False)
    assert f.on_mission_cancel(keep_leaving=True) == []
    assert f.phase == "leaving" and not f.canceling and f.held is None
    assert f.intercept("B", False) == ("held", [])
    cmds = f.on_node_state({"seq": 2, "state": "idle", "near_dock": False,
                            "last": {"action": "undock", "result": "success", "reason": ""}})
    assert _kinds(cmds) == [("release_held", "B")]


def test_dock_node_restart_mid_job_folds_flow():
    """도킹 노드가 일 도중에 다시 뜨면 번호(seq)가 1 부터 다시 온다 — 버리지 않고 흐름을 접는다."""
    f = DockFlow()
    f.on_node_state({"seq": 40, "state": "docked", "near_dock": True, "last": {}})
    f.intercept("주행", False)
    assert f.phase == "leaving"
    cmds = f.on_node_state({"seq": 1, "state": "idle", "near_dock": False, "last": {}})
    assert [(c.kind, c.arg, c.message) for c in cmds] == [
        ("drop_held", "주행", ""), ("event", "undock_failed", "dock_node_restarted")]
    assert not f.busy() and f.node_seq == 1
    f.on_node_state({"seq": 2, "state": "docked", "near_dock": True, "last": {}})
    assert f.mode == MODE_DOCKED


def test_cancel_with_dock_node_down_folds_flow():
    f = DockFlow()
    f.on_node_state({"seq": 1, "state": "idle", "near_dock": True, "last": {}})
    f.request_start(True, False, True, STAGING)
    assert f.phase == "entering"
    f.request_cancel()
    cmds = f.on_call_result(CALL_CANCEL, False, "dock_node_down")
    assert [(c.kind, c.arg, c.message) for c in cmds] == [("event", "dock_failed", "dock_node_down")]
    assert not f.busy()
    assert f.on_call_result(CALL_CANCEL, False, "idle") == []


def test_staging_relation_and_start_class():
    along, lateral, turn = staging_relation(10.3, 5.1, 180.0, 10.0, 5.0, 0.0)
    assert abs(along - 0.3) < 1e-9 and abs(lateral - 0.1) < 1e-9 and abs(abs(turn) - math.pi) < 1e-9
    along, lateral, turn = staging_relation(10.2, 5.0, 60.0, 10.0, 5.0, 90.0)
    assert abs(along) < 1e-9 and abs(lateral + 0.2) < 1e-9 and abs(turn - math.radians(30)) < 1e-9
    assert classify_start(0.0, 0.0) == "spin"
    for along in (0.05, 0.15, 0.25, 0.30):
        assert classify_start(along, 0.03) == "spin", along
    assert classify_start(0.31, 0.0) == "zone"
    assert classify_start(0.0, 0.40) == "far"
    assert classify_start(-0.80, 0.0) == "far"
    assert classify_start(0.40, 0.20) == "zone"
    assert classify_start(0.86, 0.0) == "zone"
    assert classify_start(0.40, 0.60) == "far"


def test_start_in_dock_zone_enters_without_nav():
    """도킹 노드의 near_dock 이 내려가 있어도, 로봇 위치가 도크 구역 안이면 Nav2 를 쓰지 않고 바로 들어간다."""
    f = DockFlow()
    ok, _, cmds = f.request_start(True, False, False, STAGING, in_zone=True)
    assert ok and _kinds(cmds) == [("call", CALL_START)]
    assert _kinds(f.restage(STAGING, None, True)) == [("call", CALL_START)]


def test_logic_near_dock_blocks_wake_turn_but_listens():
    logic = MissionLogic()
    logic.dock_near = True
    assert logic.on_wake_doa(90.0, True, 0.0) == []
    acts = logic.on_wake_call(0.0)
    assert not any(isinstance(a, Say) and a.text in (MSG_DOCK_WAKE, MSG_DOCK_WAIT) for a in acts)


def test_logic_idle_timers_do_not_fire_while_docking():
    """'도킹 시작' 직전에 걸려 있던 고개 원위치 회전·끊긴 복귀 재개가 도킹 중·직후에 주행을 내지 않는다."""
    for flag in ("dock_mode", "dock_pending", "dock_near"):
        logic = MissionLogic()
        logic._seek_deadline = 1.0
        logic._seek_return_yaw = 1.5
        logic._return_interrupted = True
        logic._return_resume_deadline = 1.0
        logic._return_notice_given = True
        setattr(logic, flag, MODE_DOCKING if flag == "dock_mode" else True)
        acts = logic.on_tick(100.0, NavStatus.NONE)
        assert not any(isinstance(a, (Navigate, SpinInPlace)) for a in acts), flag
        assert logic.state == State.IDLE and not logic._return_interrupted, flag
        setattr(logic, flag, "" if flag == "dock_mode" else False)
        acts = logic.on_tick(200.0, NavStatus.NONE)
        assert not any(isinstance(a, (Navigate, SpinInPlace)) for a in acts), flag


def test_held_patrol_leg_dropped_ends_patrol_instead_of_next_leg():
    """도크 안에서 경비 시작 → 나오는 중 '나오기 중지': 다음 구간으로 넘어가지 않고 경비를 끝낸다(11-19 재검토)."""
    from vica_mission_manager.mission_logic import (
        Destination as D, MapBounds, PatrolEvent, Pose2D as P2, SetNavSpeedLimit)
    route = (D(id="a", name="로비", pose=P2(1.0, 0.0, 10.0, "map"), calibrated=True),
             D(id="b", name="복도", pose=P2(1.0, 2.0, 20.0, "map"), calibrated=True))
    logic = MissionLogic()
    _, reason = logic.on_patrol_start(route, MapBounds(-50, -50, 50, 50), True, True, 0.0)
    logic.on_patrol_event({"src": "detector", "kind": "status", "ready": True, "camera_ok": True}, 0.1)
    acts = logic.on_tick(0.2, NavStatus.NONE)
    assert any(isinstance(a, Navigate) for a in acts) and logic.state == State.PATROLLING
    acts = logic.on_dock_held_dropped(1.0)
    assert not any(isinstance(a, (Navigate, SpinInPlace)) for a in acts)
    assert PatrolEvent("patrol_stopped", reason="nav_failed") in acts
    assert logic.state == State.IDLE


def test_held_drive_dropped_outside_patrol_is_a_cancel():
    logic = MissionLogic()
    logic.state = State.NAVIGATING
    acts = logic.on_dock_held_dropped(1.0)
    assert not any(isinstance(a, (Navigate, SpinInPlace)) for a in acts)
    assert logic.state != State.NAVIGATING
