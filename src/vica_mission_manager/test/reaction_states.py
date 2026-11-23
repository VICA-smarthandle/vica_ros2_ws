"""미션 요청 반응표(2026-10-08) 시험용 상태 만들기 — 실제 흐름과 같은 공개 함수만 쓴다."""
from vica_mission_manager.mission_logic import (
    WAIT_DEPART_AFTER_SPEECH_SEC,
    ApproachRequest,
    Destination,
    IntentData,
    MapBounds,
    MissionLogic,
    NavStatus,
    Pose2D,
    WaitSpot,
)

BOUNDS = MapBounds(min_x=-50, min_y=-50, max_x=50, max_y=50)
HOME = Destination(id="__home__", name="홈", pose=Pose2D(0, 0, 0, "map"))
SPOT = WaitSpot(x=1.97, y=-1.42, yaw_deg=0.0, side="right")
APPROACH = ApproachRequest(goal=Pose2D(1.0, 0.5, 30.0, "map"), track_id=7)


def dest(category="", **kw) -> Destination:
    d = dict(id="d1", name="화장실", pose=Pose2D(3, 2, 90, "map"), calibrated=True,
             arrival_message="화장실 앞에 도착했습니다.", category=category)
    d.update(kw)
    return Destination(**d)


ROOM = dest(id="r409", name="409호", pose=Pose2D(5, 1, 0, "map"), arrival_message="")
TOILET = dest("restroom", id="wc", pose=Pose2D(-3, 2, 90, "map"))
SPOT_DEST = dest("restroom", id="d1", wait_spot=SPOT, door_yaw_deg=270.0, name="화장실 입구",
                 pose=Pose2D(3.21, -1.05, 90.0, "map"),
                 arrival_message="화장실 입구 앞에 도착했습니다.")
ELEV = dest(id="elev", name="엘리베이터", pose=Pose2D(6, -2, 0, "map"), arrival_message="")
ALL = {d.id: d for d in (ROOM, TOILET, SPOT_DEST, ELEV)}


def lookup(dest_id: str):
    return ALL.get(dest_id) if dest_id else None


def intent(kind="navigate", **kw) -> IntentData:
    d = dict(intent=kind, matched_destination_id="", need_confirm=False, safety_flag="normal")
    d.update(kw)
    return IntentData(**d)


def go(d: Destination) -> IntentData:
    return intent(matched_destination_id=d.id)


def new_logic(**kw) -> MissionLogic:
    kw.setdefault("return_destination", HOME)
    kw.setdefault("arrival_dialog", True)
    return MissionLogic(**kw)


def idle(**kw):
    return new_logic(**kw), 1.0


def navigating(d=ROOM, t=0.0, **kw):
    logic = new_logic(**kw)
    logic.on_intent(go(d), d, BOUNDS, True, t)
    return logic, t + 1.0


def navigating_cancel_asked(**kw):
    """안내 주행 중 "취소" → "안내를 취소할까요?"를 물은 채 주행 계속(Task 3.9)."""
    logic, t = navigating(**kw)
    logic.on_voice_intent(intent("cancel"), t, lookup, BOUNDS, True)
    return logic, t + 1.0


def navigating_help_asked(**kw):
    """안내 주행 중 도움 요청 → "직원의 도움이 필요하신가요?"를 물은 채 주행 계속(Task 3.9)."""
    logic, t = navigating(**kw)
    logic.on_voice_intent(intent("help"), t, lookup, BOUNDS, True)
    return logic, t + 1.0


def help_requested_confirm_asked(**kw):
    """도움 요청을 보낸 뒤(일시정지, 관리자 확인 전) 엘리베이터 제안 → CONFIRMING 없이 확인 질문만 물은 직후(Task 3.3 수정 1차)."""
    logic, t = navigating_help_asked(**kw)
    logic.on_voice_intent(intent("affirm"), t, lookup, BOUNDS, True)
    logic.on_voice_intent(intent(matched_destination_id=ELEV.id, need_confirm=True), t + 1.0, lookup, BOUNDS, True)
    return logic, t + 2.0


def asking(d=TOILET, **kw):
    """도착 질문. TOILET(restroom)은 대기형 "다녀오시는 동안 여기서 기다릴까요?"."""
    logic = new_logic(**kw)
    logic.on_intent(go(d), d, BOUNDS, True, 0.0)
    logic.on_tick(1.0, NavStatus.SUCCEEDED)
    logic.on_arrival_question_spoken(2.0)
    return logic, 3.0


def returning(d=TOILET):
    """도착 질문에 "다 됐어" → 홈 복귀 중(안내를 마친 뒤)."""
    logic, t = asking(d)
    logic.on_arrival_answer(intent("finish"), t)
    return logic, t + 1.0


def returning_late():
    """도착 질문에 끝내 답이 없어 떠난 직후 — 늦은 답이 올 수 있는 복귀."""
    logic, _ = asking()
    logic.on_tick(10.5, NavStatus.NONE)
    logic.on_arrival_question_spoken(11.0)
    logic.on_tick(19.5, NavStatus.NONE)
    logic.on_tick(23.0, NavStatus.NONE)
    return logic, 24.0


def idle_braked(d=TOILET):
    """홈 가다 "비카야"로 세운 뒤 — 15초 재개 사다리가 걸린 IDLE. TOILET 은 대기 장소가 없다."""
    logic, t = returning(d)
    logic.on_return_brake(t)
    return logic, t + 1.0


def idle_braked_spot():
    """대기 장소가 있는 목적지(SPOT_DEST)에서 안내를 마치고 홈 가다 세운 뒤."""
    return idle_braked(SPOT_DEST)


def confirming(**kw):
    logic = new_logic(**kw)
    logic.on_intent(intent(matched_destination_id=TOILET.id, need_confirm=True),
                    TOILET, BOUNDS, True, 0.0)
    return logic, 1.0


def confirming_change(**kw):
    """안내 주행 중 다른 목적지 제안 → 멈추고 바꿀지 묻는 중."""
    logic, t = navigating(ROOM, **kw)
    logic.on_intent(intent(matched_destination_id=TOILET.id, need_confirm=True),
                    TOILET, BOUNDS, True, t)
    return logic, t + 1.0


def paused(**kw):
    logic, t = navigating(**kw)
    logic.on_pause_request(t)
    return logic, t + 1.0


def paused_resume_asked(**kw):
    """일시정지 중 "다시 가자"(재개 제안) → 미션이 "다시 출발할까요?"를 물은 직후(Task 3.4). 미션이 묻는 것은"""
    kw.setdefault("answer_field", True)
    logic, t = paused(**kw)
    logic.on_voice_intent(intent("resume", need_confirm=True), t, lookup, BOUNDS, True)
    return logic, t + 1.0


def failed():
    logic = new_logic(nav_retry_limit=0)
    logic.on_intent(go(ROOM), ROOM, BOUNDS, True, 0.0)
    logic.on_tick(10.0, NavStatus.FAILED)
    return logic, 10.5


def arrived():
    """관리자 주행 도착(질문 없음)."""
    logic = new_logic()
    logic.on_app_destination(dest("restroom"), BOUNDS, True, 0.0)
    logic.on_tick(10.0, NavStatus.SUCCEEDED)
    return logic, 10.5


def asking_wait_time(**kw):
    logic, t = asking(dest("reception", id="desk", name="안내데스크"), **kw)
    logic.on_arrival_answer(intent("affirm"), t)
    logic.on_arrival_question_spoken(t + 1.0)
    return logic, t + 2.0


def asking_end_confirm(**kw):
    """"몇 분쯤 걸리실까요?"에 "아니(기다리지 마)" → "여기까지 안내를 마칠까요?"(종료형)를 물은 직후(Task 3.3)."""
    logic, t = asking_wait_time(**kw)
    logic.on_voice_intent(intent("deny"), t, lookup, BOUNDS, True)
    return logic, t + 1.0


def waiting_release(minutes=10, **kw):
    logic = new_logic(**kw)
    logic.on_intent(go(SPOT_DEST), SPOT_DEST, BOUNDS, True, 0.0)
    logic.on_tick(1.0, NavStatus.SUCCEEDED)
    logic.on_arrival_question_spoken(2.0)
    logic.on_arrival_answer(intent("wait", wait_minutes=minutes), 3.0)
    return logic, 4.0


def moving_to_wait_spot(**kw):
    logic, _ = waiting_release(**kw)
    logic.on_wait_speech_spoken(logic._release_text, 7.0)
    logic.on_tick(7.0 + WAIT_DEPART_AFTER_SPEECH_SEC + 0.1, NavStatus.NONE)
    return logic, 9.5


def waiting(**kw):
    logic, _ = moving_to_wait_spot(**kw)
    logic.on_tick(10.0, NavStatus.SUCCEEDED)
    return logic, 11.0


def waiting_asked(**kw):
    """대기 중 "다 됐어" → "네, 어디로 모실까요?"를 물은 직후(30초 안)."""
    logic, t = waiting(**kw)
    logic.on_intent(intent("finish"), None, BOUNDS, True, t)
    return logic, t + 1.0


def waiting_need_asked(**kw):
    """대기 중 "취소" → "안내가 필요 없으신가요?"(뒤집힌 네/아니요)를 물은 직후(Task 3.3)."""
    logic, t = waiting(**kw)
    logic.on_voice_intent(intent("cancel"), t, lookup, BOUNDS, True)
    return logic, t + 1.0


def moving_back_to_dest():
    logic, _ = moving_to_wait_spot()
    logic.on_tick(40.0, NavStatus.FAILED)
    return logic, 41.0


def approaching(**kw):
    logic = new_logic(**kw)
    logic.on_approach_request(APPROACH, BOUNDS, True, 0.0)
    return logic, 1.0


def awaiting_user(**kw):
    logic, _ = approaching(**kw)
    logic.on_tick(5.0, NavStatus.SUCCEEDED)
    logic.on_approach_question_spoken(6.0)
    return logic, 7.0


def turning():
    logic, t = awaiting_user()
    logic.on_approach_answer(True, t)
    return logic, t + 1.0


def seeking():
    logic = new_logic(wake_doa_sign=1.0)
    logic.on_wake_doa(90.0, True, 1.0)
    return logic, 2.0


def estopped(**kw):
    logic, t = navigating(**kw)
    logic.on_estop(True, t)
    return logic, t + 1.0


PATROL_ROUTE = (ROOM, TOILET, ELEV)
DETECTOR_OK = {"src": "detector", "kind": "status", "ready": True, "camera_ok": True}


def patrol_dwell():
    """경비 시작 직후 — 감지 노드 준비를 기다리는 중."""
    logic = new_logic()
    logic.on_patrol_start(PATROL_ROUTE, BOUNDS, True, True, 0.0)
    return logic, 1.0


def patrolling():
    """감지 노드가 준비돼 첫 지점으로 가는 중."""
    logic, t = patrol_dwell()
    logic.on_patrol_event(DETECTOR_OK, t)
    logic.on_tick(t, NavStatus.NONE)
    return logic, t + 0.5


def patrol_alarm():
    logic, t = patrolling()
    logic.on_patrol_event({"src": "detector", "kind": "intruder", "conf": 0.8, "hits": 2,
                           "clip_id": "20261114_203455_intruder"}, t)
    return logic, t + 0.5


def patrol_light():
    logic, t = patrolling()
    logic.on_tick(t, NavStatus.SUCCEEDED)
    logic.on_patrol_event({"src": "detector", "kind": "light", "brightness": 35.0,
                           "photo": "light_203500.jpg"}, t + 0.5)
    return logic, t + 1.0


def patrol_paused():
    logic, t = patrolling()
    logic.on_pause_request(t)
    return logic, t + 0.5


ROWS = {
    "IDLE": idle,
    "IDLE_BRAKED": idle_braked,
    "IDLE_BRAKED_SPOT": idle_braked_spot,
    "CONFIRMING": confirming,
    "CONFIRMING_CHANGE": confirming_change,
    "NAVIGATING": navigating,
    "PAUSED": paused,
    "FAILED": failed,
    "ARRIVED": arrived,
    "ASKING_NEXT": asking,
    "ASKING_WAIT_TIME": asking_wait_time,
    "WAITING_RELEASE": waiting_release,
    "MOVING_TO_WAIT_SPOT": moving_to_wait_spot,
    "WAITING": waiting,
    "WAITING_ASKED": waiting_asked,
    "MOVING_BACK_TO_DEST": moving_back_to_dest,
    "APPROACHING": approaching,
    "AWAITING_USER": awaiting_user,
    "TURNING": turning,
    "SEEKING": seeking,
    "RETURNING": returning,
    "RETURNING_LATE": returning_late,
    "ESTOPPED": estopped,
    "PATROLLING": patrolling,
    "PATROL_DWELL": patrol_dwell,
    "PATROL_ALARM": patrol_alarm,
    "PATROL_LIGHT": patrol_light,
    "PATROL_PAUSED": patrol_paused,
}

REQUESTS = {
    "navp": intent(matched_destination_id=ELEV.id, need_confirm=True),
    "navc": intent(matched_destination_id=ELEV.id),
    "wait": intent("wait", wait_minutes=-1),
    "finish": intent("finish"),
    "cancel": intent("cancel"),
    "pause": intent("pause"),
    "resume": intent("resume"),
    "yes": intent("affirm"),
    "no": intent("deny"),
    "talk": intent("question"),
    "bye": intent("dismiss"),
    "again": intent("repeat"),
}
