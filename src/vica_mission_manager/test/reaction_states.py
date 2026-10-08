"""미션 요청 반응표(2026-10-08) 시험용 상태 만들기 — 실제 흐름과 같은 공개 함수만 쓴다.

상태마다 새 MissionLogic 을 그 상태로 데려가고 (logic, 다음 시각)을 돌려준다. 필드를 직접
고치지 않는다 — 실제 흐름이 채우는 내부 값(질문 종류·대기 장소·복귀 사다리 등)이 그대로
남아야 칸의 동작이 실기와 같다. 정본 설계: docs/superpowers/specs/2026-10-08-mission-request-
reactions-design.md
"""
from vica_mission_manager.mission_logic import (
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


# ---- 상태 ------------------------------------------------------------------
def idle():
    return new_logic(), 1.0


def navigating(d=ROOM, t=0.0):
    logic = new_logic()
    logic.on_intent(go(d), d, BOUNDS, True, t)
    return logic, t + 1.0


def asking(d=TOILET):
    """도착 질문. TOILET(restroom)은 대기형 "다녀오시는 동안 여기서 기다릴까요?"."""
    logic = new_logic()
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
    logic.on_tick(10.5, NavStatus.NONE)          # 8초 침묵 → 같은 질문 한 번 더
    logic.on_arrival_question_spoken(11.0)
    logic.on_tick(19.5, NavStatus.NONE)          # 또 침묵 → 떠나기 예고(3초 유예)
    logic.on_tick(23.0, NavStatus.NONE)          # 유예 끝 → 홈으로
    return logic, 24.0


def idle_braked(d=TOILET):
    """홈 가다 "비카야"로 세운 뒤 — 15초 재개 사다리가 걸린 IDLE. TOILET 은 대기 장소가 없다."""
    logic, t = returning(d)
    logic.on_return_brake(t)
    return logic, t + 1.0


def idle_braked_spot():
    """대기 장소가 있는 목적지(SPOT_DEST)에서 안내를 마치고 홈 가다 세운 뒤."""
    return idle_braked(SPOT_DEST)


def confirming():
    logic = new_logic()
    logic.on_intent(intent(matched_destination_id=TOILET.id, need_confirm=True),
                    TOILET, BOUNDS, True, 0.0)
    return logic, 1.0


def confirming_change():
    """안내 주행 중 다른 목적지 제안 → 멈추고 바꿀지 묻는 중."""
    logic, t = navigating(ROOM)
    logic.on_intent(intent(matched_destination_id=TOILET.id, need_confirm=True),
                    TOILET, BOUNDS, True, t)
    return logic, t + 1.0


def paused():
    logic, t = navigating()
    logic.on_pause_request(t)
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


def asking_wait_time():
    logic, t = asking(dest("reception", id="desk", name="안내데스크"))
    logic.on_arrival_answer(intent("affirm"), t)
    logic.on_arrival_question_spoken(t + 1.0)
    return logic, t + 2.0


def waiting_release(minutes=10):
    logic = new_logic()
    logic.on_intent(go(SPOT_DEST), SPOT_DEST, BOUNDS, True, 0.0)
    logic.on_tick(1.0, NavStatus.SUCCEEDED)
    logic.on_arrival_question_spoken(2.0)
    logic.on_arrival_answer(intent("wait", wait_minutes=minutes), 3.0)
    return logic, 4.0


def moving_to_wait_spot():
    logic, _ = waiting_release()
    logic.on_wait_speech_spoken(logic._release_text, 7.0)
    logic.on_tick(7.1, NavStatus.NONE)
    return logic, 7.5


def waiting():
    logic, _ = moving_to_wait_spot()
    logic.on_tick(8.0, NavStatus.SUCCEEDED)
    return logic, 9.0


def waiting_asked():
    """대기 중 "다 됐어" → "네, 어디로 모실까요?"를 물은 직후(30초 안)."""
    logic, t = waiting()
    logic.on_intent(intent("finish"), None, BOUNDS, True, t)
    return logic, t + 1.0


def moving_back_to_dest():
    logic, _ = moving_to_wait_spot()
    logic.on_tick(40.0, NavStatus.FAILED)
    return logic, 41.0


def approaching():
    logic = new_logic()
    logic.on_approach_request(APPROACH, BOUNDS, True, 0.0)
    return logic, 1.0


def awaiting_user():
    logic, _ = approaching()
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


def estopped():
    logic, t = navigating()
    logic.on_estop(True, t)
    return logic, t + 1.0


# 반응표의 행. 상태 이름 정본은 State 다 — 같은 상태의 갈래(세운 뒤·바꾸기 질문·늦은 답)는
# 반응표 칸의 쪽지에 적힌 경우를 따로 세운 것이다.
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
}

# 반응표의 열 — 음성 요청 11종. 목적지 요청은 지금 가는 곳과 다른 엘리베이터로 한다.
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
}
