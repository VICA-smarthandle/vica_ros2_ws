"""미션 요청 반응표 전수 시험 (2026-10-08, 규칙 1·2·결정 5개·다시 묻기).

상태(행) × 음성 요청(열)마다 미션이 하는 말과 바뀐 상태를 못 박는다. 정본 설계:
docs/superpowers/specs/2026-10-08-mission-request-reactions-design.md, 정본 페이지 v3.
"비카야" 열은 test_wake_reaction.py 가 맡는다. 질문·잡담(talk)은 LLM 이 답하고 미션은
말하지 않는다 — 그 칸의 규칙 2는 음성 저장소 시험이 맡는다.

PENDING 의 칸은 아직 구현 전이다(strict xfail). 작업마다 자기 칸을 지운다.
"""
import pytest

from reaction_states import BOUNDS, REQUESTS, ROWS, lookup
from vica_mission_manager.mission_logic import (
    MSG_APPROACH_ACCEPTED,
    MSG_APPROACH_BUSY,
    MSG_APPROACH_DECLINED,
    MSG_ARRIVAL_RETRY,
    MSG_ASK_ENTRANCE,
    MSG_ASK_WAIT_TIME,
    MSG_CANCEL_CONFIRM,
    MSG_CONFIRM_PROMPT_FALLBACK,
    MSG_CONFIRM_TIMEOUT,
    MSG_ESTOP_REJECT,
    MSG_FINISH,
    MSG_NOT_NAVIGATING,
    MSG_NOT_PAUSED,
    MSG_PAUSED,
    MSG_RESUMED,
    MSG_START,
    MSG_WAIT_DEFAULT,
    MSG_WAIT_FINISH_ASK,
    MSG_WAIT_SPOT_CONFIRM,
    MSG_WAIT_SPOT_DEFAULT,
    MSG_WAKE_GREETING,
    Say,
    State,
    say_destination,
)

START_ELEV = say_destination(MSG_START, "엘리베이터")
START_TOILET = say_destination(MSG_START, "화장실")
RESUMED_ROOM = say_destination(MSG_RESUMED, "409호")
ASK_ELEV = say_destination(MSG_CONFIRM_PROMPT_FALLBACK, "엘리베이터")
ASK_TOILET = say_destination(MSG_CONFIRM_PROMPT_FALLBACK, "화장실")
M2P_FRONT = MSG_WAIT_SPOT_DEFAULT.format(place="입구 앞")
M2P_RIGHT = MSG_WAIT_SPOT_DEFAULT.format(place="입구 오른쪽")
M2_RIGHT_10 = MSG_WAIT_SPOT_CONFIRM.format(minutes=10, place="입구 오른쪽")
# 새 문장(설계 7절). 상수는 해당 작업이 만든다 — 여기서는 글자 그대로 못 박는다.
SWITCH_ELEV = "네, 엘리베이터로 안내해드릴까요?"
GOING_ROOM = "지금 409호로 가는 중이에요."
NEED_ASK = "안내가 필요 없으신가요?"

COLS = ("navp", "navc", "wait", "finish", "cancel", "pause", "resume", "yes", "no", "talk")

EXPECT = {
    "IDLE": {
        "navp": ((), State.CONFIRMING),
        "navc": ((START_ELEV,), State.NAVIGATING),
        "wait": ((MSG_NOT_NAVIGATING,), State.IDLE),
        "finish": ((MSG_NOT_NAVIGATING,), State.IDLE),
        "cancel": ((MSG_NOT_NAVIGATING,), State.IDLE),
        "pause": ((MSG_NOT_NAVIGATING,), State.IDLE),
        "resume": ((MSG_NOT_PAUSED,), State.IDLE),
        "yes": ((), State.IDLE),
        "no": ((), State.IDLE),
        "talk": ((), State.IDLE),
    },
    "IDLE_BRAKED": {
        "navp": ((), State.CONFIRMING),
        "navc": ((START_ELEV,), State.NAVIGATING),
        "wait": ((M2P_FRONT,), State.MOVING_BACK_TO_DEST),
        "finish": ((MSG_FINISH,), State.RETURNING),
        "cancel": ((MSG_NOT_NAVIGATING,), State.IDLE),
        "pause": ((MSG_NOT_NAVIGATING,), State.IDLE),
        "resume": ((MSG_FINISH,), State.RETURNING),
        "yes": ((), State.IDLE),
        "no": ((), State.IDLE),
        "talk": ((), State.IDLE),
    },
    "IDLE_BRAKED_SPOT": {
        "navp": ((), State.CONFIRMING),
        "navc": ((START_ELEV,), State.NAVIGATING),
        "wait": ((M2P_RIGHT,), State.WAITING_RELEASE),
        "finish": ((MSG_FINISH,), State.RETURNING),
        "cancel": ((MSG_NOT_NAVIGATING,), State.IDLE),
        "pause": ((MSG_NOT_NAVIGATING,), State.IDLE),
        "resume": ((MSG_FINISH,), State.RETURNING),
        "yes": ((), State.IDLE),
        "no": ((), State.IDLE),
        "talk": ((), State.IDLE),
    },
    "CONFIRMING": {
        "navp": ((), State.CONFIRMING),
        "navc": ((SWITCH_ELEV,), State.CONFIRMING),
        "wait": ((MSG_CONFIRM_TIMEOUT,), State.IDLE),
        "finish": ((MSG_CONFIRM_TIMEOUT,), State.IDLE),
        "cancel": ((MSG_CONFIRM_TIMEOUT,), State.IDLE),
        "pause": ((MSG_WAKE_GREETING,), State.CONFIRMING),
        "resume": ((ASK_TOILET,), State.CONFIRMING),
        "yes": ((START_TOILET,), State.NAVIGATING),
        "no": ((MSG_CONFIRM_TIMEOUT,), State.IDLE),
        "talk": ((), State.CONFIRMING),
    },
    "CONFIRMING_CHANGE": {
        "navp": ((), State.CONFIRMING),
        "navc": ((SWITCH_ELEV,), State.CONFIRMING),
        "wait": ((RESUMED_ROOM,), State.NAVIGATING),
        "finish": ((MSG_CANCEL_CONFIRM,), State.CONFIRMING),
        "cancel": ((MSG_CANCEL_CONFIRM,), State.CONFIRMING),
        "pause": ((MSG_PAUSED,), State.PAUSED),
        "resume": ((RESUMED_ROOM,), State.NAVIGATING),
        "yes": ((START_TOILET,), State.NAVIGATING),
        "no": ((RESUMED_ROOM,), State.NAVIGATING),
        "talk": ((), State.CONFIRMING),
    },
    "NAVIGATING": {
        "navp": ((), State.CONFIRMING),
        "navc": ((ASK_ELEV,), State.CONFIRMING),
        "wait": ((MSG_PAUSED,), State.PAUSED),
        "finish": ((MSG_CANCEL_CONFIRM,), State.NAVIGATING),
        "cancel": ((MSG_CANCEL_CONFIRM,), State.NAVIGATING),
        "pause": ((MSG_PAUSED,), State.PAUSED),
        "resume": ((GOING_ROOM,), State.NAVIGATING),
        "yes": ((), State.NAVIGATING),
        "no": ((), State.NAVIGATING),
        "talk": ((), State.NAVIGATING),
    },
    "PAUSED": {
        "navp": ((), State.CONFIRMING),
        "navc": ((START_ELEV,), State.NAVIGATING),
        "wait": ((MSG_PAUSED,), State.PAUSED),
        "finish": ((MSG_CANCEL_CONFIRM,), State.PAUSED),
        "cancel": ((MSG_CANCEL_CONFIRM,), State.PAUSED),
        "pause": ((MSG_PAUSED,), State.PAUSED),
        "resume": ((RESUMED_ROOM,), State.NAVIGATING),
        "yes": ((), State.PAUSED),
        "no": ((), State.PAUSED),
        "talk": ((), State.PAUSED),
    },
    "FAILED": {
        "navp": ((), State.CONFIRMING),
        "navc": ((START_ELEV,), State.NAVIGATING),
        "wait": ((), State.FAILED),
        "finish": ((), State.FAILED),
        "cancel": ((MSG_NOT_NAVIGATING,), State.FAILED),
        "pause": ((MSG_NOT_NAVIGATING,), State.FAILED),
        "resume": ((MSG_NOT_PAUSED,), State.FAILED),
        "yes": ((), State.FAILED),
        "no": ((), State.FAILED),
        "talk": ((), State.FAILED),
    },
    "ARRIVED": {
        "navp": ((), State.CONFIRMING),
        "navc": ((START_ELEV,), State.NAVIGATING),
        "wait": ((), State.ARRIVED),
        "finish": ((), State.ARRIVED),
        "cancel": ((MSG_NOT_NAVIGATING,), State.ARRIVED),
        "pause": ((MSG_NOT_NAVIGATING,), State.ARRIVED),
        "resume": ((MSG_NOT_PAUSED,), State.ARRIVED),
        "yes": ((), State.ARRIVED),
        "no": ((), State.ARRIVED),
        "talk": ((), State.ARRIVED),
    },
    "ASKING_NEXT": {
        "navp": ((), State.CONFIRMING),
        "navc": ((START_ELEV,), State.NAVIGATING),
        "wait": ((MSG_WAIT_DEFAULT,), State.WAITING),
        "finish": ((MSG_FINISH,), State.RETURNING),
        "cancel": ((MSG_FINISH,), State.RETURNING),
        "pause": ((MSG_WAKE_GREETING,), State.ASKING_NEXT),
        "resume": ((MSG_WAIT_FINISH_ASK,), State.ASKING_NEXT),
        "yes": ((MSG_WAIT_DEFAULT,), State.WAITING),
        "no": ((MSG_ASK_ENTRANCE,), State.ASKING_NEXT),
        "talk": ((), State.ASKING_NEXT),
    },
    "ASKING_WAIT_TIME": {
        "navp": ((), State.CONFIRMING),
        "navc": ((START_ELEV,), State.NAVIGATING),
        "wait": ((MSG_ASK_WAIT_TIME,), State.ASKING_WAIT_TIME),
        "finish": ((MSG_FINISH,), State.RETURNING),
        "cancel": ((MSG_FINISH,), State.RETURNING),
        "pause": ((MSG_WAKE_GREETING,), State.ASKING_WAIT_TIME),
        "resume": ((MSG_WAIT_FINISH_ASK,), State.ASKING_NEXT),
        "yes": ((MSG_ASK_WAIT_TIME,), State.ASKING_WAIT_TIME),
        "no": ((MSG_ASK_ENTRANCE,), State.ASKING_NEXT),
        "talk": ((), State.ASKING_WAIT_TIME),
    },
    "WAITING_RELEASE": {
        "navp": ((), State.CONFIRMING),
        "navc": ((START_ELEV,), State.NAVIGATING),
        "wait": ((M2_RIGHT_10,), State.WAITING_RELEASE),
        "finish": ((MSG_WAIT_FINISH_ASK,), State.WAITING_RELEASE),
        "cancel": ((MSG_ASK_ENTRANCE,), State.ASKING_NEXT),
        "pause": ((MSG_WAKE_GREETING,), State.WAITING_RELEASE),
        "resume": ((MSG_WAIT_FINISH_ASK,), State.WAITING_RELEASE),
        "yes": ((), State.WAITING_RELEASE),
        "no": ((MSG_ASK_ENTRANCE,), State.ASKING_NEXT),
        "talk": ((), State.WAITING_RELEASE),
    },
    "MOVING_TO_WAIT_SPOT": {
        "navp": ((), State.MOVING_TO_WAIT_SPOT),
        "navc": ((), State.MOVING_TO_WAIT_SPOT),
        "wait": ((), State.MOVING_TO_WAIT_SPOT),
        "finish": ((), State.MOVING_TO_WAIT_SPOT),
        "cancel": ((MSG_NOT_NAVIGATING,), State.MOVING_TO_WAIT_SPOT),
        "pause": ((MSG_NOT_NAVIGATING,), State.MOVING_TO_WAIT_SPOT),
        "resume": ((MSG_NOT_PAUSED,), State.MOVING_TO_WAIT_SPOT),
        "yes": ((), State.MOVING_TO_WAIT_SPOT),
        "no": ((), State.MOVING_TO_WAIT_SPOT),
        "talk": ((), State.MOVING_TO_WAIT_SPOT),
    },
    "WAITING": {
        "navp": ((), State.CONFIRMING),
        "navc": ((START_ELEV,), State.NAVIGATING),
        "wait": ((M2_RIGHT_10,), State.WAITING),
        "finish": ((MSG_WAIT_FINISH_ASK,), State.WAITING),
        "cancel": ((NEED_ASK,), State.WAITING),
        "pause": ((MSG_NOT_NAVIGATING,), State.WAITING),
        "resume": ((MSG_WAIT_FINISH_ASK,), State.WAITING),
        "yes": ((), State.WAITING),
        "no": ((), State.WAITING),
        "talk": ((), State.WAITING),
    },
    "WAITING_ASKED": {
        "navp": ((), State.CONFIRMING),
        "navc": ((START_ELEV,), State.NAVIGATING),
        "wait": ((M2_RIGHT_10,), State.WAITING),
        "finish": ((MSG_FINISH,), State.RETURNING),
        "cancel": ((NEED_ASK,), State.WAITING),
        "pause": ((MSG_NOT_NAVIGATING,), State.WAITING),
        "resume": ((MSG_WAIT_FINISH_ASK,), State.WAITING),
        "yes": ((MSG_WAIT_FINISH_ASK,), State.WAITING),
        "no": ((MSG_FINISH,), State.RETURNING),
        "talk": ((), State.WAITING),
    },
    "MOVING_BACK_TO_DEST": {
        "navp": ((), State.MOVING_BACK_TO_DEST),
        "navc": ((), State.MOVING_BACK_TO_DEST),
        "wait": ((), State.MOVING_BACK_TO_DEST),
        "finish": ((), State.MOVING_BACK_TO_DEST),
        "cancel": ((MSG_NOT_NAVIGATING,), State.MOVING_BACK_TO_DEST),
        "pause": ((MSG_NOT_NAVIGATING,), State.MOVING_BACK_TO_DEST),
        "resume": ((MSG_NOT_PAUSED,), State.MOVING_BACK_TO_DEST),
        "yes": ((), State.MOVING_BACK_TO_DEST),
        "no": ((), State.MOVING_BACK_TO_DEST),
        "talk": ((), State.MOVING_BACK_TO_DEST),
    },
    "APPROACHING": {
        "navp": ((), State.APPROACHING),
        "navc": ((MSG_APPROACH_BUSY,), State.APPROACHING),
        "wait": ((), State.APPROACHING),
        "finish": ((), State.APPROACHING),
        "cancel": ((MSG_NOT_NAVIGATING,), State.APPROACHING),
        "pause": ((MSG_NOT_NAVIGATING,), State.APPROACHING),
        "resume": ((MSG_NOT_PAUSED,), State.APPROACHING),
        "yes": ((), State.APPROACHING),
        "no": ((), State.APPROACHING),
        "talk": ((), State.APPROACHING),
    },
    "AWAITING_USER": {
        "navp": ((MSG_APPROACH_ACCEPTED,), State.TURNING),
        "navc": ((MSG_APPROACH_ACCEPTED,), State.TURNING),
        "wait": ((), State.AWAITING_USER),
        "finish": ((), State.AWAITING_USER),
        "cancel": ((MSG_APPROACH_DECLINED,), State.RETURNING),
        "pause": ((MSG_WAKE_GREETING,), State.AWAITING_USER),
        "resume": ((MSG_WAKE_GREETING,), State.AWAITING_USER),
        "yes": ((MSG_APPROACH_ACCEPTED,), State.TURNING),
        "no": ((MSG_APPROACH_DECLINED,), State.RETURNING),
        "talk": ((), State.AWAITING_USER),
    },
    "TURNING": {
        "navp": ((), State.TURNING),
        "navc": ((), State.TURNING),
        "wait": ((), State.TURNING),
        "finish": ((), State.TURNING),
        "cancel": ((MSG_NOT_NAVIGATING,), State.TURNING),
        "pause": ((MSG_NOT_NAVIGATING,), State.TURNING),
        "resume": ((MSG_NOT_PAUSED,), State.TURNING),
        "yes": ((), State.TURNING),
        "no": ((), State.TURNING),
        "talk": ((), State.TURNING),
    },
    "SEEKING": {
        "navp": ((), State.SEEKING),
        "navc": ((MSG_APPROACH_BUSY,), State.SEEKING),
        "wait": ((), State.SEEKING),
        "finish": ((), State.SEEKING),
        "cancel": ((MSG_NOT_NAVIGATING,), State.SEEKING),
        "pause": ((MSG_NOT_NAVIGATING,), State.SEEKING),
        "resume": ((MSG_NOT_PAUSED,), State.SEEKING),
        "yes": ((), State.SEEKING),
        "no": ((), State.SEEKING),
        "talk": ((), State.SEEKING),
    },
    "RETURNING": {
        "navp": ((), State.CONFIRMING),
        "navc": ((START_ELEV,), State.NAVIGATING),
        "wait": ((M2P_FRONT,), State.MOVING_BACK_TO_DEST),
        "finish": ((MSG_FINISH,), State.RETURNING),
        "cancel": ((MSG_NOT_NAVIGATING,), State.RETURNING),
        "pause": ((MSG_WAKE_GREETING,), State.IDLE),
        "resume": ((MSG_NOT_PAUSED,), State.RETURNING),
        "yes": ((), State.RETURNING),
        "no": ((), State.RETURNING),
        "talk": ((), State.RETURNING),
    },
    "RETURNING_LATE": {
        "navp": ((), State.CONFIRMING),
        "navc": ((START_ELEV,), State.NAVIGATING),
        "wait": ((M2P_FRONT,), State.MOVING_BACK_TO_DEST),
        "finish": ((MSG_FINISH,), State.RETURNING),
        "cancel": ((MSG_NOT_NAVIGATING,), State.RETURNING),
        "pause": ((MSG_WAKE_GREETING,), State.IDLE),
        "resume": ((MSG_NOT_PAUSED,), State.RETURNING),
        "yes": ((M2P_FRONT,), State.MOVING_BACK_TO_DEST),
        "no": ((MSG_FINISH,), State.RETURNING),
        "talk": ((), State.RETURNING),
    },
    "ESTOPPED": {
        "navp": ((MSG_ESTOP_REJECT,), State.ESTOPPED),
        "navc": ((MSG_ESTOP_REJECT,), State.ESTOPPED),
        "wait": ((), State.ESTOPPED),
        "finish": ((), State.ESTOPPED),
        "cancel": ((MSG_ESTOP_REJECT,), State.ESTOPPED),
        "pause": ((MSG_ESTOP_REJECT,), State.ESTOPPED),
        "resume": ((MSG_ESTOP_REJECT,), State.ESTOPPED),
        "yes": ((), State.ESTOPPED),
        "no": ((), State.ESTOPPED),
        "talk": ((), State.ESTOPPED),
    },
}

PENDING = {
    # Task 5
    "CONFIRMING.finish": 5,
    "CONFIRMING_CHANGE.finish": 5,
    "IDLE.finish": 5,
    "NAVIGATING.finish": 5,
    "PAUSED.finish": 5,
    # Task 6
    "ASKING_NEXT.resume": 6,
    "ASKING_WAIT_TIME.resume": 6,
    "AWAITING_USER.resume": 6,
    "CONFIRMING.resume": 6,
    "NAVIGATING.resume": 6,
    "WAITING.resume": 6,
    "WAITING_ASKED.resume": 6,
    "WAITING_RELEASE.resume": 6,
    # Task 7
    "ASKING_WAIT_TIME.no": 7,
    "AWAITING_USER.cancel": 7,
    "CONFIRMING.cancel": 7,
    "WAITING_ASKED.no": 7,
    "WAITING_RELEASE.cancel": 7,
    "WAITING_RELEASE.no": 7,
    # Task 8
    "WAITING.cancel": 8,
    "WAITING_ASKED.cancel": 8,
    # Task 9
    "IDLE_BRAKED.finish": 9,
    "IDLE_BRAKED.resume": 9,
    "IDLE_BRAKED.wait": 9,
    "IDLE_BRAKED_SPOT.finish": 9,
    "IDLE_BRAKED_SPOT.resume": 9,
    "IDLE_BRAKED_SPOT.wait": 9,
    "RETURNING.finish": 9,
    "RETURNING.wait": 9,
    "RETURNING_LATE.finish": 9,
    "RETURNING_LATE.no": 9,
    "RETURNING_LATE.wait": 9,
    "RETURNING_LATE.yes": 9,
    # Task 10
    "ASKING_WAIT_TIME.yes": 10,
    "CONFIRMING.navc": 10,
    "CONFIRMING_CHANGE.navc": 10,
    # Task 11
    "AWAITING_USER.navc": 11,
    "AWAITING_USER.navp": 11,
    "TURNING.navc": 11,
    # Task 12
    "WAITING_ASKED.yes": 12,
}


def _cases():
    for row, cells in EXPECT.items():
        for col in COLS:
            key = f"{row}.{col}"
            marks = ()
            if key in PENDING:
                marks = (pytest.mark.xfail(strict=True, reason=f"Task {PENDING[key]}"),)
            yield pytest.param(row, col, id=key, marks=marks)


def test_table_covers_every_state_and_request():
    """새 상태·새 요청이 생기면 표에서 빠지지 않게."""
    reached = set()
    for row, build in ROWS.items():
        logic, _ = build()
        reached.add(logic.state)
        assert set(EXPECT[row]) == set(COLS), row
    assert reached == set(State)
    assert set(EXPECT) == set(ROWS)
    assert set(REQUESTS) == set(COLS)
    assert set(PENDING) <= {f"{r}.{c}" for r in EXPECT for c in COLS}


@pytest.mark.parametrize("row,col", list(_cases()))
def test_reaction(row, col):
    logic, now = ROWS[row]()
    actions = logic.on_voice_intent(REQUESTS[col], now, lookup, BOUNDS, True)
    says = tuple(a.text for a in actions if isinstance(a, Say))
    expected_says, expected_state = EXPECT[row][col]
    assert (says, logic.state) == (expected_says, expected_state)
