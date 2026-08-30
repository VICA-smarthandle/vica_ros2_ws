"""도착 후 대화 (arrival-dialog-flow, 2026-08-30).

도착 → 유형별 질문 → 대기(wait)/종료(finish)/다음(navigate)/무응답 갈래.
말을 알아듣는 일은 음성 몫이고, 이 모듈은 정리된 intent 만 받는다.
문구 정본은 mission_logic 상수(캐시가 구운 판을 재생하므로 글자 일치가 계약).
"""
import pytest

from vica_mission_manager.mission_logic import (
    Destination, IntentData, MapBounds, MissionLogic, Navigate, NavStatus,
    Pose2D, Say, State,
    MSG_ASK_RESTROOM, MSG_ASK_ENTRANCE, MSG_ASK_GENERIC, MSG_ASK_WAIT_TIME,
    MSG_WAIT_DEFAULT, MSG_FINISH, MSG_GOING_HOME, MSG_LEAVING_NOTICE,
    MSG_ARRIVAL_RETRY, MSG_WAIT_RESUME_ASK,
)

BOUNDS = MapBounds(min_x=-50, min_y=-50, max_x=50, max_y=50)
HOME = Destination(id="__home__", name="홈", pose=Pose2D(x=0, y=0, yaw_deg=0, frame_id="map"))


def _dest(category="", **kw):
    d = dict(id="d1", name="화장실", pose=Pose2D(x=3, y=2, yaw_deg=90, frame_id="map"),
             calibrated=True, arrival_message="화장실 앞에 도착했습니다.",
             category=category)
    d.update(kw)
    return Destination(**d)


def _intent(intent="navigate", **kw):
    d = dict(intent=intent, matched_destination_id="d1", need_confirm=False,
             safety_flag="normal")
    d.update(kw)
    return IntentData(**d)


def _say(actions):
    return [a.text for a in actions if isinstance(a, Say)]


def arrive(category="restroom", home=HOME):
    """주행 → 도착 → 질문 재생완료까지. ASKING_NEXT 상태의 logic 을 준다."""
    logic = MissionLogic(return_destination=home, arrival_dialog=True)
    logic.on_intent(_intent(), _dest(category), BOUNDS, True, 0.0)
    acts = logic.on_tick(1.0, NavStatus.SUCCEEDED)
    assert logic.state == State.ASKING_NEXT, _say(acts)
    logic.on_arrival_question_spoken(2.0)     # 재생완료 → 8초 시작
    return logic


class TestTypeQuestion:
    def test_restroom_asks_wait(self):
        logic = MissionLogic(arrival_dialog=True)
        logic.on_intent(_intent(), _dest("restroom"), BOUNDS, True, 0.0)
        acts = logic.on_tick(1.0, NavStatus.SUCCEEDED)
        assert MSG_ASK_RESTROOM in _say(acts)

    def test_entrance_asks_finish(self):
        logic = MissionLogic(arrival_dialog=True)
        logic.on_intent(_intent(), _dest("entrance"), BOUNDS, True, 0.0)
        assert MSG_ASK_ENTRANCE in _say(logic.on_tick(1.0, NavStatus.SUCCEEDED))

    def test_generic_asks_wait_or_end(self):
        logic = MissionLogic(arrival_dialog=True)
        logic.on_intent(_intent(), _dest("reception"), BOUNDS, True, 0.0)
        assert MSG_ASK_GENERIC in _say(logic.on_tick(1.0, NavStatus.SUCCEEDED))

    def test_dialog_off_keeps_legacy_dwell(self):
        """arrival_dialog=False 면 기존대로 도착 후 dwell → idle."""
        logic = MissionLogic(arrival_dialog=False)
        logic.on_intent(_intent(), _dest("restroom"), BOUNDS, True, 0.0)
        acts = logic.on_tick(1.0, NavStatus.SUCCEEDED)
        assert logic.state == State.ARRIVED
        assert MSG_ASK_RESTROOM not in _say(acts)


class TestWaitAnswer:
    def test_restroom_affirm_waits_30_no_time_question(self):
        """restroom 은 '네'면 시간 안 묻고 최대 30분 대기."""
        logic = arrive("restroom")
        acts = logic.on_arrival_answer(_intent("affirm"), 3.0)
        assert logic.state == State.WAITING
        assert MSG_WAIT_DEFAULT in _say(acts)

    def test_wait_with_time_confirms_and_waits(self):
        logic = arrive("restroom")
        acts = logic.on_arrival_answer(_intent("wait", wait_minutes=20), 3.0)
        assert logic.state == State.WAITING
        assert any("20분" in t for t in _say(acts))

    def test_wait_capped_at_30(self):
        logic = arrive("restroom")
        acts = logic.on_arrival_answer(_intent("wait", wait_minutes=99), 3.0)
        assert any("30분" in t for t in _say(acts))

    def test_generic_wait_without_time_asks_how_long(self):
        """그 외 유형에서 대기(wait, 시간없음)면 '몇 분쯤?' 후속 질문."""
        logic = arrive("reception")
        acts = logic.on_arrival_answer(_intent("wait", wait_minutes=-1), 3.0)
        assert logic.state == State.ASKING_WAIT_TIME
        assert MSG_ASK_WAIT_TIME in _say(acts)
        logic.on_arrival_question_spoken(4.0)
        acts2 = logic.on_arrival_answer(_intent("wait", wait_minutes=10), 5.0)
        assert logic.state == State.WAITING
        assert any("10분" in t for t in _say(acts2))


class TestFinishAndNext:
    def test_finish_goes_home(self):
        logic = arrive("restroom")
        acts = logic.on_arrival_answer(_intent("finish"), 3.0)
        assert MSG_FINISH in _say(acts)
        assert logic.state == State.RETURNING
        assert any(isinstance(a, Navigate) for a in acts)

    def test_entrance_affirm_is_finish(self):
        """entrance 는 종료형 — '네'면 홈으로."""
        logic = arrive("entrance")
        acts = logic.on_arrival_answer(_intent("affirm"), 3.0)
        assert logic.state == State.RETURNING
        assert MSG_FINISH in _say(acts)

    def test_cancel_after_arrival_goes_home(self):
        """도착 후 cancel = finish 와 동일(홈 복귀). 2026-08-30 사용자 결정."""
        logic = arrive("restroom")
        acts = logic.on_arrival_answer(_intent("cancel"), 3.0)
        assert logic.state == State.RETURNING

    def test_next_destination_navigates(self):
        logic = arrive("restroom")
        nxt = _dest("reception", id="d2", name="안내소")
        acts = logic.on_arrival_answer(
            _intent("navigate", matched_destination_id="d2"), 3.0, next_dest=nxt)
        assert logic.state == State.NAVIGATING


class TestNoAnswerLadder:
    def test_unknown_retries_once_then_leaves(self):
        logic = arrive("restroom")
        acts = logic.on_arrival_answer(_intent("unknown"), 3.0)
        assert MSG_ARRIVAL_RETRY in _say(acts)
        assert logic.state == State.ASKING_NEXT       # 아직 안 떠남
        logic.on_arrival_question_spoken(4.0)
        acts2 = logic.on_arrival_answer(_intent("unknown"), 5.0)
        assert MSG_LEAVING_NOTICE in _say(acts2)      # 두 번째 실패 → 예고

    def test_silence_8s_then_leaving_notice(self):
        logic = arrive("restroom")   # 질문 재생완료 2.0 → 8초 데드라인 10.0
        assert _say(logic.on_tick(9.0, NavStatus.NONE)) == []
        acts = logic.on_tick(10.5, NavStatus.NONE)
        assert MSG_LEAVING_NOTICE in _say(acts)

    def test_leaving_grace_then_goes_home(self):
        logic = arrive("restroom")
        logic.on_tick(10.5, NavStatus.NONE)           # 예고 (유예 3초 시작)
        acts = logic.on_tick(14.0, NavStatus.NONE)    # 유예 경과
        assert MSG_GOING_HOME in _say(acts)
        assert logic.state == State.RETURNING

    def test_grace_interrupt_returns_to_dialog(self):
        logic = arrive("restroom")
        logic.on_tick(10.5, NavStatus.NONE)           # 예고
        acts = logic.on_arrival_answer(_intent("wait", wait_minutes=10), 11.0)
        assert logic.state == State.WAITING           # 끼어들면 산다


class TestWaitingState:
    def test_wake_asks_resume(self):
        logic = arrive("restroom")
        logic.on_arrival_answer(_intent("affirm"), 3.0)   # WAITING
        acts = logic.on_wake(20.0)
        assert MSG_WAIT_RESUME_ASK in _say(acts)
        assert logic.state == State.ASKING_NEXT

    def test_wait_timeout_leaves(self):
        logic = arrive("restroom")
        logic.on_arrival_answer(_intent("wait", wait_minutes=1), 3.0)  # 1분
        acts = logic.on_tick(3.0 + 61.0, NavStatus.NONE)
        assert MSG_GOING_HOME in _say(acts)
        assert logic.state == State.RETURNING
