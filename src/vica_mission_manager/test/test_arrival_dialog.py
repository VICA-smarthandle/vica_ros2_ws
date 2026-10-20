"""도착 후 대화 (arrival-dialog-flow, 2026-08-30)."""
import pytest

from vica_mission_manager.mission_logic import (
    Destination, IntentData, MapBounds, MissionLogic, Navigate, NavStatus,
    Pose2D, Say, State,
    MSG_ASK_RESTROOM, MSG_ASK_ENTRANCE, MSG_ASK_GENERIC, MSG_ASK_WAIT_TIME,
    MSG_WAIT_DEFAULT, MSG_FINISH, MSG_LEAVING_NOTICE,
    MSG_ARRIVAL_RETRY, MSG_WAIT_EXPIRED, GoalEvent, MSG_WAIT_FINISH_ASK,
    WAIT_FINISH_REPEAT_SEC, Haptic, HAPTIC_PATTERN_WAKE_LOCATE,
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
    logic.on_arrival_question_spoken(2.0)
    return logic


class TestTypeQuestion:
    def test_restroom_asks_wait(self):
        logic = MissionLogic(arrival_dialog=True)
        logic.on_intent(_intent(), _dest("restroom"), BOUNDS, True, 0.0)
        acts = logic.on_tick(1.0, NavStatus.SUCCEEDED)
        assert any(MSG_ASK_RESTROOM in t for t in _say(acts))
        assert any("도착했습니다" in t for t in _say(acts))

    def test_entrance_asks_finish(self):
        logic = MissionLogic(arrival_dialog=True)
        logic.on_intent(_intent(), _dest("entrance"), BOUNDS, True, 0.0)
        assert any(MSG_ASK_ENTRANCE in t for t in _say(logic.on_tick(1.0, NavStatus.SUCCEEDED)))

    def test_generic_asks_wait_or_end(self):
        logic = MissionLogic(arrival_dialog=True)
        logic.on_intent(_intent(), _dest("reception"), BOUNDS, True, 0.0)
        assert any(MSG_ASK_GENERIC in t for t in _say(logic.on_tick(1.0, NavStatus.SUCCEEDED)))

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
        assert _say(acts) == ["안내소로 안내를 시작합니다."]
        kinds = [type(a).__name__ for a in acts]
        assert kinds.index("Say") < kinds.index("Navigate")


class TestNoAnswerLadder:
    def test_unknown_retries_once_then_leaves(self):
        """못 알아들은 답에는 질문 자체를 한 번 더 묻는다(2026-10-08 다시 묻기 — 옛 문장은"""
        logic = arrive("restroom")
        acts = logic.on_arrival_answer(_intent("unknown"), 3.0)
        assert _say(acts) == [MSG_ASK_RESTROOM]
        assert logic.state == State.ASKING_NEXT
        logic.on_arrival_question_spoken(4.0)
        acts2 = logic.on_arrival_answer(_intent("unknown"), 5.0)
        assert MSG_LEAVING_NOTICE in _say(acts2)

    def test_silence_8s_reasks_same_question_then_leaving_notice(self):
        logic = arrive("restroom")
        assert _say(logic.on_tick(9.0, NavStatus.NONE)) == []
        acts = logic.on_tick(10.5, NavStatus.NONE)
        assert _say(acts) == [MSG_ASK_RESTROOM] and logic.state == State.ASKING_NEXT
        logic.on_arrival_question_spoken(11.0)
        assert _say(logic.on_tick(18.0, NavStatus.NONE)) == []
        assert MSG_LEAVING_NOTICE in _say(logic.on_tick(19.5, NavStatus.NONE))

    def test_reask_answered_wait_survives(self):
        logic = arrive("restroom")
        logic.on_tick(10.5, NavStatus.NONE)
        acts = logic.on_arrival_answer(_intent("affirm"), 12.0)
        assert logic.state == State.WAITING and MSG_WAIT_DEFAULT in _say(acts)

    def _leaving(self, logic):
        logic.on_tick(10.5, NavStatus.NONE)
        logic.on_arrival_question_spoken(11.0)
        logic.on_tick(19.5, NavStatus.NONE)

    def test_leaving_grace_then_goes_home(self):
        logic = arrive("restroom")
        self._leaving(logic)
        acts = logic.on_tick(23.0, NavStatus.NONE)
        assert not _say(acts)
        assert logic.state == State.RETURNING

    def test_grace_interrupt_returns_to_dialog(self):
        logic = arrive("restroom")
        self._leaving(logic)
        acts = logic.on_arrival_answer(_intent("wait", wait_minutes=10), 20.0)
        assert logic.state == State.WAITING

    def test_unknown_then_silence_leaves_without_second_reask(self):
        logic = arrive("restroom")
        logic.on_arrival_answer(_intent("unknown"), 3.0)
        logic.on_arrival_question_spoken(4.0)
        assert MSG_LEAVING_NOTICE in _say(logic.on_tick(12.5, NavStatus.NONE))


class TestDenyReconfirm:
    """대기형 질문의 '아니오'는 종료형으로 한 번 더 묻는다 (2026-09-20 사용자 결정)."""

    def test_deny_on_wait_question_asks_finish_question(self):
        logic = arrive("restroom")
        acts = logic.on_arrival_answer(_intent("deny"), 3.0)
        assert _say(acts) == [MSG_ASK_ENTRANCE] and logic.state == State.ASKING_NEXT

    def test_second_deny_means_wait(self):
        logic = arrive("restroom")
        logic.on_arrival_answer(_intent("deny"), 3.0)
        acts = logic.on_arrival_answer(_intent("deny"), 5.0)
        assert logic.state == State.WAITING and MSG_WAIT_DEFAULT in _say(acts)

    def test_affirm_after_reconfirm_finishes(self):
        logic = arrive("restroom")
        logic.on_arrival_answer(_intent("deny"), 3.0)
        acts = logic.on_arrival_answer(_intent("affirm"), 5.0)
        assert MSG_FINISH in _say(acts) and logic.state == State.RETURNING

    def test_generic_question_deny_also_reconfirms(self):
        logic = arrive("")
        acts = logic.on_arrival_answer(_intent("deny"), 3.0)
        assert _say(acts) == [MSG_ASK_ENTRANCE]

    def test_entrance_deny_still_waits_without_reconfirm(self):
        logic = arrive("entrance")
        acts = logic.on_arrival_answer(_intent("deny"), 3.0)
        assert logic.state == State.WAITING and MSG_WAIT_DEFAULT in _say(acts)


class TestWakeKeepsArrivalQuestion:
    """"비카야"는 도착 질문을 접지 않는다(2026-10-07 사용자 결정, 호출 반응표)."""

    def test_wake_keeps_question_and_wait_answer_still_works(self):
        logic = arrive("restroom")
        assert logic.on_wake(3.0) == []
        assert logic.state == State.ASKING_NEXT
        acts = logic.on_arrival_answer(_intent("wait", wait_minutes=5), 4.0)
        assert logic.state == State.WAITING
        assert any("5분" in t for t in _say(acts))

    def test_wake_keeps_wait_time_question(self):
        logic = arrive("reception")
        logic.on_arrival_answer(_intent("affirm"), 3.0)
        assert logic.state == State.ASKING_WAIT_TIME
        logic.on_wake(4.0)
        assert logic.state == State.ASKING_WAIT_TIME
        logic.on_arrival_answer(_intent("wait", wait_minutes=10), 5.0)
        assert logic.state == State.WAITING

    def test_wake_withdraws_leaving_notice(self):
        """떠나기 예고 유예 중 불렀으면 떠나지 않는다 — 사용자가 곁에 있다."""
        logic = arrive("restroom")
        logic.on_arrival_answer(_intent("unknown"), 3.0)
        acts = logic.on_arrival_answer(_intent("unknown"), 5.0)
        assert MSG_LEAVING_NOTICE in _say(acts)
        logic.on_wake(6.0)
        assert logic.state == State.ASKING_NEXT
        assert not any(isinstance(a, Navigate) for a in logic.on_tick(20.0, NavStatus.NONE))
        assert logic.state == State.ASKING_NEXT
        logic.on_arrival_question_spoken(21.0)
        acts = logic.on_tick(30.0, NavStatus.NONE)
        assert logic.state == State.ASKING_NEXT
        assert any(MSG_ASK_RESTROOM in t for t in _say(acts))


class TestWaitingState:
    def test_wake_keeps_waiting(self):
        """WAITING 중 "비카야" → 대기를 이어 가며 새 대화(2026-10-07 호출 반응표)."""
        logic = arrive("restroom")
        logic.on_arrival_answer(_intent("affirm"), 3.0)
        left = logic.wait_left_sec(20.0)
        assert logic.on_wake(20.0) == [Haptic(HAPTIC_PATTERN_WAKE_LOCATE)]
        assert logic.state == State.WAITING
        assert logic.wait_left_sec(20.0) == left

    def test_wait_timeout_leaves(self):
        logic = arrive("restroom")
        logic.on_arrival_answer(_intent("wait", wait_minutes=1), 3.0)
        acts = logic.on_tick(3.0 + 61.0, NavStatus.NONE)
        assert _say(acts) == [MSG_WAIT_EXPIRED]
        assert [a.event for a in acts if isinstance(a, GoalEvent)] == ["wait_expired"]
        assert logic.state == State.RETURNING


class TestReturnBrake:
    def test_wake_during_return_cancels_quietly(self):
        """홈 복귀 중 '비카야' → 복귀 취소, 질문 없이 새 대화(9/1 A안)."""
        from vica_mission_manager.mission_logic import CancelNav
        logic = arrive("restroom")
        logic.on_arrival_answer(_intent("finish"), 3.0)
        assert logic.state == State.RETURNING
        acts = logic.on_return_brake(5.0)
        assert logic.state == State.IDLE
        assert not _say(acts)
        assert any(isinstance(a, CancelNav) for a in acts)

    def test_return_brake_ignored_when_not_returning(self):
        logic = MissionLogic()
        assert logic.on_return_brake(1.0) == []


class TestLoadHome:
    def test_home_yaml_becomes_return_destination(self, tmp_path):
        from vica_mission_manager.destinations import load_home
        (tmp_path / "home.yaml").write_text(
            "pose:\n  frame_id: map\n  x: 1.5\n  y: -0.5\n  yaw: 90.0\n"
            'label: "입구"\n')
        (tmp_path / "destinations.yaml").write_text("destinations: []\n")
        home = load_home(str(tmp_path / "destinations.yaml"))
        assert home is not None and home.id == "__home__"
        assert home.pose.x == 1.5

    def test_no_home_yaml_is_none(self, tmp_path):
        from vica_mission_manager.destinations import load_home
        (tmp_path / "destinations.yaml").write_text("destinations: []\n")
        assert load_home(str(tmp_path / "destinations.yaml")) is None


class TestArrivalNavigateConfirm:
    """도착 후 대화 중 새 목적지 '제안'은 즉시 출발하면 안 된다 (2026-08-30 실기)."""

    def test_proposal_joins_confirm_flow_not_navigate(self):
        logic = arrive("restroom")
        logic.on_arrival_answer(_intent("affirm"), 3.0)
        logic.on_wake(10.0)
        assert logic.state == State.WAITING
        nxt = _dest("", id="d2", name="입구")
        acts = logic.on_intent(_intent("navigate", matched_destination_id="d2",
                                       need_confirm=True), nxt, BOUNDS, True, 11.0)
        assert logic.state == State.CONFIRMING
        assert not any(isinstance(a, Navigate) for a in acts)
        acts2 = logic.on_intent(_intent("navigate", matched_destination_id="d2"),
                                nxt, BOUNDS, True, 12.0)
        assert logic.state == State.NAVIGATING
        assert any(isinstance(a, Navigate) for a in acts2)

    def test_confirmed_navigate_still_departs_immediately(self):
        """확정(need_confirm=False)으로 온 navigate 는 기존대로 즉시 출발."""
        logic = arrive("restroom")
        nxt = _dest("", id="d2", name="안내소")
        acts = logic.on_arrival_answer(
            _intent("navigate", matched_destination_id="d2"), 3.0, next_dest=nxt)
        assert logic.state == State.NAVIGATING
        assert any(isinstance(a, Navigate) for a in acts)

    def test_exit_is_noop_outside_dialog(self):
        logic = MissionLogic()
        logic.exit_arrival_dialog()
        assert logic.state == State.IDLE

    def test_exit_during_open_seek_window_clears_it(self):
        """exit_arrival_dialog 는 IDLE 로 가는 길 중 _to_idle() 을 안 거치는"""
        logic = arrive("restroom")
        logic._seek_deadline = 999.0
        logic._seek_return_yaw = 1.23
        logic.exit_arrival_dialog()
        assert logic._seek_deadline is None
        assert logic._seek_return_yaw is None


class TestEarHold:
    """무응답 시계는 귀가 바쁜 동안 멈춘다 (2026-08-30 실기 — 답이 STT·LLM"""

    def test_open_listen_holds_silence_deadline(self):
        logic = arrive("restroom")
        logic.on_listen_state("open", 3.0)
        assert _say(logic.on_tick(10.5, NavStatus.NONE)) == []
        logic.on_listen_state("speech", 11.0)
        assert _say(logic.on_tick(12.0, NavStatus.NONE)) == []

    def test_closed_grants_grace_for_llm(self):
        """전사 성공(closed) 후에도 LLM 처리 시간(8초, 2026-10-10 6 -> 8)을 기다린다."""
        logic = arrive("restroom")
        logic.on_listen_state("open", 3.0)
        logic.on_listen_state("speech", 7.0)
        logic.on_listen_state("closed", 9.5)
        assert _say(logic.on_tick(10.5, NavStatus.NONE)) == []
        assert _say(logic.on_tick(17.0, NavStatus.NONE)) == []
        acts = logic.on_tick(18.0, NavStatus.NONE)
        assert _say(acts) == [MSG_ASK_RESTROOM]

    def test_llm_thinking_holds_the_arrival_clock(self):
        """LLM 이 생각 중(/vica/thinking)이면 도착 질문도 다시 묻지 않는다(2026-10-10)."""
        logic = arrive("restroom")
        logic.on_listen_state("open", 3.0)
        logic.on_listen_state("closed", 9.5)
        logic.on_llm_thinking(True, 8.0)
        assert _say(logic.on_tick(19.0, NavStatus.NONE)) == []
        logic.on_llm_thinking(False, 19.5)
        acts = logic.on_tick(21.0, NavStatus.NONE)
        assert _say(acts) == [MSG_ASK_RESTROOM]

    def test_empty_fires_promptly(self):
        logic = arrive("restroom")
        logic.on_listen_state("open", 3.0)
        logic.on_listen_state("empty", 8.5)
        acts = logic.on_tick(10.5, NavStatus.NONE)
        assert _say(acts) == [MSG_ASK_RESTROOM]

    def test_stuck_open_ear_has_failsafe_cap(self):
        """닫힘 신호를 영영 못 받아도 20초 상한 뒤엔 떠난다 (무한 대기 방지)."""
        logic = arrive("restroom")
        logic.on_listen_state("open", 3.0)
        assert _say(logic.on_tick(15.0, NavStatus.NONE)) == []
        acts = logic.on_tick(24.0, NavStatus.NONE)
        assert _say(acts) == [MSG_ASK_RESTROOM]


class TestReturnNet:
    """복귀 중 늦게 도착한 답의 마지막 그물 (2026-08-30)."""

    def test_quiet_brake_then_wait_answer(self):
        from vica_mission_manager.mission_logic import CancelNav
        logic = arrive("restroom")
        logic.on_arrival_answer(_intent("finish"), 3.0)
        acts = logic.on_return_brake(5.0, quiet=True)
        assert logic.state == State.ASKING_NEXT
        assert _say(acts) == []
        assert any(isinstance(a, CancelNav) for a in acts)
        acts2 = logic.on_arrival_answer(_intent("wait", wait_minutes=10), 5.5)
        assert logic.state == State.WAITING
        assert any("10분" in t for t in _say(acts2))


class TestAppCancelInWaiting:
    """관리자 회수 (2026-08-31): WAITING(최대 30분)은 앱 취소로 풀 수 있어야"""

    def test_cancel_releases_waiting(self):
        from vica_mission_manager.mission_logic import GateReason
        logic = arrive("restroom")
        logic.on_arrival_answer(_intent("wait", wait_minutes=30), 3.0)
        assert logic.state == State.WAITING
        actions, reason = logic.on_cancel_request(10.0)
        assert reason == GateReason.OK
        assert logic.state == State.IDLE
        assert logic._wait_until is None
        assert any("취소" in t for t in _say(actions))

    def test_cancel_still_rejected_in_asking(self):
        from vica_mission_manager.mission_logic import GateReason
        logic = arrive("restroom")
        actions, reason = logic.on_cancel_request(3.0)
        assert reason != GateReason.OK


class TestGenericIsWaitStyle:
    '''그 외 유형 질문 개편 (2026-08-31 사용자 결정): "여기서 대기할까요?"'''

    def test_generic_affirm_asks_time(self):
        logic = arrive("reception")
        acts = logic.on_arrival_answer(_intent("affirm"), 3.0)
        assert logic.state == State.ASKING_WAIT_TIME
        assert MSG_ASK_WAIT_TIME in _say(acts)

    def test_generic_deny_reconfirms_then_finishes(self):
        logic = arrive("reception")
        acts = logic.on_arrival_answer(_intent("deny"), 3.0)
        assert logic.state == State.ASKING_NEXT and _say(acts) == [MSG_ASK_ENTRANCE]
        acts = logic.on_arrival_answer(_intent("affirm"), 5.0)
        assert logic.state == State.RETURNING
        assert MSG_FINISH in _say(acts)

    def test_restroom_affirm_still_skips_time(self):
        logic = arrive("restroom")
        logic.on_arrival_answer(_intent("affirm"), 3.0)
        assert logic.state == State.WAITING

    def test_entrance_wait_skips_time_question(self):
        """entrance 에서 "기다려줘"(시간 없음) — 스펙대로 시간 안 묻고 30분."""
        logic = arrive("entrance")
        acts = logic.on_arrival_answer(_intent("wait", wait_minutes=-1), 3.0)
        assert logic.state == State.WAITING
        assert any("30분" in t for t in _say(acts))


class TestAskWaitTimeRejectsYesNo:
    """시간 질문에 네/아니오는 답이 아니다 (구멍 ②: 옛 질문 태그가 남아"""

    def _to_wait_time(self):
        logic = arrive("reception")
        logic.on_arrival_answer(_intent("affirm"), 3.0)
        logic.on_arrival_question_spoken(4.0)
        return logic

    def test_affirm_reasks(self):
        logic = self._to_wait_time()
        acts = logic.on_arrival_answer(_intent("affirm"), 5.0)
        assert logic.state == State.ASKING_WAIT_TIME
        assert _say(acts) == [MSG_ASK_WAIT_TIME]

    def test_deny_reasks(self):
        """이 경로(on_arrival_answer 직접)는 옛 그물이다 — 음성 요청은 on_voice_intent 가"""
        logic = self._to_wait_time()
        acts = logic.on_arrival_answer(_intent("deny"), 5.0)
        assert logic.state == State.ASKING_WAIT_TIME
        assert _say(acts) == [MSG_ASK_WAIT_TIME]


class TestAskingStuckFallback:
    """구멍 ①: 질문 재생이 끊기면 tts_done 이 없어 시계가 영영 시작 안 됐다"""

    def test_lost_tts_done_still_leaves(self):
        logic = MissionLogic(return_destination=HOME, arrival_dialog=True)
        logic.on_intent(_intent(), _dest("restroom"), BOUNDS, True, 0.0)
        logic.on_tick(1.0, NavStatus.SUCCEEDED)
        assert _say(logic.on_tick(20.0, NavStatus.NONE)) == []
        acts = logic.on_tick(32.0, NavStatus.NONE)
        assert any("돌아가겠습니다" in t for t in _say(acts))


def test_wake_resume_does_not_trip_stuck_fallback():
    """WAITING 각성 직후 낡은 진입 시각으로 폴백이 즉발하면 안 된다."""
    logic = arrive("restroom")
    logic.on_arrival_answer(_intent("affirm"), 3.0)
    logic.on_wake(600.0)
    acts = logic.on_tick(601.0, NavStatus.NONE)
    assert not any("돌아가겠습니다" in t for t in _say(acts))


from vica_mission_manager.mission_logic import CancelNav, GateReason


def _builders():
    """선점당할 활성 상태를 만드는 빌더들. (상태 이름 -> logic)"""
    def navigating():
        logic = MissionLogic(return_destination=HOME, arrival_dialog=True)
        logic.on_intent(_intent(), _dest("restroom"), BOUNDS, True, 0.0)
        return logic
    def confirming():
        logic = MissionLogic(arrival_dialog=True)
        logic.on_intent(_intent(need_confirm=True), _dest("restroom"), BOUNDS, True, 0.0)
        return logic
    def paused():
        logic = navigating()
        logic.on_pause_request(1.0)
        return logic
    def asking():
        return arrive("restroom")
    def asking_time():
        logic = arrive("reception")
        logic.on_arrival_answer(_intent("affirm"), 3.0)
        return logic
    def waiting():
        logic = arrive("restroom")
        logic.on_arrival_answer(_intent("affirm"), 3.0)
        return logic
    def returning():
        logic = arrive("restroom")
        logic.on_arrival_answer(_intent("finish"), 3.0)
        return logic
    return {
        State.NAVIGATING: navigating, State.CONFIRMING: confirming,
        State.PAUSED: paused, State.ASKING_NEXT: asking,
        State.ASKING_WAIT_TIME: asking_time, State.WAITING: waiting,
        State.RETURNING: returning,
    }


class TestAppOverride:
    """앱 새 목적지는 ESTOPPED 만 빼고 어느 상태든 선점한다 — 내부 취소 후"""

    def test_preempts_every_active_state(self):
        nxt = _dest("", id="99999999-9999-4999-8999-999999999999", name="입구")
        for want_state, build in _builders().items():
            logic = build()
            assert logic.state == want_state
            actions, reason = logic.on_app_destination(nxt, BOUNDS, True, 50.0)
            assert reason == GateReason.OK, want_state
            assert logic.state == State.NAVIGATING, want_state
            assert logic.active_destination.name == "입구", want_state
            assert any(isinstance(a, Navigate) for a in actions), want_state
            assert logic._wait_until is None and logic.paused_destination is None

    def test_idle_still_works(self):
        logic = MissionLogic(arrival_dialog=True)
        actions, reason = logic.on_app_destination(_dest(""), BOUNDS, True, 0.0)
        assert reason == GateReason.OK and logic.state == State.NAVIGATING

    def test_estopped_rejected(self):
        logic = MissionLogic()
        logic.on_estop(True, 0.0)
        _, reason = logic.on_app_destination(_dest(""), BOUNDS, True, 1.0)
        assert reason == GateReason.ESTOP_ACTIVE

    def test_app_drive_skips_arrival_dialog(self):
        """앱 주행 도착은 도착 멘트만 하고 조용히 정지 (source 태그)."""
        logic = MissionLogic(return_destination=HOME, arrival_dialog=True)
        logic.on_app_destination(_dest("restroom"), BOUNDS, True, 0.0)
        acts = logic.on_tick(10.0, NavStatus.SUCCEEDED)
        assert logic.state == State.ARRIVED
        assert any("도착" in t for t in _say(acts))
        assert not any("기다릴까요" in t for t in _say(acts))

    def test_voice_drive_still_gets_dialog(self):
        logic = arrive("restroom")
        assert logic.state == State.ASKING_NEXT


class TestAppCancelAll:
    """앱 취소는 ESTOPPED 만 빼고 전부 정리하고 IDLE (2026-08-31 결정)."""

    def test_cancels_every_active_state(self):
        for want_state, build in _builders().items():
            logic = build()
            actions, reason = logic.on_app_cancel(50.0)
            assert reason == GateReason.OK, want_state
            assert logic.state == State.IDLE, want_state
            assert logic._wait_until is None and logic.paused_destination is None

    def test_idle_cancel_is_quiet_ok(self):
        logic = MissionLogic()
        actions, reason = logic.on_app_cancel(0.0)
        assert reason == GateReason.OK
        assert actions == []

    def test_estopped_cancel_rejected(self):
        logic = MissionLogic()
        logic.on_estop(True, 0.0)
        _, reason = logic.on_app_cancel(1.0)
        assert reason != GateReason.OK


class TestLedgerAccessors:
    """대장(P1)이 읽는 대기 접근자 — 판단이 아니라 값 노출."""

    def test_wait_minutes_and_left(self):
        logic = arrive("")
        logic.on_arrival_answer(_intent("wait", wait_minutes=10), 3.0)
        assert logic.state == State.WAITING
        assert logic.wait_minutes_requested() == 10
        assert logic.wait_left_sec(63.0) == 540
        assert logic.wait_left_sec(3.0 + 601.0) == 0

    def test_not_waiting_is_minus_one(self):
        logic = MissionLogic()
        assert logic.wait_minutes_requested() == -1 and logic.wait_left_sec(0.0) == -1
        logic = arrive("")
        logic.on_arrival_answer(_intent("wait", wait_minutes=10), 3.0)
        nxt = _dest("", id="d2", name="입구")
        logic.on_intent(_intent("navigate", matched_destination_id="d2"),
                        nxt, BOUNDS, True, 10.0)
        assert logic.state == State.NAVIGATING
        assert logic.wait_minutes_requested() == -1 and logic.wait_left_sec(10.0) == -1


class TestQuestionDuringArrivalDialog:
    """도착 질문 중 정보 질문 — LLM 이 답했으니 미션은 사다리를 쓰지 않는다(2026-10-07 검토)."""

    def test_question_keeps_the_dialog_without_the_retry_line(self):
        logic = arrive("restroom")
        assert logic.on_arrival_answer(_intent("question"), 3.0) == []
        assert logic.state == State.ASKING_NEXT
        assert logic._arrival_retried is False
        logic.on_arrival_question_spoken(4.0)
        acts = logic.on_tick(13.0, NavStatus.NONE)
        assert any(MSG_ASK_RESTROOM in t for t in _say(acts))


class TestFinishWhileWaiting:
    """대기 중 돌아와 "다 됐어" — 다음 목적지를 묻는다(2026-10-07 사용자 결정)."""

    def waiting(self):
        logic = arrive("restroom", home=HOME)
        logic.on_arrival_answer(_intent("affirm"), 3.0)
        assert logic.state == State.WAITING
        return logic

    def test_asks_where_to_and_keeps_waiting(self):
        logic = self.waiting()
        acts = logic.on_intent(_intent("finish", matched_destination_id=""), None, BOUNDS, True, 20.0)
        says = [a for a in acts if isinstance(a, Say)]
        assert [s.text for s in says] == [MSG_WAIT_FINISH_ASK]
        assert says[0].expects_reply and says[0].priority == "response"
        assert logic.state == State.WAITING

    def test_destination_after_the_question_goes(self):
        logic = self.waiting()
        logic.on_intent(_intent("finish", matched_destination_id=""), None, BOUNDS, True, 20.0)
        logic.on_intent(_intent(matched_destination_id="d2"), _dest("reception", id="d2", name="안내소"),
                        BOUNDS, True, 25.0)
        assert logic.state == State.NAVIGATING

    def test_second_finish_ends_the_guide(self):
        """갈 곳이 없다는 뜻 — 같은 질문을 되풀이하지 않고 안내를 끝낸다."""
        logic = self.waiting()
        logic.on_intent(_intent("finish", matched_destination_id=""), None, BOUNDS, True, 20.0)
        acts = logic.on_intent(_intent("finish", matched_destination_id=""), None, BOUNDS, True, 30.0)
        assert MSG_FINISH in _say(acts)
        assert logic.state == State.RETURNING

    def test_finish_much_later_asks_again(self):
        logic = self.waiting()
        logic.on_intent(_intent("finish", matched_destination_id=""), None, BOUNDS, True, 20.0)
        acts = logic.on_intent(_intent("finish", matched_destination_id=""), None, BOUNDS, True,
                               20.0 + WAIT_FINISH_REPEAT_SEC + 1.0)
        assert _say(acts) == [MSG_WAIT_FINISH_ASK]
        assert logic.state == State.WAITING
