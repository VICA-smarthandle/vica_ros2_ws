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
        assert any(MSG_ASK_RESTROOM in t for t in _say(acts))
        assert any("도착했습니다" in t for t in _say(acts))  # 도착 멘트가 합쳐 나온다

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


class TestReturnBrake:
    def test_wake_during_return_cancels_and_asks(self):
        """홈 복귀 중 '비카야' → 복귀 취소 + 다시 안내 질문 (작업 E)."""
        from vica_mission_manager.mission_logic import CancelNav
        logic = arrive("restroom")
        logic.on_arrival_answer(_intent("finish"), 3.0)   # RETURNING
        assert logic.state == State.RETURNING
        acts = logic.on_return_brake(5.0)
        assert logic.state == State.ASKING_NEXT
        assert MSG_WAIT_RESUME_ASK in _say(acts)
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
    """도착 후 대화 중 새 목적지 '제안'은 즉시 출발하면 안 된다 (2026-08-30 실기).

    navigate 는 2단계다 — 제안(need_confirm=True, 음성이 확인 질문을 말함) →
    확정(need_confirm=False). 제안 단계에서 출발해버리면 질문 전에 달리고,
    뒤따라온 확정 답은 "주행 중 새 요청"으로 거절된다(MSG_BUSY 실기 재현).
    """

    def test_proposal_joins_confirm_flow_not_navigate(self):
        logic = arrive("restroom")
        logic.on_arrival_answer(_intent("affirm"), 3.0)     # WAITING
        logic.on_wake(10.0)                                  # 재개 질문
        # "입구로 가자" 제안 — 노드가 대화를 닫고 일반 확인 흐름으로 넘긴다
        logic.exit_arrival_dialog()
        assert logic.state == State.IDLE
        nxt = _dest("", id="d2", name="입구")
        acts = logic.on_intent(_intent("navigate", matched_destination_id="d2",
                                       need_confirm=True), nxt, BOUNDS, True, 11.0)
        assert logic.state == State.CONFIRMING               # 출발 안 함
        assert not any(isinstance(a, Navigate) for a in acts)
        # "그래" 확정 → 그때 출발
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
        logic.exit_arrival_dialog()                          # 아무 일 없음
        assert logic.state == State.IDLE


class TestEarHold:
    """무응답 시계는 귀가 바쁜 동안 멈춘다 (2026-08-30 실기 — 답이 STT·LLM
    을 통과하는 동안 8초가 먼저 울려 "응답이 없어" 하고 떠났다)."""

    def test_open_listen_holds_silence_deadline(self):
        logic = arrive("restroom")               # 질문 재생완료 2.0 → 데드라인 10.0
        logic.on_listen_state("open", 3.0)       # 사용자 쪽 창 열림
        assert _say(logic.on_tick(10.5, NavStatus.NONE)) == []   # 잡아둠
        logic.on_listen_state("speech", 11.0)    # 말하는 중
        assert _say(logic.on_tick(12.0, NavStatus.NONE)) == []

    def test_closed_grants_grace_for_llm(self):
        """전사 성공(closed) 후에도 LLM 처리 시간(6초)을 기다린다."""
        logic = arrive("restroom")
        logic.on_listen_state("open", 3.0)
        logic.on_listen_state("speech", 7.0)
        logic.on_listen_state("closed", 9.5)     # STT 통과 — LLM 진행 중
        assert _say(logic.on_tick(10.5, NavStatus.NONE)) == []   # 유예
        acts = logic.on_tick(16.0, NavStatus.NONE)               # 유예 소진
        assert any("돌아가겠습니다" in t for t in _say(acts))

    def test_empty_fires_promptly(self):
        logic = arrive("restroom")
        logic.on_listen_state("open", 3.0)
        logic.on_listen_state("empty", 8.5)      # 빈손 — 진짜 침묵
        acts = logic.on_tick(10.5, NavStatus.NONE)
        assert any("돌아가겠습니다" in t for t in _say(acts))

    def test_stuck_open_ear_has_failsafe_cap(self):
        """닫힘 신호를 영영 못 받아도 20초 상한 뒤엔 떠난다 (무한 대기 방지)."""
        logic = arrive("restroom")
        logic.on_listen_state("open", 3.0)       # 그리고 닫힘 신호 유실
        assert _say(logic.on_tick(15.0, NavStatus.NONE)) == []
        acts = logic.on_tick(24.0, NavStatus.NONE)   # 3.0+20 상한 초과
        assert any("돌아가겠습니다" in t for t in _say(acts))


class TestReturnNet:
    """복귀 중 늦게 도착한 답의 마지막 그물 (2026-08-30)."""

    def test_quiet_brake_then_wait_answer(self):
        from vica_mission_manager.mission_logic import CancelNav
        logic = arrive("restroom")
        logic.on_arrival_answer(_intent("finish"), 3.0)      # RETURNING
        acts = logic.on_return_brake(5.0, quiet=True)        # 조용한 브레이크
        assert logic.state == State.ASKING_NEXT
        assert _say(acts) == []                              # 질문 없이
        assert any(isinstance(a, CancelNav) for a in acts)
        acts2 = logic.on_arrival_answer(_intent("wait", wait_minutes=10), 5.5)
        assert logic.state == State.WAITING
        assert any("10분" in t for t in _say(acts2))


class TestAppCancelInWaiting:
    """관리자 회수 (2026-08-31): WAITING(최대 30분)은 앱 취소로 풀 수 있어야
    한다 — 이전엔 취소 게이트가 NAVIGATING/PAUSED 만 허용해 회수 불가였다.
    ASKING_* 는 길어야 1분이고 침묵이면 저절로 홈에 오므로 열지 않는다
    (사용자 결정 — 범위 최소)."""

    def test_cancel_releases_waiting(self):
        from vica_mission_manager.mission_logic import GateReason
        logic = arrive("restroom")
        logic.on_arrival_answer(_intent("wait", wait_minutes=30), 3.0)
        assert logic.state == State.WAITING
        actions, reason = logic.on_cancel_request(10.0)
        assert reason == GateReason.OK
        assert logic.state == State.IDLE
        # 대기 타이머가 완전히 정리돼야 한다 — 남으면 유령 복귀가 된다
        assert logic._wait_until is None
        assert any("취소" in t for t in _say(actions))

    def test_cancel_still_rejected_in_asking(self):
        from vica_mission_manager.mission_logic import GateReason
        logic = arrive("restroom")   # ASKING_NEXT
        actions, reason = logic.on_cancel_request(3.0)
        assert reason != GateReason.OK


class TestGenericIsWaitStyle:
    """그 외 유형 질문 개편 (2026-08-31 사용자 결정): "여기서 대기할까요?"
    — "네"=대기(시간 질문으로), "아니요"=종료. 네/아니오 판단이 단순해진다."""

    def test_generic_affirm_asks_time(self):
        logic = arrive("reception")
        acts = logic.on_arrival_answer(_intent("affirm"), 3.0)
        assert logic.state == State.ASKING_WAIT_TIME
        assert MSG_ASK_WAIT_TIME in _say(acts)

    def test_generic_deny_finishes(self):
        logic = arrive("reception")
        acts = logic.on_arrival_answer(_intent("deny"), 3.0)
        assert logic.state == State.RETURNING
        assert MSG_FINISH in _say(acts)

    def test_restroom_affirm_still_skips_time(self):
        logic = arrive("restroom")
        logic.on_arrival_answer(_intent("affirm"), 3.0)
        assert logic.state == State.WAITING          # 시간 안 묻고 기본 30분

    def test_entrance_wait_skips_time_question(self):
        """entrance 에서 "기다려줘"(시간 없음) — 스펙대로 시간 안 묻고 30분."""
        logic = arrive("entrance")
        acts = logic.on_arrival_answer(_intent("wait", wait_minutes=-1), 3.0)
        assert logic.state == State.WAITING
        assert any("30분" in t for t in _say(acts))


class TestAskWaitTimeRejectsYesNo:
    """시간 질문에 네/아니오는 답이 아니다 (구멍 ②: 옛 질문 태그가 남아
    "네"가 종료로 오해석됐다) — 재질문으로 보낸다."""

    def _to_wait_time(self):
        logic = arrive("reception")
        logic.on_arrival_answer(_intent("affirm"), 3.0)   # → ASKING_WAIT_TIME
        logic.on_arrival_question_spoken(4.0)
        return logic

    def test_affirm_reasks(self):
        logic = self._to_wait_time()
        acts = logic.on_arrival_answer(_intent("affirm"), 5.0)
        assert logic.state == State.ASKING_WAIT_TIME       # 홈에 안 감
        assert MSG_ARRIVAL_RETRY in _say(acts)

    def test_deny_reasks(self):
        logic = self._to_wait_time()
        acts = logic.on_arrival_answer(_intent("deny"), 5.0)
        assert logic.state == State.ASKING_WAIT_TIME       # 30분 대기도 안 함
        assert MSG_ARRIVAL_RETRY in _say(acts)


class TestAskingStuckFallback:
    """구멍 ①: 질문 재생이 끊기면 tts_done 이 없어 시계가 영영 시작 안 됐다
    — 진입 후 30초가 지나면 강제로 무응답 절차를 연다."""

    def test_lost_tts_done_still_leaves(self):
        logic = MissionLogic(return_destination=HOME, arrival_dialog=True)
        logic.on_intent(_intent(), _dest("restroom"), BOUNDS, True, 0.0)
        logic.on_tick(1.0, NavStatus.SUCCEEDED)            # ASKING_NEXT 진입
        # tts_done 유실 — on_arrival_question_spoken 호출 없음
        assert _say(logic.on_tick(20.0, NavStatus.NONE)) == []   # 아직 상한 전
        acts = logic.on_tick(32.0, NavStatus.NONE)               # 진입+30초 초과
        assert any("돌아가겠습니다" in t for t in _say(acts))


def test_wake_resume_does_not_trip_stuck_fallback():
    """WAITING 각성 직후 낡은 진입 시각으로 폴백이 즉발하면 안 된다."""
    logic = arrive("restroom")
    logic.on_arrival_answer(_intent("affirm"), 3.0)     # WAITING
    logic.on_wake(600.0)                                 # 10분 뒤 각성
    acts = logic.on_tick(601.0, NavStatus.NONE)          # 질문 재생 중일 시각
    assert not any("돌아가겠습니다" in t for t in _say(acts))
