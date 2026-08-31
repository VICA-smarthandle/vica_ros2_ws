"""mission_logic 순수 로직 unit test (테스트 계획: 게이트 16+조합, 상태 전이, deg→쿼터니언)."""
import math

import pytest

from vica_mission_manager.mission_logic import (
    APPROACH_TURN_TIMEOUT_SEC,
    SpinInPlace,
    MSG_APPROACH_QUESTION,
    MSG_STALE_CONFIRM,
    MSG_APPROACH_ACCEPTED,
    MSG_APPROACH_DECLINED,
    MSG_APPROACH_ONBOARDING,
    PERSON_APPROACH_SPEED_PERCENT,
    ApproachRequest,
    CancelNav,
    Destination,
    GateReason,
    IntentData,
    MapBounds,
    MissionLogic,
    Navigate,
    NavStatus,
    Pose2D,
    Say,
    SetNavSpeedLimit,
    State,
    check_approach_gate,
    check_gate,
    pose_valid,
    yaw_deg_to_quaternion,
)

BOUNDS = MapBounds(min_x=-15.1, min_y=-8.59, max_x=10.0, max_y=8.0)


def make_dest(**kw):
    defaults = dict(
        id="room_407",
        name="윤지영 교수님 사무실",
        pose=Pose2D(x=3.0, y=2.0, yaw_deg=90.0, frame_id="map"),
        is_approachable=True,
        calibrated=True,
        arrival_message="윤지영 교수님 사무실 앞에 도착했습니다.",
    )
    defaults.update(kw)
    return Destination(**defaults)


def make_intent(**kw):
    defaults = dict(
        intent="navigate",
        matched_destination_id="room_407",
        need_confirm=False,
        safety_flag="normal",
    )
    defaults.update(kw)
    return IntentData(**defaults)


# ---- 게이트 조합 -------------------------------------------------------------


class TestGate:
    def test_all_pass(self):
        assert check_gate(make_intent(), make_dest(), BOUNDS, False, True) == GateReason.OK

    @pytest.mark.parametrize("intent_type", ["question", "clarify", "unknown", ""])
    def test_not_navigate(self, intent_type):
        r = check_gate(make_intent(intent=intent_type), make_dest(), BOUNDS, False, True)
        assert r == GateReason.NOT_NAVIGATE

    def test_no_matched_id(self):
        r = check_gate(make_intent(matched_destination_id=""), make_dest(), BOUNDS, False, True)
        assert r == GateReason.NO_MATCHED_ID

    def test_need_confirm(self):
        r = check_gate(make_intent(need_confirm=True), make_dest(), BOUNDS, False, True)
        assert r == GateReason.NEED_CONFIRM

    @pytest.mark.parametrize("flag", ["emergency", "", "warn"])
    def test_safety_flag(self, flag):
        r = check_gate(make_intent(safety_flag=flag), make_dest(), BOUNDS, False, True)
        assert r == GateReason.SAFETY_FLAG

    def test_estop_active(self):
        assert (
            check_gate(make_intent(), make_dest(), BOUNDS, True, True)
            == GateReason.ESTOP_ACTIVE
        )

    def test_unknown_destination(self):
        assert check_gate(make_intent(), None, BOUNDS, False, True) == GateReason.UNKNOWN_DESTINATION

    def test_private_destination(self):
        dest = make_dest(authorization="private")
        assert (
            check_gate(make_intent(), dest, BOUNDS, False, True)
            == GateReason.PRIVATE_DESTINATION
        )

    def test_not_approachable(self):
        dest = make_dest(is_approachable=False)
        assert check_gate(make_intent(), dest, BOUNDS, False, True) == GateReason.NOT_APPROACHABLE

    def test_pose_zero_placeholder(self):
        dest = make_dest(pose=Pose2D(0.0, 0.0, 0.0), calibrated=None)
        assert check_gate(make_intent(), dest, BOUNDS, False, True) == GateReason.POSE_INVALID

    def test_pose_calibrated_false(self):
        dest = make_dest(calibrated=False)
        assert check_gate(make_intent(), dest, BOUNDS, False, True) == GateReason.POSE_INVALID

    def test_pose_wrong_frame(self):
        dest = make_dest(pose=Pose2D(3.0, 2.0, 0.0, frame_id="odom"))
        assert check_gate(make_intent(), dest, BOUNDS, False, True) == GateReason.POSE_INVALID

    @pytest.mark.parametrize("x,y", [(100.0, 0.0), (0.0, 100.0), (-20.0, 0.0), (0.0, -20.0)])
    def test_pose_out_of_bounds(self, x, y):
        dest = make_dest(pose=Pose2D(x, y, 0.0))
        assert check_gate(make_intent(), dest, BOUNDS, False, True) == GateReason.POSE_INVALID

    def test_nav_not_ready(self):
        assert (
            check_gate(make_intent(), make_dest(), BOUNDS, False, False)
            == GateReason.NAV_NOT_READY
        )

    def test_bounds_none_skips_bounds_check(self):
        dest = make_dest(pose=Pose2D(100.0, 100.0, 0.0))
        assert check_gate(make_intent(), dest, None, False, True) == GateReason.OK

    def test_priority_not_navigate_before_others(self):
        # 여러 조건이 동시에 실패해도 첫 실패 사유를 돌려준다
        r = check_gate(
            make_intent(intent="question", matched_destination_id=""),
            None, BOUNDS, True, False,
        )
        assert r == GateReason.NOT_NAVIGATE


class TestPoseValid:
    def test_valid(self):
        assert pose_valid(make_dest(), BOUNDS)

    def test_calibrated_none_nonzero_pose_ok(self):
        # calibrated 필드가 없는 기존 yaml — (0,0) 아니면 통과
        assert pose_valid(make_dest(calibrated=None), BOUNDS)

    def test_boundary_inclusive(self):
        dest = make_dest(pose=Pose2D(BOUNDS.max_x, BOUNDS.max_y, 0.0))
        assert pose_valid(dest, BOUNDS)


# ---- deg → 쿼터니언 ----------------------------------------------------------


class TestYawConversion:
    @pytest.mark.parametrize(
        "deg,expect_z,expect_w",
        [
            (0.0, 0.0, 1.0),
            (90.0, math.sin(math.pi / 4), math.cos(math.pi / 4)),
            (180.0, 1.0, 0.0),
            (-90.0, -math.sin(math.pi / 4), math.cos(math.pi / 4)),
            (360.0, 0.0, -1.0),  # -q == q (같은 회전)
        ],
    )
    def test_values(self, deg, expect_z, expect_w):
        x, y, z, w = yaw_deg_to_quaternion(deg)
        assert x == 0.0 and y == 0.0
        assert z == pytest.approx(expect_z, abs=1e-9)
        assert w == pytest.approx(expect_w, abs=1e-9)

    def test_unit_norm(self):
        for deg in (-720.5, -33.3, 0.0, 45.0, 123.4, 719.9):
            x, y, z, w = yaw_deg_to_quaternion(deg)
            assert math.hypot(z, w) == pytest.approx(1.0)


# ---- 상태 전이 ---------------------------------------------------------------


def start_navigation(logic, t=0.0):
    actions = logic.on_intent(make_intent(), make_dest(), BOUNDS, True, t)
    assert logic.state == State.NAVIGATING
    return actions


class TestTransitions:
    def test_idle_direct_navigate(self):
        logic = MissionLogic()
        actions = start_navigation(logic)
        assert any(isinstance(a, Navigate) for a in actions)
        assert any(isinstance(a, Say) for a in actions)

    def test_confirm_then_same_dest_navigates(self):
        logic = MissionLogic()
        assert logic.on_intent(make_intent(need_confirm=True), make_dest(), BOUNDS, True, 0.0) == []
        assert logic.state == State.CONFIRMING
        actions = logic.on_intent(make_intent(), make_dest(), BOUNDS, True, 5.0)
        assert logic.state == State.NAVIGATING
        assert any(isinstance(a, Navigate) for a in actions)

    def test_confirm_timeout_30s(self):
        logic = MissionLogic(confirm_timeout_sec=30.0)
        logic.on_intent(make_intent(need_confirm=True), make_dest(), BOUNDS, True, 0.0)
        assert logic.on_tick(29.9, NavStatus.NONE) == []
        assert logic.state == State.CONFIRMING
        actions = logic.on_tick(30.0, NavStatus.NONE)
        assert logic.state == State.IDLE
        assert any(isinstance(a, Say) for a in actions)

    def test_stale_confirm_different_dest_rejected(self):
        logic = MissionLogic()
        logic.on_intent(
            make_intent(need_confirm=True, matched_destination_id="restroom"),
            make_dest(id="restroom"), BOUNDS, True, 0.0,
        )
        actions = logic.on_intent(make_intent(matched_destination_id="room_407"),
                                  make_dest(), BOUNDS, True, 5.0)
        assert logic.state == State.IDLE
        assert any(isinstance(a, Say) for a in actions)
        assert not any(isinstance(a, Navigate) for a in actions)

    def test_navigating_rejects_new_navigate(self):
        logic = MissionLogic()
        start_navigation(logic)
        actions = logic.on_intent(make_intent(matched_destination_id="restroom"),
                                  make_dest(id="restroom"), BOUNDS, True, 1.0)
        assert logic.state == State.NAVIGATING
        assert actions and isinstance(actions[0], Say)

    def test_nav_success_arrival_message_then_idle(self):
        logic = MissionLogic(dwell_sec=2.0)
        start_navigation(logic)
        actions = logic.on_tick(10.0, NavStatus.SUCCEEDED)
        assert logic.state == State.ARRIVED
        assert any("도착" in a.text for a in actions if isinstance(a, Say))
        logic.on_tick(12.0, NavStatus.NONE)
        assert logic.state == State.IDLE

    def test_nav_failure_then_idle(self):
        """재시도를 끄면 실패 뒤 dwell 만큼 머물고 IDLE 로 간다."""
        logic = MissionLogic(dwell_sec=2.0, nav_retry_limit=0)
        start_navigation(logic)
        actions = logic.on_tick(10.0, NavStatus.FAILED)
        assert logic.state == State.FAILED
        assert any(isinstance(a, Say) for a in actions)
        logic.on_tick(12.5, NavStatus.NONE)
        assert logic.state == State.IDLE

    def test_nav_failure_retries_same_destination(self):
        """실패하면 같은 목적지로 스스로 다시 나선다.

        2026-08-15 실기에서 정체의 절반이 "사람이 앱을 다시 누르기까지 걸린
        시간"이었다. run9 #6 은 Goal failed 뒤 20초를 아무도 아무것도 하지 않고
        보냈다. 그 공백을 없앤다.
        """
        logic = MissionLogic(dwell_sec=2.0, nav_retry_limit=2, nav_retry_delay_sec=3.0)
        started = start_navigation(logic)
        dest = [a for a in started if isinstance(a, Navigate)][0].destination
        logic.on_tick(10.0, NavStatus.FAILED)
        assert logic.state == State.FAILED

        # dwell(2 s)이 지나도 재시도 예약이 있으면 IDLE 로 내려가지 않는다.
        # 이 순서가 뒤집히면 _to_idle 이 예약을 지워 재시도가 사라진다.
        logic.on_tick(12.5, NavStatus.NONE)
        assert logic.state == State.FAILED

        actions = logic.on_tick(13.5, NavStatus.NONE)
        assert logic.state == State.NAVIGATING
        navigates = [a for a in actions if isinstance(a, Navigate)]
        assert len(navigates) == 1
        assert navigates[0].destination.id == dest.id
        assert logic.active_destination is not None

    def test_nav_retry_stops_at_limit(self):
        """한도를 넘으면 안내하고 멈춘다. 통과 불가능한 자리에서 영원히
        시도하면 이용자가 상황을 알 수 없다."""
        logic = MissionLogic(dwell_sec=2.0, nav_retry_limit=1, nav_retry_delay_sec=1.0)
        start_navigation(logic)
        logic.on_tick(10.0, NavStatus.FAILED)
        logic.on_tick(11.5, NavStatus.NONE)
        assert logic.state == State.NAVIGATING  # 1회차 재시도

        actions = logic.on_tick(20.0, NavStatus.FAILED)
        assert logic.state == State.FAILED
        assert any("실패" in a.text for a in actions if isinstance(a, Say))
        logic.on_tick(22.5, NavStatus.NONE)
        assert logic.state == State.IDLE  # 더 시도하지 않는다

    def test_user_cancel_is_not_retried(self):
        """사용자가 거둔 목표를 로봇이 되살리면 안 된다."""
        logic = MissionLogic(dwell_sec=2.0, nav_retry_limit=2, nav_retry_delay_sec=1.0)
        start_navigation(logic)
        logic.on_tick(10.0, NavStatus.CANCELED)
        assert logic.state == State.FAILED
        logic.on_tick(12.5, NavStatus.NONE)
        assert logic.state == State.IDLE

    def test_question_during_confirming_keeps_state(self):
        logic = MissionLogic()
        logic.on_intent(make_intent(need_confirm=True), make_dest(), BOUNDS, True, 0.0)
        assert logic.on_intent(make_intent(intent="question"), None, BOUNDS, True, 5.0) == []
        assert logic.state == State.CONFIRMING


class TestEmergency:
    def test_hard_keyword_cancels_and_estops(self):
        logic = MissionLogic()
        start_navigation(logic)
        actions = logic.on_emergency("멈춰", 5.0)
        assert logic.state == State.ESTOPPED
        assert any(isinstance(a, CancelNav) for a in actions)
        assert any(isinstance(a, Say) for a in actions)

    @pytest.mark.parametrize("kw", ["멈춰", "정지", "스탑", "스톱", "안돼", "위험해"])
    def test_all_hard_keywords(self, kw):
        logic = MissionLogic()
        start_navigation(logic)
        logic.on_emergency(kw, 5.0)
        assert logic.state == State.ESTOPPED

    @pytest.mark.parametrize("kw", ["천천히", "느리게", "잠깐", "아무말"])
    def test_soft_keywords_ignored_v1(self, kw):
        logic = MissionLogic()
        start_navigation(logic)
        assert logic.on_emergency(kw, 5.0) == []
        assert logic.state == State.NAVIGATING

    def test_estop_latch_true_while_navigating(self):
        logic = MissionLogic()
        start_navigation(logic)
        actions = logic.on_estop(True, 5.0)
        assert logic.state == State.ESTOPPED
        assert any(isinstance(a, CancelNav) for a in actions)

    def test_estopped_rejects_navigate(self):
        logic = MissionLogic()
        logic.on_estop(True, 0.0)
        actions = logic.on_intent(make_intent(), make_dest(), BOUNDS, True, 1.0)
        assert logic.state == State.ESTOPPED
        assert actions and isinstance(actions[0], Say)
        assert not any(isinstance(a, Navigate) for a in actions)

    def test_estop_release_needs_grace_and_no_auto_resume(self):
        logic = MissionLogic(estop_release_grace_sec=2.0)
        start_navigation(logic)
        logic.on_estop(True, 5.0)
        logic.on_estop(False, 6.0)
        assert logic.on_tick(7.9, NavStatus.NONE) == []
        assert logic.state == State.ESTOPPED
        actions = logic.on_tick(8.0, NavStatus.NONE)
        assert logic.state == State.IDLE
        assert any(isinstance(a, Say) for a in actions)
        # 자동 재개 금지: active_destination 이 남아 있지 않다
        assert logic.active_destination is None

    def test_voice_only_estop_without_latch_releases_after_grace(self):
        # 진행순서 ③(래치 배선) 미배포 상태의 심층 방어 경로
        logic = MissionLogic(estop_release_grace_sec=2.0)
        start_navigation(logic)
        logic.on_emergency("멈춰", 5.0)
        assert logic.on_tick(6.9, NavStatus.NONE) == []
        logic.on_tick(7.0, NavStatus.NONE)
        assert logic.state == State.IDLE

    def test_estop_reasserted_resets_release(self):
        logic = MissionLogic(estop_release_grace_sec=2.0)
        logic.on_estop(True, 0.0)
        logic.on_estop(False, 1.0)
        logic.on_estop(True, 2.0)  # 해제 도중 재활성화
        logic.on_tick(10.0, NavStatus.NONE)
        assert logic.state == State.ESTOPPED

    def test_estop_say_has_emergency_priority(self):
        # TTS 큐에서 긴급 멘트가 내레이션/응답을 앞지르는 계약 (tts_queue.py)
        logic = MissionLogic()
        start_navigation(logic)
        actions = logic.on_emergency("멈춰", 5.0)
        says = [a for a in actions if isinstance(a, Say)]
        assert says and all(s.priority == "emergency" for s in says)

    def test_reject_say_has_response_priority(self):
        logic = MissionLogic()
        start_navigation(logic)
        actions = logic.on_intent(make_intent(matched_destination_id="restroom"),
                                  make_dest(id="restroom"), BOUNDS, True, 1.0)
        assert actions[0].priority == "response"

    def test_start_and_arrival_say_are_narration(self):
        logic = MissionLogic()
        actions = start_navigation(logic)
        assert all(s.priority == "narration" for s in actions if isinstance(s, Say))
        actions = logic.on_tick(10.0, NavStatus.SUCCEEDED)
        assert all(s.priority == "narration" for s in actions if isinstance(s, Say))

    def test_nav_failed_say_is_not_narration(self):
        """주행 실패 안내는 큐 정원 초과로 버려지면 안 된다 (tts_queue._trim).

        narration 은 가장 먼저 버려지는 등급이다. 사용자가 왜 멈췄는지 알 유일한
        단서가 조용히 사라지면, 눈으로 확인할 수 없는 사용자는 상태를 오해한다.
        """
        logic = MissionLogic()
        start_navigation(logic)
        actions = logic.on_tick(10.0, NavStatus.FAILED)
        says = [a for a in actions if isinstance(a, Say)]
        assert says, "주행 실패 시 안내가 없다"
        assert all(s.priority == "response" for s in says)

    def test_estop_released_say_preempts_estop_ment(self):
        """해제 안내는 emergency 등급 — 걸림 멘트가 아직 재생 중이면 끊고
        즉시 나가야 한다(2026-08-31 결정). narration 은 물론 response 도
        안 된다: response 는 걸림 멘트 완주를 기다려 낡은 소식이 된다."""
        logic = MissionLogic()
        start_navigation(logic)
        logic.on_estop(True, 1.0)
        logic.on_estop(False, 2.0)
        # 해제 유예(estop_release_grace_sec)가 지나야 안내가 나온다
        actions = logic.on_tick(2.0 + logic.estop_release_grace_sec, NavStatus.NONE)
        says = [a for a in actions if isinstance(a, Say)]
        assert says, "비상 멈춤 해제 안내가 없다"
        assert all(s.priority == "emergency" for s in says)

    def test_estop_latch_spam_says_once(self):
        # 20Hz 주기 발행 — 같은 상태 반복 수신 시 멘트 중복 금지
        # (정지 중 걸림은 침묵 규칙이라 주행 중으로 검증한다)
        logic = MissionLogic()
        start_navigation(logic)
        first = logic.on_estop(True, 0.0)
        assert any(isinstance(a, Say) for a in first)
        assert logic.on_estop(True, 0.05) == []
        assert logic.on_estop(True, 0.10) == []


# ---- 취소 / 일시정지 / 재개 ---------------------------------------------------


class TestPauseResumeCancel:
    """안전 사건이 아닌 목표 조작. E-stop 과 달리 래치도 reset 도 없다."""

    def test_pause_keeps_destination_and_resume_returns_to_it(self):
        logic = MissionLogic()
        start_navigation(logic)
        dest = logic.active_destination

        actions, reason = logic.on_pause_request(1.0)
        assert reason == GateReason.OK
        assert logic.state == State.PAUSED
        assert logic.paused_destination is dest
        assert logic.active_destination is None

        actions, reason = logic.on_resume_request(True, 2.0)
        assert reason == GateReason.OK
        assert logic.state == State.NAVIGATING
        navigate = [a for a in actions if isinstance(a, Navigate)]
        assert len(navigate) == 1
        assert navigate[0].destination is dest
        assert logic.paused_destination is None

    def test_pause_emits_paused_event_not_canceled(self):
        # 앱이 일시정지를 "주행 끝"으로 오해하면 안 된다.
        logic = MissionLogic()
        start_navigation(logic)
        actions, _ = logic.on_pause_request(1.0)
        cancels = [a for a in actions if isinstance(a, CancelNav)]
        assert len(cancels) == 1
        assert cancels[0].event == "goal_paused"

    def test_estop_discards_paused_destination(self):
        # 보관분이 남으면 E-stop 뒤에 "다시 출발"이 통해 자동 재개 금지가 깨진다.
        logic = MissionLogic()
        start_navigation(logic)
        logic.on_pause_request(1.0)
        logic.on_estop(True, 2.0)
        assert logic.state == State.ESTOPPED
        assert logic.paused_destination is None

        _, reason = logic.on_resume_request(True, 3.0)
        assert reason == GateReason.ESTOP_ACTIVE

        # 해제하고 grace 가 지나 idle 로 돌아와도 재개 대상은 없어야 한다.
        logic.on_estop(False, 4.0)
        logic.on_tick(10.0, NavStatus.NONE)
        _, reason = logic.on_resume_request(True, 11.0)
        assert reason == GateReason.NOT_PAUSED

    @pytest.mark.parametrize(
        "command", ["on_cancel_request", "on_pause_request"]
    )
    def test_estop_blocks_cancel_and_pause(self, command):
        logic = MissionLogic()
        start_navigation(logic)
        logic.on_estop(True, 1.0)
        _, reason = getattr(logic, command)(2.0)
        assert reason == GateReason.ESTOP_ACTIVE

    def test_cancel_requires_active_navigation(self):
        logic = MissionLogic()
        _, reason = logic.on_cancel_request(0.0)
        assert reason == GateReason.NOT_NAVIGATING

    def test_cancel_clears_everything(self):
        logic = MissionLogic()
        start_navigation(logic)
        actions, reason = logic.on_cancel_request(1.0)
        assert reason == GateReason.OK
        assert logic.state == State.IDLE
        assert logic.active_destination is None
        assert logic.paused_destination is None
        cancels = [a for a in actions if isinstance(a, CancelNav)]
        assert cancels and cancels[0].event == "goal_canceled"

    def test_cancel_allowed_while_paused(self):
        logic = MissionLogic()
        start_navigation(logic)
        logic.on_pause_request(1.0)
        _, reason = logic.on_cancel_request(2.0)
        assert reason == GateReason.OK
        assert logic.state == State.IDLE

    def test_resume_requires_paused_state(self):
        logic = MissionLogic()
        start_navigation(logic)
        _, reason = logic.on_resume_request(True, 1.0)
        assert reason == GateReason.NOT_PAUSED


class TestVoiceCancelConfirm:
    """음성 취소는 잘못 알아들으면 안내가 끊기므로 되물어 확인한다."""

    def test_confirm_request_does_not_cancel_yet(self):
        logic = MissionLogic()
        start_navigation(logic)
        actions, reason = logic.on_cancel_confirm_request(1.0)
        assert reason == GateReason.OK
        assert logic.cancel_confirm_pending is True
        # 확인하는 동안에도 주행은 계속된다.
        assert logic.state == State.NAVIGATING
        assert not any(isinstance(a, CancelNav) for a in actions)

    def test_confirm_question_expects_a_reply(self):
        """되묻기는 질문이다. expects_reply 로 표시해야 노드가
        /vica/listen_request 를 발행하고, 사용자가 "비카야" 재호출 없이
        "네/아니요"로 답할 수 있다."""
        logic = MissionLogic()
        start_navigation(logic)
        actions, _ = logic.on_cancel_confirm_request(1.0)
        says = [a for a in actions if isinstance(a, Say)]
        assert says and says[0].expects_reply is True

    def test_plain_announcements_do_not_expect_a_reply(self):
        """일반 안내 멘트("안내를 시작합니다")에 재청취가 걸리면, 말 끝날 때마다
        마이크가 열려 주변 소음이 발화로 들어간다. 기본값은 반드시 False."""
        logic = MissionLogic()
        actions = start_navigation(logic)
        says = [a for a in actions if isinstance(a, Say)]
        assert says
        assert all(s.expects_reply is False for s in says)

    def test_affirmative_answer_cancels(self):
        logic = MissionLogic()
        start_navigation(logic)
        logic.on_cancel_confirm_request(1.0)
        actions = logic.on_cancel_confirm_answer(True, 2.0)
        assert logic.state == State.IDLE
        assert any(isinstance(a, CancelNav) for a in actions)

    def test_negative_answer_keeps_navigating(self):
        logic = MissionLogic()
        start_navigation(logic)
        logic.on_cancel_confirm_request(1.0)
        logic.on_cancel_confirm_answer(False, 2.0)
        assert logic.state == State.NAVIGATING
        assert logic.cancel_confirm_pending is False

    def test_timeout_keeps_navigating(self):
        # 응답이 없으면 취소하지 않고 안내를 이어간다.
        logic = MissionLogic()
        start_navigation(logic)
        logic.on_cancel_confirm_request(1.0)
        logic.on_tick(1.0 + logic.confirm_timeout_sec + 0.1, NavStatus.RUNNING)
        assert logic.cancel_confirm_pending is False
        assert logic.state == State.NAVIGATING


# ---- 사람 접근 (탐지 → 접근 → 질문 → 응답분기) ---------------------------------
#
# 설계 정본: devlog/2026-08-23-사람접근-구현설계.md 4절.
#
#   IDLE ──approachable──→ APPROACHING ──도착──→ AWAITING_USER
#    ↑                          │                     │
#    │                          │ 실패·이탈·포기      ├─ "네"  → (이번 범위 끝)
#    └───────── RETURNING ←─────┴─────────────────────┴─ "아니오"·무응답 8초


def make_home(**kw):
    """복귀할 대기 위치. 지도상 좌표는 아직 [미정] 이라 시험에서만 정한다."""
    defaults = dict(
        id="standby",
        name="대기 위치",
        pose=Pose2D(x=-2.0, y=-1.0, yaw_deg=0.0, frame_id="map"),
        calibrated=True,
    )
    defaults.update(kw)
    return Destination(**defaults)


def make_approach(**kw):
    """접근 요청. goal 은 approach_geometry.approach_goal() 이 이미 계산한 값이다."""
    defaults = dict(
        goal=Pose2D(x=1.0, y=0.5, yaw_deg=30.0, frame_id="map"),
        track_id=7,
        approachable=True,
    )
    defaults.update(kw)
    return ApproachRequest(**defaults)


def start_approach(logic, t=0.0, request=None, nav_ready=True):
    actions, reason = logic.on_approach_request(
        request or make_approach(), BOUNDS, nav_ready, t
    )
    assert reason == GateReason.OK
    assert logic.state == State.APPROACHING
    return actions


def arrive_and_ask(logic, t=5.0):
    """접근 goal 도착 → 질문 → AWAITING_USER 까지 진행시킨다."""
    actions = logic.on_tick(t, NavStatus.SUCCEEDED)
    assert logic.state == State.AWAITING_USER
    return actions


class TestApproachGate:
    """탐지 결과는 요청이지 goal 이 아니다. 승인은 Mission Manager 만 한다."""

    def test_idle_request_passes(self):
        assert (
            check_approach_gate(
                make_approach(), State.IDLE, None, BOUNDS, False, True, False
            )
            == GateReason.OK
        )

    def test_not_approachable_rejected(self):
        # 시계열 판정은 detector 몫이지만, 값이 실려 오면 Mission 도 확인한다.
        r = check_approach_gate(
            make_approach(approachable=False), State.IDLE, None, BOUNDS, False, True, False
        )
        assert r == GateReason.NOT_APPROACHABLE

    def test_track_id_none_rejected(self):
        # PersonDetection.TRACK_ID_NONE(0) — 추적 id 가 없으면 억제도 못 건다.
        r = check_approach_gate(
            make_approach(track_id=0), State.IDLE, None, BOUNDS, False, True, False
        )
        assert r == GateReason.NO_TRACK_ID

    def test_estop_rejected(self):
        r = check_approach_gate(
            make_approach(), State.IDLE, None, BOUNDS, True, True, False
        )
        assert r == GateReason.ESTOP_ACTIVE

    def test_suppressed_track_rejected(self):
        r = check_approach_gate(
            make_approach(), State.IDLE, None, BOUNDS, False, True, True
        )
        assert r == GateReason.TRACK_SUPPRESSED

    @pytest.mark.parametrize(
        "state", [State.NAVIGATING, State.CONFIRMING, State.PAUSED, State.ARRIVED]
    )
    def test_busy_with_a_real_guidance_rejected(self, state):
        # 안내 중인 사용자가 우선이다. 접근은 IDLE 에서만 시작한다.
        r = check_approach_gate(make_approach(), state, None, BOUNDS, False, True, False)
        assert r == GateReason.BUSY_NAVIGATING

    @pytest.mark.parametrize(
        "state", [State.AWAITING_USER, State.RETURNING]
    )
    def test_busy_approaching_rejected(self, state):
        r = check_approach_gate(make_approach(), state, 7, BOUNDS, False, True, False)
        assert r == GateReason.BUSY_APPROACHING

    def test_other_track_while_approaching_rejected(self):
        r = check_approach_gate(
            make_approach(track_id=9), State.APPROACHING, 7, BOUNDS, False, True, False
        )
        assert r == GateReason.BUSY_APPROACHING

    def test_same_track_while_approaching_allowed(self):
        # 사람이 움직이면 goal 을 갱신해야 한다 (설계 5절).
        r = check_approach_gate(
            make_approach(track_id=7), State.APPROACHING, 7, BOUNDS, False, True, False
        )
        assert r == GateReason.OK

    def test_goal_none_rejected(self):
        # approach_geometry.approach_goal() 이 방향을 못 정하면 None 을 준다.
        r = check_approach_gate(
            make_approach(goal=None), State.IDLE, None, BOUNDS, False, True, False
        )
        assert r == GateReason.POSE_INVALID

    def test_goal_out_of_map_rejected(self):
        r = check_approach_gate(
            make_approach(goal=Pose2D(100.0, 100.0, 0.0)),
            State.IDLE, None, BOUNDS, False, True, False,
        )
        assert r == GateReason.POSE_INVALID

    def test_goal_wrong_frame_rejected(self):
        r = check_approach_gate(
            make_approach(goal=Pose2D(1.0, 0.5, 0.0, frame_id="base_link")),
            State.IDLE, None, BOUNDS, False, True, False,
        )
        assert r == GateReason.POSE_INVALID

    def test_nav_not_ready_rejected(self):
        r = check_approach_gate(
            make_approach(), State.IDLE, None, BOUNDS, False, False, False
        )
        assert r == GateReason.NAV_NOT_READY


class TestApproachTransitions:
    def test_idle_to_approaching_sends_goal_with_speed_limit(self):
        logic = MissionLogic()
        actions = start_approach(logic)
        navigates = [a for a in actions if isinstance(a, Navigate)]
        assert len(navigates) == 1
        assert navigates[0].destination.pose == make_approach().goal
        limits = [a for a in actions if isinstance(a, SetNavSpeedLimit)]
        assert limits and limits[0].percent == PERSON_APPROACH_SPEED_PERCENT
        assert logic.approach_track_id == 7

    def test_rejected_request_changes_nothing(self):
        logic = MissionLogic()
        actions, reason = logic.on_approach_request(
            make_approach(approachable=False), BOUNDS, True, 0.0
        )
        assert reason == GateReason.NOT_APPROACHABLE
        assert actions == []
        assert logic.state == State.IDLE

    def test_rejection_is_silent(self):
        """거절을 말로 하지 않는다. 요청자는 사람이 아니라 노드이고, 다가가지도
        않은 사람에게 로봇이 혼잣말을 하면 그것이 더 이상하다."""
        logic = MissionLogic()
        logic.on_estop(True, 0.0)
        actions, reason = logic.on_approach_request(make_approach(), BOUNDS, True, 1.0)
        assert reason == GateReason.ESTOP_ACTIVE
        assert actions == []

    def test_arrival_asks_and_releases_speed_limit(self):
        logic = MissionLogic()
        start_approach(logic)
        actions = arrive_and_ask(logic, 5.0)
        says = [a for a in actions if isinstance(a, Say)]
        assert says and says[0].text == MSG_APPROACH_QUESTION
        # 질문이므로 재청취 창을 연다 — "비카야" 재호출 없이 답하게 한다.
        assert says[0].expects_reply is True
        # 안내 멘트가 아니라 응답이다. 큐 정원 초과로 버려지면 대화가 끊긴다.
        assert says[0].priority == "response"
        limits = [a for a in actions if isinstance(a, SetNavSpeedLimit)]
        assert limits and limits[0].percent == 0.0

    def test_yes_turns_handle_toward_person(self):
        """수락하면 180도 돌아 핸들을 사람 쪽으로 낸다 (2026-08-24 범위 확장).

        정지 거리 1.1 m 는 애초에 이 회전의 반경 기준으로 설계됐다(설계 6.3절).
        """
        logic = MissionLogic()
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        actions = logic.on_approach_answer(True, 6.0)
        assert logic.state == State.TURNING
        assert any(isinstance(a, Say) for a in actions)
        spins = [a for a in actions if isinstance(a, SpinInPlace)]
        assert len(spins) == 1
        assert spins[0].yaw_rad == pytest.approx(math.pi)
        # 회전은 Navigate 가 아니다 — goal 을 새로 만들지 않는다.
        assert not any(isinstance(a, Navigate) for a in actions)

    def test_yes_suppresses_track_before_turn(self):
        # 수락한 사람에게 로봇이 곧바로 다시 다가가면 안 된다 — 회전과 무관하게.
        logic = MissionLogic()
        start_approach(logic, request=make_approach(track_id=7))
        arrive_and_ask(logic, 5.0)
        logic.on_approach_answer(True, 6.0)
        logic.on_tick(8.0, NavStatus.SUCCEEDED)      # 회전 완료 -> IDLE
        _, reason = logic.on_approach_request(
            make_approach(track_id=7), BOUNDS, True, 9.0)
        assert reason == GateReason.TRACK_SUPPRESSED

    def test_turn_done_goes_idle(self):
        logic = MissionLogic()
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        logic.on_approach_answer(True, 6.0)
        actions = logic.on_tick(14.0, NavStatus.SUCCEEDED)
        assert logic.state == State.IDLE
        assert not any(isinstance(a, Navigate) for a in actions)

    def test_turn_failed_is_not_fatal(self):
        # 회전 실패(장애물 감지 등)는 안내 실패가 아니다. 조용히 끝낸다.
        logic = MissionLogic()
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        logic.on_approach_answer(True, 6.0)
        logic.on_tick(14.0, NavStatus.FAILED)
        assert logic.state == State.IDLE

    def test_turn_stuck_times_out(self):
        # spin 이 시작조차 안 되면(노드 결함) TURNING 에 갇힌다. 시계로 탈출한다.
        logic = MissionLogic()
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        logic.on_approach_answer(True, 6.0)
        assert logic.on_tick(6.0 + APPROACH_TURN_TIMEOUT_SEC - 0.1,
                             NavStatus.NONE) == []
        assert logic.state == State.TURNING
        logic.on_tick(6.0 + APPROACH_TURN_TIMEOUT_SEC, NavStatus.NONE)
        assert logic.state == State.IDLE

    def test_turn_disabled_keeps_old_behavior(self):
        logic = MissionLogic(approach_turn_yaw_rad=0.0)
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        actions = logic.on_approach_answer(True, 6.0)
        assert logic.state == State.IDLE
        assert not any(isinstance(a, SpinInPlace) for a in actions)

    def test_estop_during_turn_cancels_spin(self):
        logic = MissionLogic()
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        logic.on_approach_answer(True, 6.0)
        actions = logic.on_estop(True, 7.0)
        assert any(isinstance(a, CancelNav) for a in actions)
        assert logic.state != State.TURNING

    def test_no_returns_to_standby(self):
        logic = MissionLogic(return_destination=make_home())
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        actions = logic.on_approach_answer(False, 6.0)
        assert logic.state == State.RETURNING
        navigates = [a for a in actions if isinstance(a, Navigate)]
        assert len(navigates) == 1
        assert navigates[0].destination.id == "standby"

    def test_no_answer_within_8s_returns(self):
        logic = MissionLogic(
            return_destination=make_home(), approach_response_timeout_sec=8.0
        )
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        assert logic.on_tick(12.9, NavStatus.NONE) == []
        assert logic.state == State.AWAITING_USER
        actions = logic.on_tick(13.0, NavStatus.NONE)
        assert logic.state == State.RETURNING
        assert any(isinstance(a, Say) for a in actions)

    def test_timeout_counts_from_playback_end(self):
        """8초는 질문 재생이 끝난 시점부터다 (설계 6.2절). 재생이 언제 끝났는지는
        노드만 알 수 있으므로 노드가 시각을 다시 넣어 준다."""
        logic = MissionLogic(
            return_destination=make_home(), approach_response_timeout_sec=8.0
        )
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        logic.on_approach_question_spoken(7.0)  # 재생 종료
        assert logic.on_tick(14.9, NavStatus.NONE) == []
        assert logic.state == State.AWAITING_USER
        logic.on_tick(15.0, NavStatus.NONE)
        assert logic.state == State.RETURNING

    def test_returning_completion_goes_idle(self):
        logic = MissionLogic(return_destination=make_home())
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        logic.on_approach_answer(False, 6.0)
        logic.on_tick(20.0, NavStatus.SUCCEEDED)
        assert logic.state == State.IDLE
        assert logic.active_destination is None

    @pytest.mark.parametrize("status", [NavStatus.FAILED, NavStatus.CANCELED])
    def test_returning_finishes_even_if_it_fails(self, status):
        # 복귀에 실패해도 접근 상태에 갇히면 안 된다. 다음 요청을 받아야 한다.
        logic = MissionLogic(return_destination=make_home())
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        logic.on_approach_answer(False, 6.0)
        logic.on_tick(20.0, status)
        assert logic.state == State.IDLE

    def test_returning_without_standby_pose_finishes(self):
        """대기 위치는 아직 [미정] 이다. 좌표가 없으면 제자리에서 접근만 끝낸다."""
        logic = MissionLogic(return_destination=None)
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        actions = logic.on_approach_answer(False, 6.0)
        assert not any(isinstance(a, Navigate) for a in actions)
        assert logic.state == State.RETURNING
        logic.on_tick(6.5, NavStatus.NONE)
        assert logic.state == State.IDLE

    def test_approach_failure_returns_without_retry(self):
        """접근 실패는 재시도하지 않는다. 사람은 3초 뒤 그 자리에 없다."""
        logic = MissionLogic(return_destination=make_home(), nav_retry_limit=2)
        start_approach(logic)
        actions = logic.on_tick(5.0, NavStatus.FAILED)
        assert logic.state == State.RETURNING
        assert not any(
            isinstance(a, Navigate) and a.destination.id.startswith("approach")
            for a in actions
        )

    def test_approach_does_not_announce_distance(self):
        """남은 거리 안내는 핸들을 잡은 사용자용이다. 아직 남이다."""
        logic = MissionLogic()
        start_approach(logic)
        actions = logic.on_tick(1.0, NavStatus.RUNNING, distance_remaining=3.0)
        assert not any(isinstance(a, Say) for a in actions)

    def test_answer_without_question_is_ignored(self):
        logic = MissionLogic()
        assert logic.on_approach_answer(True, 1.0) == []
        assert logic.state == State.IDLE


class TestApproachGoalUpdate:
    """사람이 움직이면 새 goal 을 보낸다. 다만 자주 보내면 BT 가 처음부터 다시
    시작해 재계획만 반복한다 (설계 5절, 갱신 임계 0.5 m)."""

    def test_moved_far_enough_updates_goal(self):
        logic = MissionLogic()
        start_approach(logic)
        moved = make_approach(goal=Pose2D(x=1.8, y=0.5, yaw_deg=30.0))
        actions, reason = logic.on_approach_request(moved, BOUNDS, True, 1.0)
        assert reason == GateReason.OK
        assert logic.state == State.APPROACHING
        navigates = [a for a in actions if isinstance(a, Navigate)]
        assert len(navigates) == 1
        assert navigates[0].destination.pose.x == 1.8

    def test_small_movement_does_not_resend(self):
        logic = MissionLogic()
        start_approach(logic)
        nudged = make_approach(goal=Pose2D(x=1.2, y=0.5, yaw_deg=30.0))
        actions, reason = logic.on_approach_request(nudged, BOUNDS, True, 1.0)
        assert reason == GateReason.OK
        assert actions == []
        assert logic.approach_goal_pose.x == 1.0  # 보내지 않았으니 그대로다


class TestReapproachSuppression:
    """RETURNING 완료 후 같은 track_id 는 60초간 재접근하지 않는다 (설계 4절)."""

    def _return_once(self, logic, t=0.0):
        start_approach(logic, t)
        arrive_and_ask(logic, t + 5.0)
        logic.on_approach_answer(False, t + 6.0)
        logic.on_tick(t + 10.0, NavStatus.SUCCEEDED)
        assert logic.state == State.IDLE

    def test_same_track_blocked_for_60s(self):
        logic = MissionLogic(return_destination=make_home(), reapproach_suppress_sec=60.0)
        self._return_once(logic)
        _, reason = logic.on_approach_request(make_approach(), BOUNDS, True, 69.9)
        assert reason == GateReason.TRACK_SUPPRESSED
        _, reason = logic.on_approach_request(make_approach(), BOUNDS, True, 70.0)
        assert reason == GateReason.OK

    def test_other_track_is_free(self):
        logic = MissionLogic(return_destination=make_home())
        self._return_once(logic)
        _, reason = logic.on_approach_request(
            make_approach(track_id=8), BOUNDS, True, 11.0
        )
        assert reason == GateReason.OK

    def test_accepted_person_is_also_suppressed(self):
        """"네"라고 답한 사람에게 곧바로 다시 다가가면 안 된다. 인계는 다음
        사이클 몫이고, 그 사이 재접근이 열려 있으면 로봇이 같은 사람에게 계속
        다가간다."""
        logic = MissionLogic()
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        logic.on_approach_answer(True, 6.0)
        # 회전 중에는 busy 로 거절된다. 억제는 회전이 끝난 뒤부터 판정한다.
        _, busy = logic.on_approach_request(make_approach(), BOUNDS, True, 7.0)
        assert busy != GateReason.OK
        logic.on_tick(8.0, NavStatus.SUCCEEDED)      # 회전 완료 -> IDLE
        _, reason = logic.on_approach_request(make_approach(), BOUNDS, True, 9.0)
        assert reason == GateReason.TRACK_SUPPRESSED


class TestApproachCancel:
    """이탈·포기 판정은 detector 가 하고 /vica/mission/cancel_approach 로 알린다."""

    def test_cancel_while_approaching_returns(self):
        logic = MissionLogic(return_destination=make_home())
        start_approach(logic)
        actions, reason = logic.on_approach_cancel_request(3.0)
        assert reason == GateReason.OK
        assert logic.state == State.RETURNING
        assert any(isinstance(a, CancelNav) for a in actions)

    def test_cancel_while_awaiting_user_returns(self):
        logic = MissionLogic(return_destination=make_home())
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        _, reason = logic.on_approach_cancel_request(6.0)
        assert reason == GateReason.OK
        assert logic.state == State.RETURNING

    def test_cancel_when_not_approaching_rejected(self):
        logic = MissionLogic()
        _, reason = logic.on_approach_cancel_request(1.0)
        assert reason == GateReason.NOT_APPROACHING
        assert logic.state == State.IDLE

    def test_cancel_during_guidance_is_not_an_approach_cancel(self):
        # 안내 주행을 접근 취소로 끊으면 안 된다. 경로가 다르다.
        logic = MissionLogic()
        start_navigation(logic)
        _, reason = logic.on_approach_cancel_request(1.0)
        assert reason == GateReason.NOT_APPROACHING
        assert logic.state == State.NAVIGATING


class TestApproachSafety:
    """접근 중에도 E-stop 과 긴급어는 그대로 작동한다 (설계 7절)."""

    @pytest.mark.parametrize(
        "state_setup",
        ["approaching", "awaiting_user", "returning"],
    )
    def test_estop_drops_the_approach(self, state_setup):
        logic = MissionLogic(return_destination=make_home())
        start_approach(logic)
        if state_setup != "approaching":
            arrive_and_ask(logic, 5.0)
        if state_setup == "returning":
            logic.on_approach_answer(False, 6.0)
            assert logic.state == State.RETURNING

        logic.on_estop(True, 7.0)
        assert logic.state == State.ESTOPPED
        # 목적지를 보관하지 않는다 — 해제 뒤 자동 재개가 없어야 한다.
        assert logic.active_destination is None
        assert logic.paused_destination is None
        assert logic.approach_track_id is None

    def test_estop_while_approaching_cancels_the_goal(self):
        logic = MissionLogic()
        start_approach(logic)
        actions = logic.on_estop(True, 3.0)
        assert any(isinstance(a, CancelNav) for a in actions)
        assert any(
            isinstance(a, SetNavSpeedLimit) and a.percent == 0.0 for a in actions
        )

    def test_estop_release_returns_to_idle_without_resuming(self):
        logic = MissionLogic(estop_release_grace_sec=2.0)
        start_approach(logic)
        logic.on_estop(True, 3.0)
        logic.on_estop(False, 4.0)
        logic.on_tick(6.0, NavStatus.NONE)
        assert logic.state == State.IDLE
        assert logic.approach_track_id is None
        assert logic.approach_goal_pose is None

    def test_estopped_person_is_not_reapproached_at_once(self):
        # 비상 정지가 걸린 대상에게 해제 직후 다시 다가가지 않는다.
        logic = MissionLogic(estop_release_grace_sec=2.0)
        start_approach(logic)
        logic.on_estop(True, 3.0)
        logic.on_estop(False, 4.0)
        logic.on_tick(6.0, NavStatus.NONE)
        _, reason = logic.on_approach_request(make_approach(), BOUNDS, True, 7.0)
        assert reason == GateReason.TRACK_SUPPRESSED

    @pytest.mark.parametrize("kw", ["멈춰", "정지", "위험해"])
    def test_hard_keyword_still_works_while_approaching(self, kw):
        logic = MissionLogic()
        start_approach(logic)
        actions = logic.on_emergency(kw, 3.0)
        assert logic.state == State.ESTOPPED
        assert any(isinstance(a, CancelNav) for a in actions)

    def test_hard_keyword_while_awaiting_user(self):
        # 질문에 "안돼"로 답하면 그것은 거절이 아니라 긴급어다. 긴급어가 이긴다.
        logic = MissionLogic()
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        logic.on_emergency("안돼", 6.0)
        assert logic.state == State.ESTOPPED

    @pytest.mark.parametrize(
        "state_setup", ["approaching", "awaiting_user", "returning"]
    )
    def test_destination_request_is_rejected_during_approach(self, state_setup):
        logic = MissionLogic(return_destination=make_home())
        start_approach(logic)
        if state_setup != "approaching":
            arrive_and_ask(logic, 5.0)
        if state_setup == "returning":
            logic.on_approach_answer(False, 6.0)
        before = logic.state

        actions = logic.on_intent(make_intent(), make_dest(), BOUNDS, True, 7.0)
        assert logic.state == before
        assert not any(isinstance(a, Navigate) for a in actions)
        says = [a for a in actions if isinstance(a, Say)]
        assert says and says[0].priority == "response"

    @pytest.mark.parametrize(
        "command", ["on_cancel_request", "on_pause_request", "on_cancel_confirm_request"]
    )
    def test_guidance_commands_do_not_touch_the_approach(self, command):
        # 안내용 취소·일시정지는 접근에 관여하지 않는다. 접근 취소는 전용 경로다.
        logic = MissionLogic()
        start_approach(logic)
        _, reason = getattr(logic, command)(3.0)
        assert reason == GateReason.NOT_NAVIGATING
        assert logic.state == State.APPROACHING


class TestStaleConfirmListens:
    def test_stale_confirm_retry_prompt_expects_a_reply(self):
        """"다시 말씀해 주세요"는 질문이다 — expects_reply 없이는 말해 놓고
        안 듣는다 (2026-08-28 실기: 사용자가 "비카야"를 다시 불러야 했다)."""
        logic = MissionLogic()
        logic.on_intent(make_intent(need_confirm=True), make_dest(), BOUNDS,
                        True, 0.0)
        assert logic.state == State.CONFIRMING
        actions = logic.on_intent(
            make_intent(matched_destination_id="다른_목적지"), make_dest(),
            BOUNDS, True, 1.0)
        says = [a for a in actions if isinstance(a, Say)]
        assert says and says[0].text == MSG_STALE_CONFIRM
        assert says[0].expects_reply is True
class TestApproachVoiceHooks:
    """계획 문서(voice docs/approach-voice-flow.md)의 남은 두 조각.

    문구 정본은 voice replies.py·ment_cache — 글자까지 일치해야 사전 녹음이
    재생된다(갈라지면 캐시가 빗나가 매번 합성). 여기 하드코딩된 기대 문구가
    그 계약의 사본이다.
    """

    def _accept_with_turn(self, logic):
        start_approach(logic)
        arrive_and_ask(logic)
        logic.on_approach_answer(True, 6.0)          # 수락 -> TURNING
        assert logic.state == State.TURNING

    def test_question_is_the_recorded_long_greeting(self):
        assert MSG_APPROACH_QUESTION.startswith("안녕하세요? 저는 시각장애인")
        assert MSG_APPROACH_QUESTION.endswith("안내를 받으시겠어요?")

    def test_accept_speaks_turn_notice(self):
        """수락 멘트 = 회전 예고 — 예고 없는 움직임 금지(2026-08-25 결정)."""
        assert MSG_APPROACH_ACCEPTED == "네, 잠시만 기다려주세요. 로봇이 회전하니 주의하세요."

    def test_decline_speaks_farewell(self):
        assert MSG_APPROACH_DECLINED == "알겠습니다. 이만 물러납니다."

    def test_turn_success_speaks_done_then_onboarding(self):
        logic = MissionLogic()
        self._accept_with_turn(logic)
        actions = logic.on_tick(7.0, NavStatus.SUCCEEDED)
        says = [a for a in actions if isinstance(a, Say)]
        assert [s.text for s in says] == [
            "회전이 완료되었습니다.", MSG_APPROACH_ONBOARDING]
        assert says[1].expects_reply is True         # 온보딩 끝 = 재청취 창
        assert logic.state == State.IDLE

    def test_turn_failure_skips_done_but_still_onboards(self):
        """회전 실패에 '완료되었습니다'는 거짓말 — 생략. 다만 수락한 사람을
        침묵 속에 버려두지 않도록 온보딩은 한다."""
        logic = MissionLogic()
        self._accept_with_turn(logic)
        actions = logic.on_tick(7.0, NavStatus.FAILED)
        says = [a for a in actions if isinstance(a, Say)]
        assert [s.text for s in says] == [MSG_APPROACH_ONBOARDING]
        assert says[0].expects_reply is True
        assert logic.state == State.IDLE


class TestEstopStateNarration:
    """E-stop 안내 최종 규칙 (2026-08-31): 움직이는 중에 걸릴 때만 말한다.

    정지 중 걸림(통신 순단 자동복구 포함)은 사용자에게 달라지는 게 없어
    침묵하고, 침묵 걸림은 해제도 침묵한다(안 알린 걸 해제만 알리면 이상).
    """

    def test_estop_while_driving_announces(self):
        logic = MissionLogic()
        start_navigation(logic)
        actions = logic.on_estop(True, 1.0)
        says = [a.text for a in actions if isinstance(a, Say)]
        assert says == ["안전을 위해 멈추겠습니다. 관리자를 호출했습니다."]

    def test_estop_while_idle_is_silent(self):
        logic = MissionLogic()
        actions = logic.on_estop(True, 1.0)
        assert [a for a in actions if isinstance(a, Say)] == []

    def test_silent_estop_silent_release(self):
        logic = MissionLogic(estop_release_grace_sec=0.0)
        logic.on_estop(True, 1.0)                       # 정지 중 — 침묵
        logic.on_estop(False, 2.0)
        actions = logic.on_tick(3.0, NavStatus.NONE)
        assert [a for a in actions if isinstance(a, Say)] == []
        assert logic.state == State.IDLE                # 상태 전이는 정상

    def test_announced_estop_announces_release(self):
        logic = MissionLogic(estop_release_grace_sec=0.0)
        start_navigation(logic)
        logic.on_estop(True, 1.0)                       # 주행 중 — 발화
        logic.on_estop(False, 2.0)
        actions = logic.on_tick(3.0, NavStatus.NONE)
        says = [a.text for a in actions if isinstance(a, Say)]
        assert says == ["비상멈춤이 해제되었습니다."]

    def test_voice_emergency_while_idle_is_silent(self):
        logic = MissionLogic()
        actions = logic.on_emergency("멈춰", 1.0)
        assert [a for a in actions if isinstance(a, Say)] == []

    def test_voice_emergency_while_driving_announces(self):
        logic = MissionLogic()
        start_navigation(logic)
        actions = logic.on_emergency("멈춰", 1.0)
        says = [a.text for a in actions if isinstance(a, Say)]
        assert says == ["안전을 위해 멈추겠습니다. 관리자를 호출했습니다."]
