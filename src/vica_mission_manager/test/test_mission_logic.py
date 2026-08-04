"""mission_logic 순수 로직 unit test (테스트 계획: 게이트 16+조합, 상태 전이, deg→쿼터니언)."""
import math

import pytest

from vica_mission_manager.mission_logic import (
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
    State,
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
        logic = MissionLogic(dwell_sec=2.0)
        start_navigation(logic)
        actions = logic.on_tick(10.0, NavStatus.FAILED)
        assert logic.state == State.FAILED
        assert any(isinstance(a, Say) for a in actions)
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

    def test_estop_released_say_is_not_narration(self):
        """비상 멈춤 해제 안내도 같은 이유로 narration 이면 안 된다."""
        logic = MissionLogic()
        start_navigation(logic)
        logic.on_estop(True, 1.0)
        logic.on_estop(False, 2.0)
        # 해제 유예(estop_release_grace_sec)가 지나야 안내가 나온다
        actions = logic.on_tick(2.0 + logic.estop_release_grace_sec, NavStatus.NONE)
        says = [a for a in actions if isinstance(a, Say)]
        assert says, "비상 멈춤 해제 안내가 없다"
        assert all(s.priority == "response" for s in says)

    def test_estop_latch_spam_says_once(self):
        # 20Hz 주기 발행 — 같은 상태 반복 수신 시 멘트 중복 금지
        logic = MissionLogic()
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
