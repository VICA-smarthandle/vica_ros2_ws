"""mission_logic 순수 로직 unit test (테스트 계획: 게이트 16+조합, 상태 전이, deg→쿼터니언)."""
import math

import pytest

from vica_mission_manager.mission_logic import (
    APPROACH_TURN_TIMEOUT_SEC,
    APPROACH_QUESTION_STUCK_SEC,
    SpinInPlace,
    StopSpeech,
    MSG_APPROACH_QUESTION,
    MSG_APPROACH_ACCEPTED,
    MSG_APPROACH_DECLINED,
    MSG_APPROACH_NO_ANSWER,
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
    SEEK_LOOK_SEC,
    SEEK_MIN_YAW_RAD,
    SEEK_TURN_TIMEOUT_SEC,
    USER_ATTACHED_SUPPRESS_SEC,
    WAKE_CONSUMED_GUARD_SEC,
    NEAR_CALL_MAX_M,
    NEAR_CALL_NO_SPIN_M,
    LEAVING_GRACE_SEC,
    MSG_LEAVING_NOTICE,
    RETURN_RESUME_SEC,
    doa_to_spin_yaw,
    wrap_to_pi,
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

    def test_confirm_affirm_starts_navigation(self):
        """확인 질문의 "네"는 LLM 추측 없이 그 목적지로 바로 확정 출발한다
        (2026-08-31 실기: 이 통로가 없어 affirm 이 버려지고 30초 뒤 취소됐다)."""
        logic = MissionLogic()
        logic.on_intent(make_intent(need_confirm=True), make_dest(), BOUNDS, True, 0.0)
        assert logic.confirming_dest_id == "room_407"
        actions = logic.on_confirm_answer(True, make_dest(), BOUNDS, True, 3.0)
        assert logic.state == State.NAVIGATING
        assert any(isinstance(a, Navigate) for a in actions)
        assert any(isinstance(a, Say) for a in actions)

    def test_confirm_deny_cancels_quietly_to_idle(self):
        logic = MissionLogic()
        logic.on_intent(make_intent(need_confirm=True), make_dest(), BOUNDS, True, 0.0)
        actions = logic.on_confirm_answer(False, make_dest(), BOUNDS, True, 3.0)
        assert logic.state == State.IDLE
        assert any(isinstance(a, Say) for a in actions)
        assert not any(isinstance(a, Navigate) for a in actions)

    def test_confirm_answer_ignored_outside_confirming(self):
        logic = MissionLogic()
        assert logic.on_confirm_answer(True, make_dest(), BOUNDS, True, 0.0) == []
        assert logic.state == State.IDLE
        assert logic.confirming_dest_id is None

    def test_confirm_affirm_without_dest_keeps_waiting(self):
        """목적지를 되찾지 못하면 아무 데나 출발하지 않고 확인 상태를 유지한다
        — LLM 확정 navigate 나 타임아웃이 이어받는다."""
        logic = MissionLogic()
        logic.on_intent(make_intent(need_confirm=True), make_dest(), BOUNDS, True, 0.0)
        assert logic.on_confirm_answer(True, None, BOUNDS, True, 3.0) == []
        assert logic.state == State.CONFIRMING
        # 엉뚱한 목적지가 넘어와도 마찬가지다.
        assert logic.on_confirm_answer(
            True, make_dest(id="restroom"), BOUNDS, True, 4.0) == []
        assert logic.state == State.CONFIRMING

    def test_confirm_affirm_still_passes_gate(self):
        """확인 답이라도 게이트는 그대로 밟는다 — nav 미준비면 출발하지 않는다."""
        logic = MissionLogic()
        logic.on_intent(make_intent(need_confirm=True), make_dest(), BOUNDS, True, 0.0)
        actions = logic.on_confirm_answer(True, make_dest(), BOUNDS, False, 3.0)
        assert logic.state != State.NAVIGATING
        assert not any(isinstance(a, Navigate) for a in actions)

    def test_stale_confirm_different_dest_rejected(self):
        logic = MissionLogic()
        logic.on_intent(
            make_intent(need_confirm=True, matched_destination_id="restroom"),
            make_dest(id="restroom"), BOUNDS, True, 0.0,
        )
        actions = logic.on_intent(make_intent(matched_destination_id="room_407"),
                                  make_dest(), BOUNDS, True, 5.0)
        assert logic.state == State.IDLE
        # 9/1 감량: 멘트 없이 접는다 — 출발만 안 하면 된다.
        assert not any(isinstance(a, Say) for a in actions)
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
        # 재시도 안내는 9/1 감량(침묵 재출발)이라, 재시도를 꺼서 최종 실패
        # 안내(MSG_NAV_FAILED)를 바로 받는다 — 이 시험의 본래 관심사다.
        logic = MissionLogic(nav_retry_limit=0)
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
        # auto_return_home 을 켠 경우다. 기본값은 꺼짐이며 그때는 제자리에
        # 선다 - test_default_does_not_drive_home_after_approach 참고.
        logic = MissionLogic(return_destination=make_home(), auto_return_home=True)
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        actions = logic.on_approach_answer(False, 6.0)
        assert logic.state == State.RETURNING
        navigates = [a for a in actions if isinstance(a, Navigate)]
        assert len(navigates) == 1
        assert navigates[0].destination.id == "standby"

    def test_no_answer_stuck_fallback_returns(self):
        """tts_done 이 영영 안 오면(TTS 사망) 안전망 30초로 탈출한다.
        예전의 '큐 시각부터 8초'는 질문 음성(8.0초)과 겹쳐 답할 창이
        0초가 되는 결함이었다(2026-08-31 실기)."""
        logic = MissionLogic(
            return_destination=make_home(), approach_response_timeout_sec=8.0
        )
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        # 옛 폴백이라면 13.0 에 이탈했을 시각 — 아직 기다려야 한다.
        assert logic.on_tick(13.0, NavStatus.NONE) == []
        assert logic.state == State.AWAITING_USER
        assert logic.on_tick(34.9, NavStatus.NONE) == []
        actions = logic.on_tick(35.1, NavStatus.NONE)
        assert logic.state == State.RETURNING
        assert any(isinstance(a, Say) for a in actions)

    def test_question_as_long_as_window_still_gets_full_8s(self):
        """실기 재현: 질문 재생이 8.0초(응답 창과 같은 길이)여도 재생완료부터
        8초를 온전히 기다린다. 큐 시각 기준이면 여기서 창이 0초였다."""
        logic = MissionLogic(
            return_destination=make_home(), approach_response_timeout_sec=8.0
        )
        start_approach(logic)
        arrive_and_ask(logic, 0.0)
        logic.on_approach_question_spoken(8.2)  # 8.0초 wav + 지연
        assert logic.on_tick(8.3, NavStatus.NONE) == []
        assert logic.state == State.AWAITING_USER
        # 재생완료 + 8초 직전까지는 답을 기다린다
        assert logic.on_tick(16.1, NavStatus.NONE) == []
        assert logic.state == State.AWAITING_USER
        # 그 안에 온 답은 정상 수락된다
        actions = logic.on_approach_answer(True, 16.15)
        assert logic.state == State.TURNING
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
        logic = MissionLogic(return_destination=make_home(), auto_return_home=True)
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        logic.on_approach_answer(False, 6.0)
        logic.on_tick(20.0, NavStatus.SUCCEEDED)
        assert logic.state == State.IDLE
        assert logic.active_destination is None

    @pytest.mark.parametrize("status", [NavStatus.FAILED, NavStatus.CANCELED])
    def test_returning_finishes_even_if_it_fails(self, status):
        # 복귀에 실패해도 접근 상태에 갇히면 안 된다. 다음 요청을 받아야 한다.
        logic = MissionLogic(return_destination=make_home(), auto_return_home=True)
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        logic.on_approach_answer(False, 6.0)
        logic.on_tick(20.0, status)
        assert logic.state == State.IDLE

    def test_default_does_not_drive_home_after_approach(self):
        """기본값은 자동 홈 복귀가 꺼져 있다.

        홈 복귀는 2026-08-27 현재 실기 [미검증] 이다. 확인 전에 자동 주행부터
        켜면 아무도 안 보는 사이에 로봇이 처음 달려 보게 된다. 관리자가 앱에서
        부르는 복귀는 이 값과 무관하게 늘 동작하므로 실기 확인은 그쪽으로 한다.
        """
        logic = MissionLogic(return_destination=make_home())
        assert logic.auto_return_home is False
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        actions = logic.on_approach_answer(False, 6.0)
        # 홈이 지정돼 있어도 이번 복귀에는 주행이 없다.
        assert not any(isinstance(a, Navigate) for a in actions)
        assert logic.active_destination is None
        assert logic.state == State.RETURNING
        # 갈 곳이 없으므로 곧바로 끝난다.
        logic.on_tick(6.5, NavStatus.NONE)
        assert logic.state == State.IDLE

    def test_admin_return_home_works_even_when_auto_is_off(self):
        """관리자가 부르는 복귀는 auto_return_home 과 무관하게 동작한다."""
        logic = MissionLogic(return_destination=make_home())
        accepted, reason, actions = logic.on_return_home_request(True, 0.0)
        assert accepted is True
        assert reason is GateReason.OK
        navigates = [a for a in actions if isinstance(a, Navigate)]
        assert len(navigates) == 1
        assert navigates[0].destination.id == "standby"

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
        # 2026-09-01 감량: 엇갈린 confirm 은 멘트 없이 접는다 — 침묵이면
        # 사용자가 다시 말하고, 그 요청이 새 확인 흐름을 연다.
        assert not any(isinstance(a, Say) for a in actions)
        assert logic.state == State.IDLE
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

    def test_turn_success_onboards_without_done_ment(self):
        """회전 완료 멘트는 9/1 감량 — 바로 뒤 온보딩 질문이 완료를 대신한다."""
        logic = MissionLogic()
        self._accept_with_turn(logic)
        actions = logic.on_tick(7.0, NavStatus.SUCCEEDED)
        says = [a for a in actions if isinstance(a, Say)]
        assert [s.text for s in says] == [MSG_APPROACH_ONBOARDING]
        assert says[0].expects_reply is True         # 온보딩 끝 = 재청취 창
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


class TestSpeechFlushOnCancel:
    """취소·앱 선점은 하던 말부터 끊는다 (2026-09-01 큐 청소) — 상태는 즉시
    바뀌는데 낡은 멘트("방2 앞에 도착했습니다…")가 이어 나오면 헛소리가 된다."""

    def test_voice_cancel_flushes_first(self):
        logic = MissionLogic()
        start_navigation(logic)
        actions, _ = logic.on_cancel_request(1.0)
        assert actions and isinstance(actions[0], StopSpeech)

    def test_app_cancel_flushes_first(self):
        logic = MissionLogic()
        start_navigation(logic)
        actions, _ = logic.on_app_cancel(1.0)
        assert actions and isinstance(actions[0], StopSpeech)

    def test_app_preempt_flushes_first(self):
        logic = MissionLogic()
        start_navigation(logic)
        actions, _ = logic.on_app_destination(
            make_dest(id="restroom"), BOUNDS, True, 1.0)
        assert actions and isinstance(actions[0], StopSpeech)
        assert any(isinstance(a, Navigate) for a in actions)


class TestWakeFoldsStaleQuestions:
    """"비카야" = 새 대화 (2026-09-01) — 답-대기 상태를 조용히 접는다.
    답 자리가 살아 있으면 새 대화의 첫 마디가 옛 질문의 답으로 오인 접수된다."""

    def test_wake_folds_confirming_and_stray_affirm_is_dead(self):
        logic = MissionLogic()
        logic.on_intent(make_intent(need_confirm=True), make_dest(), BOUNDS,
                        True, 0.0)
        assert logic.on_wake(1.0) == []          # 멘트 없이 접는다
        assert logic.state == State.IDLE
        # 접힌 확인 질문의 답("그래" 오전사)은 아무것도 못 움직인다
        assert logic.on_confirm_answer(True, make_dest(), BOUNDS, True, 2.0) == []
        assert logic.state == State.IDLE

    def test_wake_folds_approach_question_and_stays_put(self):
        logic = MissionLogic(return_destination=make_home())
        start_approach(logic)
        arrive_and_ask(logic, 5.0)
        assert logic.on_wake(6.0) == []
        assert logic.state == State.IDLE         # 물러나지 않고 제자리
        assert logic.on_approach_answer(True, 7.0) == []


class TestWakeConsumedGuardsWakeDoa:
    """wake·wake_doa 는 같은 콜백 그룹이라 wake 가 먼저 상태를
    IDLE 로 내린다. 그 직후의 wake_doa 를 state == IDLE 만으로 통과시키면
    "옛 대화를 접었을 뿐"인 wake 를 새 호출로 오인해 SEEKING 이 열린다 —
    사람이 핸들을 잡고 로봇 뒤에 서 있을 때 최대 180도 제자리 회전이 터지는
    사고. wake/on_return_brake 가 상태를 바꾼 시각을 함께 봐야 두 토픽의
    도착 순서와 무관하게 결과가 같아진다."""

    def test_wake_doa_right_after_waiting_wake_does_not_open_seeking(self):
        logic = MissionLogic()
        logic.state = State.WAITING
        logic.on_wake(10.0)
        assert logic.state == State.IDLE
        # 같은 호출의 wake_doa 가 수 ms 뒤 도착했다고 가정한다.
        actions = logic.on_wake_doa(90.0, True, 10.001)
        assert not any(isinstance(a, SpinInPlace) for a in actions)
        assert logic.state == State.IDLE

    def test_wake_doa_right_after_asking_next_wake_does_not_open_seeking(self):
        logic = MissionLogic(arrival_dialog=True)
        logic.on_intent(make_intent(), make_dest(), BOUNDS, True, 0.0)
        logic.on_tick(1.0, NavStatus.SUCCEEDED)
        assert logic.state == State.ASKING_NEXT
        logic.on_wake(1.001)
        assert logic.state == State.IDLE
        actions = logic.on_wake_doa(90.0, True, 1.002)
        assert not any(isinstance(a, SpinInPlace) for a in actions)
        assert logic.state == State.IDLE

    def test_old_on_wake_behavior_is_unchanged(self):
        """on_wake 자체(답-대기 상태를 IDLE 로 접기)는 그대로다 — 막히는 것은
        그 직후의 wake_doa 뿐이다."""
        logic = MissionLogic()
        logic.state = State.WAITING
        assert logic.on_wake(1.0) == []
        assert logic.state == State.IDLE

    def test_state_order_first_is_rejected_by_state_gate(self):
        """wake_doa 가 먼저 오면(아직 옛 상태) 기존 state 관문이 거절한다 —
        시각 관문과 무관한 경로도 여전히 막힌다."""
        logic = MissionLogic()
        logic.state = State.WAITING
        assert logic.on_wake_doa(90.0, True, 10.0) == []
        assert logic.state == State.WAITING

    def test_guard_expires_and_a_real_new_call_opens_seeking(self):
        """2초가 지난 뒤는 진짜 새 호출이다 — 과도한 봉쇄가 아니다."""
        logic = MissionLogic()
        logic.state = State.WAITING
        logic.on_wake(1.0)
        assert logic.state == State.IDLE
        actions = logic.on_wake_doa(
            90.0, True, 1.0 + WAKE_CONSUMED_GUARD_SEC + 0.01)
        assert logic.state == State.SEEKING
        assert any(isinstance(a, SpinInPlace) for a in actions)


class TestUserAttachedSuppressesWakeDoa:
    """접근 회전이 끝나 사용자가 손잡이를 받아든 직후도 같은 사고 조건이다
    (2026-09-10 사용자 결정) — 이 전이는 wake 가 아니라 회전 완료가
    일으킨 것이라 _wake_consumed_at 도장이 안 찍힌다. 재청취 창이 만료된
    뒤 "비카야, 화장실"처럼 부르면 DOA≈180(핸들 쪽)이 그대로 SEEKING 을
    열어, 손잡이를 잡고 로봇 옆에 선 사용자 앞에서 최대 180도 제자리
    회전이 다시 터진다."""

    def _accept_and_finish_turn(self, logic, t_answer=1.0, t_done=2.0,
                                 nav_status=NavStatus.SUCCEEDED):
        logic.on_approach_answer(True, t_answer)
        assert logic.state == State.TURNING
        logic.on_tick(t_done, nav_status)
        assert logic.state == State.IDLE

    def test_wake_doa_right_after_turn_done_is_rejected(self):
        logic = MissionLogic(return_destination=make_home())
        start_approach(logic)
        arrive_and_ask(logic, t=1.0)
        self._accept_and_finish_turn(logic, t_answer=1.0, t_done=2.0)
        actions = logic.on_wake_doa(180.0, True, 2.001)
        assert not any(isinstance(a, SpinInPlace) for a in actions)
        assert logic.state == State.IDLE

    def test_wake_doa_rejected_even_when_turn_failed(self):
        """회전이 실패해도 사용자는 이미 승낙하고 그 자리에 있다."""
        logic = MissionLogic(return_destination=make_home())
        start_approach(logic)
        arrive_and_ask(logic, t=1.0)
        self._accept_and_finish_turn(logic, t_answer=1.0, t_done=2.0,
                                      nav_status=NavStatus.FAILED)
        actions = logic.on_wake_doa(180.0, True, 2.001)
        assert not any(isinstance(a, SpinInPlace) for a in actions)

    def test_wake_doa_opens_after_silent_suppress_window(self):
        """아무도 말을 걸지 않은 채 60초가 다 지나면 다른 사람의 호출을
        다시 받는다 — 되감기가 없었던 경우."""
        logic = MissionLogic(return_destination=make_home())
        start_approach(logic)
        arrive_and_ask(logic, t=1.0)
        self._accept_and_finish_turn(logic, t_answer=1.0, t_done=2.0)
        actions = logic.on_wake_doa(
            180.0, True, 2.0 + USER_ATTACHED_SUPPRESS_SEC + 0.01)
        assert logic.state == State.SEEKING
        assert any(isinstance(a, SpinInPlace) for a in actions)

    def test_wake_rewinds_the_suppress_window(self):
        """붙어 있는 사용자가 만료 직전에 다시 말을 걸면 시계가 되감긴다 —
        원래 만료 시각을 지나도 여전히 막혀야 한다."""
        logic = MissionLogic(return_destination=make_home())
        start_approach(logic)
        arrive_and_ask(logic, t=1.0)
        self._accept_and_finish_turn(logic, t_answer=1.0, t_done=2.0)
        original_expiry = logic._user_attached_until
        assert original_expiry == pytest.approx(2.0 + USER_ATTACHED_SUPPRESS_SEC)
        logic.on_wake(original_expiry - 1.0)          # 만료 직전에 다시 말함
        actions = logic.on_wake_doa(180.0, True, original_expiry + 1.0)
        assert not any(isinstance(a, SpinInPlace) for a in actions)
        assert logic.state == State.IDLE

    def test_decline_path_is_not_suppressed(self):
        """거절 경로는 RETURNING 으로 빠지므로 억제를 걸지 않는다 — 찾기
        모드는 원래 IDLE 에서만 열리니 복귀가 끝나기 전까지는 자동으로
        막힌다. 복귀가 끝나 IDLE 이 되면 다른 사람의 호출은 그대로 들어야
        한다(2026-09-10 사용자 결정)."""
        logic = MissionLogic()   # 홈 미지정 — 제자리에서 복귀가 곧바로 끝난다
        start_approach(logic)
        arrive_and_ask(logic, t=1.0)
        logic.on_approach_answer(False, 1.0)
        assert logic.state == State.RETURNING
        logic.on_tick(1.1, NavStatus.NONE)
        assert logic.state == State.IDLE
        actions = logic.on_wake_doa(180.0, True, 1.11)
        assert logic.state == State.SEEKING
        assert any(isinstance(a, SpinInPlace) for a in actions)

    def test_zero_yaw_shortcut_still_suppresses(self):
        """회전량을 0으로 꺼도(예: 좁은 곳) 승낙한 사용자는 그 자리에 있다 —
        State.TURNING 을 거치지 않는 지름길에도 같은 억제가 걸려야 한다."""
        logic = MissionLogic(return_destination=make_home(),
                              approach_turn_yaw_rad=0.0)
        start_approach(logic)
        arrive_and_ask(logic, t=1.0)
        logic.on_approach_answer(True, 1.0)
        assert logic.state == State.IDLE   # 회전 없이 바로 온보딩
        actions = logic.on_wake_doa(180.0, True, 1.001)
        assert not any(isinstance(a, SpinInPlace) for a in actions)


class TestReturnBrakeGuardsWakeDoa:
    """on_return_brake 도 _wake_consumed_at 도장을 찍는다 — 복귀 중
    "비카야"로 브레이크를 밟은 직후 같은 콜백 그룹 경합으로 wake_doa 가
    뒤따라오면, 도장이 없으면 그 소비를 새 호출로 오인해 SEEKING 이 열린다."""

    def test_wake_doa_right_after_return_brake_does_not_open_seeking(self):
        logic = MissionLogic()
        logic.state = State.RETURNING
        logic.on_return_brake(10.0)
        assert logic.state == State.IDLE
        actions = logic.on_wake_doa(90.0, True, 10.001)
        assert not any(isinstance(a, SpinInPlace) for a in actions)
        assert logic.state == State.IDLE


class TestReturnResumeAfterCallInterrupt:
    """복귀 중 호출로 끊긴 뒤 무기한 정지하지 않고 결국 복귀를 재개한다
    (2026-09-10 사용자 승인 흐름). 기준 시각은 항상 on_return_brake 가 불린
    순간이다 — 청취 창(음성 쪽, 약 8초)의 길이는 이 모듈이 모른다."""

    def test_silence_for_15s_gives_leaving_notice(self):
        logic = MissionLogic(return_destination=make_home())
        logic.state = State.RETURNING
        logic.active_destination = make_home()
        logic.on_return_brake(0.0)
        assert logic.state == State.IDLE
        # 15초 직전까지는 조용하다.
        assert logic.on_tick(RETURN_RESUME_SEC - 0.1, NavStatus.NONE) == []
        assert logic.state == State.IDLE
        # 딱 15초에 떠나기 예고 — 아직 복귀를 재개하지는 않는다.
        actions = logic.on_tick(RETURN_RESUME_SEC, NavStatus.NONE)
        assert any(
            isinstance(a, Say) and a.text == MSG_LEAVING_NOTICE for a in actions
        )
        assert logic.state == State.IDLE

    def test_resume_after_notice_grace_elapses(self):
        logic = MissionLogic(return_destination=make_home())
        logic.state = State.RETURNING
        logic.active_destination = make_home()
        logic.on_return_brake(0.0)
        logic.on_tick(RETURN_RESUME_SEC, NavStatus.NONE)   # 예고
        # 유예 직전까지는 아직 제자리다.
        assert logic.on_tick(
            RETURN_RESUME_SEC + LEAVING_GRACE_SEC - 0.1, NavStatus.NONE
        ) == []
        assert logic.state == State.IDLE
        # 유예가 다 되면 복귀를 재개한다.
        actions = logic.on_tick(
            RETURN_RESUME_SEC + LEAVING_GRACE_SEC, NavStatus.NONE
        )
        assert logic.state == State.RETURNING
        assert any(isinstance(a, Navigate) for a in actions)
        assert logic._return_interrupted is False

    def test_answer_within_grace_cancels_the_resume(self):
        """예고 뒤 유예 안에 목적지를 말하면 복귀로 새지 않는다."""
        logic = MissionLogic(return_destination=make_home())
        logic.state = State.RETURNING
        logic.active_destination = make_home()
        logic.on_return_brake(0.0)
        logic.on_tick(RETURN_RESUME_SEC, NavStatus.NONE)   # 예고, 유예 시작
        answer_t = RETURN_RESUME_SEC + 1.0   # 유예(3초) 안
        actions = logic.on_intent(make_intent(), make_dest(), BOUNDS, True, answer_t)
        assert logic.state == State.NAVIGATING
        assert any(isinstance(a, Navigate) for a in actions)
        assert logic._return_interrupted is False
        # 유예가 다 지나도 복귀로 새지 않는다 — 이미 새 안내 중이다.
        later = logic.on_tick(
            RETURN_RESUME_SEC + LEAVING_GRACE_SEC + 5.0, NavStatus.RUNNING
        )
        assert logic.state == State.NAVIGATING
        assert not any(
            isinstance(a, Say) and a.text == MSG_LEAVING_NOTICE for a in later
        )

    def test_answer_within_15s_skips_notice_and_resume_entirely(self):
        """15초 안에 목적지를 말하면 예고도 복귀도 없다."""
        logic = MissionLogic(return_destination=make_home())
        logic.state = State.RETURNING
        logic.active_destination = make_home()
        logic.on_return_brake(0.0)
        actions = logic.on_intent(make_intent(), make_dest(), BOUNDS, True, 5.0)
        assert logic.state == State.NAVIGATING
        assert any(isinstance(a, Navigate) for a in actions)
        assert logic._return_interrupted is False
        later = logic.on_tick(
            RETURN_RESUME_SEC + LEAVING_GRACE_SEC + 5.0, NavStatus.RUNNING
        )
        assert logic.state == State.NAVIGATING
        assert not any(
            isinstance(a, Say) and a.text == MSG_LEAVING_NOTICE for a in later
        )

    def test_call_during_return_wait_is_rejected_and_ladder_still_resumes(self):
        """2026-09-10 사용자 결정: 복귀 재개 사다리가 도는 동안은 회전 자체를
        거절한다 — 오탐 한 번이 회전 왕복(최대 16초)과 사다리 재대기
        (18초)를 더해 30초 넘게 로봇을 통행로에 붙잡을 수 있어서다. 두
        번째 호출이 방향을 담고 와도 SEEKING 이 열리지 않고, 사다리는 첫
        호출(on_return_brake) 시각 기준으로 그대로 돌아 복귀로 끝난다.
        (이 시험은 원래 "회전이 끼어들어도 결국 재개된다"를 봤다 — 그
        회전 자체가 없어졌으므로 지금은 "회전이 거절된다"를 보도록
        뜻을 바꿨다.)"""
        logic = MissionLogic(return_destination=make_home())
        logic.state = State.RETURNING
        logic.active_destination = make_home()
        logic.on_return_brake(0.0)
        assert logic._return_interrupted is True

        # 두 번째 "비카야"(방향 포함) — WAKE_CONSUMED_GUARD_SEC(3초) 지난
        # 뒤라 방금 브레이크 소비의 여진이 아니라 진짜 새 호출인데도 거절된다.
        actions = logic.on_wake_doa(90.0, True, 4.0)
        assert actions == []
        assert not any(isinstance(a, SpinInPlace) for a in actions)
        assert logic.state == State.IDLE
        assert logic._seek_deadline is None
        assert logic._return_resume_deadline == pytest.approx(RETURN_RESUME_SEC)

        # 사다리는 회전 없이, 첫 호출 시각 기준으로 그대로 돈다.
        actions = logic.on_tick(RETURN_RESUME_SEC, NavStatus.NONE)
        assert any(
            isinstance(a, Say) and a.text == MSG_LEAVING_NOTICE for a in actions
        )
        actions = logic.on_tick(RETURN_RESUME_SEC + LEAVING_GRACE_SEC, NavStatus.NONE)
        assert logic.state == State.RETURNING
        assert logic._return_interrupted is False
        assert any(isinstance(a, Navigate) for a in actions)

    def test_frontal_call_during_return_wait_is_also_rejected(self):
        """회전 거절을 넣기 전에는 on_wake_doa 의 무회전 분기(정면 ±10도,
        SEEK_MIN_YAW_RAD 미만)가 _to_idle() 을 거치지 않고 _seek_deadline
        만 직접 열어, 탐색 창이 열려 있는 도중에 복귀 사다리가 예고를
        말하고 떠나는 결함이 있었다(재현: on_return_brake(0.0) 뒤 t=10
        정면 호출로 _seek_deadline=18, t=15 IDLE tick 이 예고 발화, t=18
        창 종료와 _go_home 이 같은 tick 에 겹침). _return_interrupted 동안
        on_wake_doa 자체를 거절하는 것으로 이 분기도 뿌리에서 함께
        막힌다 — 탐색 창이 아예 안 열리고, 두 시계는 절대 같은 IDLE
        위에서 만나지 않는다(SEEK_LOOK_SEC 주석 참고)."""
        logic = MissionLogic(return_destination=make_home())
        logic.state = State.RETURNING
        logic.active_destination = make_home()
        logic.on_return_brake(0.0)

        # 정면 호출(회전 없이 탐색 창만 여는 분기) — 역시 거절된다.
        actions = logic.on_wake_doa(3.0, True, 10.0)
        assert actions == []
        assert logic._seek_deadline is None
        assert logic.state == State.IDLE

        # 사다리는 겹침 없이 정상 진행한다.
        actions = logic.on_tick(RETURN_RESUME_SEC, NavStatus.NONE)
        assert any(
            isinstance(a, Say) and a.text == MSG_LEAVING_NOTICE for a in actions
        )
        actions = logic.on_tick(RETURN_RESUME_SEC + LEAVING_GRACE_SEC, NavStatus.NONE)
        assert logic.state == State.RETURNING

    def test_app_cancel_during_return_wait_clears_the_ladder(self):
        """2026-09-10 사용자 결정: 관리자 취소가 IDLE 에서 GateReason.OK
        만 돌려주고 조용히 수락하던 기존 동작 때문에, 사다리가 그대로
        돌아 18초 뒤 로봇이 취소를 무시한 것처럼 출발하던 결함. 지금은
        취소가 사다리도 함께 청산한다 — 사람이 명시적으로 내린 지시이니,
        그 뒤 로봇이 제자리에 계속 서 있는 것이 맞다."""
        logic = MissionLogic(return_destination=make_home())
        logic.state = State.RETURNING
        logic.active_destination = make_home()
        logic.on_return_brake(0.0)
        assert logic._return_interrupted is True

        actions, reason = logic.on_app_cancel(5.0)
        assert reason == GateReason.OK
        assert logic._return_interrupted is False
        assert logic._return_resume_deadline is None

        # 유예가 다 지나도 복귀로 새지 않는다 — 취소가 끝이다.
        later = logic.on_tick(
            RETURN_RESUME_SEC + LEAVING_GRACE_SEC + 5.0, NavStatus.NONE
        )
        assert logic.state == State.IDLE
        assert not any(
            isinstance(a, Say) and a.text == MSG_LEAVING_NOTICE for a in later
        )

    def test_app_destination_during_return_wait_clears_the_ladder(self):
        """on_app_destination 의 청산은 on_intent 와 같은 패턴
        (_force_clear_all 에 모음)이라 위험은 낮았지만 코드 검토로만
        확인돼 있었다 — 한 줄 시험으로 실제 동작을 확인한다."""
        logic = MissionLogic(return_destination=make_home())
        logic.state = State.RETURNING
        logic.active_destination = make_home()
        logic.on_return_brake(0.0)

        actions, reason = logic.on_app_destination(make_dest(), BOUNDS, True, 5.0)
        assert reason == GateReason.OK
        assert logic.state == State.NAVIGATING
        assert logic._return_interrupted is False

    def test_plain_idle_never_starts_the_timer(self):
        """복귀 중이 아니었던 평범한 IDLE 에서는 이 사다리가 아예 안 걸린다."""
        logic = MissionLogic(return_destination=make_home())
        assert logic.state == State.IDLE
        actions = logic.on_tick(1000.0, NavStatus.NONE)
        assert actions == []
        assert logic._return_interrupted is False
        assert logic._return_resume_deadline is None
        assert logic.state == State.IDLE


class TestAwaitingUserWakeGuardsWakeDoa:
    """사람 1.1 m 앞에서 질문 대기 중(AWAITING_USER)의 "비카야"도 on_wake 가
    _to_idle() 뒤 _wake_consumed_at 도장을 찍는 자리다 — 뒤따라온 wake_doa 가
    이 소비를 새 호출로 오인하면 방금 접근한 사람 앞에서 SEEKING 이 열린다."""

    def test_wake_doa_right_after_awaiting_user_wake_does_not_open_seeking(self):
        logic = MissionLogic()
        logic.state = State.AWAITING_USER
        logic.approach_track_id = 7
        logic.on_wake(10.0)
        assert logic.state == State.IDLE
        actions = logic.on_wake_doa(90.0, True, 10.001)
        assert not any(isinstance(a, SpinInPlace) for a in actions)
        assert logic.state == State.IDLE


class TestConfirmReproposalIsAnswer:
    """확인 중 같은 목적지의 재제안(confirm=True)은 답이다 (2026-09-01) —
    LLM 이 "응 화장실로 가자"를 재제안으로 되돌려도 출발해야 한다."""

    def test_same_dest_reproposal_starts_navigation(self):
        logic = MissionLogic()
        logic.on_intent(make_intent(need_confirm=True), make_dest(), BOUNDS,
                        True, 0.0)
        assert logic.state == State.CONFIRMING
        actions = logic.on_intent(make_intent(need_confirm=True), make_dest(),
                                  BOUNDS, True, 3.0)
        assert logic.state == State.NAVIGATING
        assert any(isinstance(a, Navigate) for a in actions)

    def test_different_dest_reproposal_switches_confirming(self):
        logic = MissionLogic()
        logic.on_intent(make_intent(need_confirm=True), make_dest(), BOUNDS,
                        True, 0.0)
        actions = logic.on_intent(
            make_intent(need_confirm=True, matched_destination_id="restroom"),
            make_dest(id="restroom"), BOUNDS, True, 3.0)
        assert logic.state == State.CONFIRMING
        assert logic.confirming_dest_id == "restroom"
        assert not any(isinstance(a, Navigate) for a in actions)


class TestDoaToSpinYaw:
    """마이크 각도(0~359°, 정면 0) -> SpinInPlace 회전량(rad, 양수=반시계).

    sign 은 마이크 각도가 반시계로 커지면 +1, 시계로 커지면 -1 이다.
    이 부호를 틀리면 로봇이 정확히 반대로 돈다 (설계 §5, 실측으로 정한다).
    """

    def test_front_is_no_turn(self):
        assert doa_to_spin_yaw(0.0, 1.0) == pytest.approx(0.0)

    def test_ccw_mic_left_turns_left(self):
        assert doa_to_spin_yaw(90.0, 1.0) == pytest.approx(math.pi / 2)

    def test_cw_mic_left_turns_right(self):
        """부호가 반대면 같은 각도가 반대쪽 회전이 된다."""
        assert doa_to_spin_yaw(90.0, -1.0) == pytest.approx(-math.pi / 2)

    def test_takes_the_short_way_round(self):
        """270° 는 왼쪽으로 270° 가 아니라 오른쪽으로 90° 다."""
        assert doa_to_spin_yaw(270.0, 1.0) == pytest.approx(-math.pi / 2)

    def test_behind_is_half_turn(self):
        """뒤(핸들 쪽)는 어느 방향으로 돌든 180° 다."""
        assert abs(doa_to_spin_yaw(180.0, 1.0)) == pytest.approx(math.pi)

    def test_just_left_of_front(self):
        assert doa_to_spin_yaw(359.0, 1.0) == pytest.approx(math.radians(-1.0))


class TestWrapToPi:
    def test_leaves_small_angles_alone(self):
        assert wrap_to_pi(1.0) == pytest.approx(1.0)

    def test_wraps_over_half_turn(self):
        assert wrap_to_pi(math.radians(270.0)) == pytest.approx(math.radians(-90.0))

    def test_wraps_negative(self):
        assert wrap_to_pi(math.radians(-270.0)) == pytest.approx(math.radians(90.0))


class TestSeekEntry:
    """"비카야" 방향으로 고개 돌리기 — 대기 중에만 연다."""

    def test_wake_doa_turns_toward_the_sound(self):
        logic = MissionLogic(wake_doa_sign=1.0)
        actions = logic.on_wake_doa(90.0, True, 1.0)
        assert logic.state == State.SEEKING
        spins = [a for a in actions if isinstance(a, SpinInPlace)]
        assert len(spins) == 1
        assert spins[0].yaw_rad == pytest.approx(math.pi / 2)

    def test_no_new_ment(self):
        """호출 응답 "네?"는 음성이 이미 했다. 미션은 말하지 않는다."""
        logic = MissionLogic()
        actions = logic.on_wake_doa(90.0, True, 1.0)
        assert not any(isinstance(a, Say) for a in actions)

    def test_remembers_how_to_get_back(self):
        logic = MissionLogic(wake_doa_sign=1.0)
        logic.on_wake_doa(90.0, True, 1.0)
        assert logic._seek_return_yaw == pytest.approx(-math.pi / 2)

    def test_sound_from_the_front_does_not_spin(self):
        """이미 그쪽을 보고 있다 — 돌지 않고 찾기만 한다."""
        logic = MissionLogic(wake_doa_sign=1.0)
        actions = logic.on_wake_doa(3.0, True, 1.0)
        assert logic.state == State.IDLE
        assert not any(isinstance(a, SpinInPlace) for a in actions)
        assert logic._seek_deadline == pytest.approx(1.0 + SEEK_LOOK_SEC)

    def test_ignored_while_guiding(self):
        """안내 중 "비카야"는 기존 사용자의 명령이다 — 고개를 돌리면 안 된다."""
        logic = MissionLogic()
        logic.state = State.NAVIGATING
        assert logic.on_wake_doa(90.0, True, 1.0) == []
        assert logic.state == State.NAVIGATING

    def test_ignored_while_estopped(self):
        logic = MissionLogic()
        logic.estop_active = True
        assert logic.on_wake_doa(90.0, True, 1.0) == []
        assert logic.state == State.IDLE

    def test_ignored_when_nav_not_ready(self):
        logic = MissionLogic()
        assert logic.on_wake_doa(90.0, False, 1.0) == []
        assert logic.state == State.IDLE

    def test_seeking_holds_a_live_goal(self):
        """E-stop 이 회전을 취소할 수 있어야 한다."""
        logic = MissionLogic()
        logic.on_wake_doa(90.0, True, 1.0)
        actions = logic.on_estop(True, 2.0)
        assert any(isinstance(a, CancelNav) for a in actions)

    def test_destination_request_rejected_while_seeking(self):
        """회전 중 목적지 요청을 받아 버리면 SpinInPlace 를 취소하지 않은 채
        Navigate 가 나가 두 goal 이 동시에 발행된다 (설계 4절, 2026-09-10).
        TURNING 과 같은 처리 — 새 멘트 없이 기존 MSG_APPROACH_BUSY 를 쓴다."""
        logic = MissionLogic()
        logic.on_wake_doa(90.0, True, 1.0)
        actions = logic.on_intent(make_intent(), make_dest(), BOUNDS, True, 2.0)
        assert logic.state == State.SEEKING
        assert not any(isinstance(a, Navigate) for a in actions)
        says = [a for a in actions if isinstance(a, Say)]
        assert len(says) == 1

    def test_app_preempt_cancels_the_spin_while_seeking(self):
        """음성 경로(위 시험)는 SEEKING 을 거부하는데, 앱 선점 경로
        (_force_clear_all)는 예전엔 등록 목적지 상태만 CancelNav 를 내 SEEKING 의
        spin 을 못 끊었다 — Navigate 가 그 위에 그대로 나가 두 goal 이
        동시에 /cmd_vel_req 에 붙는 사고였다."""
        logic = MissionLogic()
        logic.on_wake_doa(90.0, True, 1.0)
        assert logic.state == State.SEEKING
        actions, reason = logic.on_app_destination(make_dest(), BOUNDS, True, 2.0)
        assert reason == GateReason.OK
        assert any(isinstance(a, CancelNav) for a in actions)
        assert any(isinstance(a, Navigate) for a in actions)
        assert logic.state == State.NAVIGATING


def seek_and_finish_turn(logic, doa=90.0, t0=1.0):
    """호출 -> 회전 -> 회전 완료. 탐색 창이 열린 IDLE 을 만든다."""
    logic.on_wake_doa(doa, True, t0)
    logic.on_tick(t0 + 1.0, NavStatus.SUCCEEDED)
    return logic


class TestSeekLookWindow:
    def test_turn_done_returns_to_idle_with_a_window(self):
        """IDLE 로 내려오는 것이 요점이다 — 접근 관문은 IDLE 만 통과시킨다."""
        logic = MissionLogic()
        seek_and_finish_turn(logic, t0=1.0)
        assert logic.state == State.IDLE
        assert logic._seek_deadline == pytest.approx(2.0 + SEEK_LOOK_SEC)

    def test_person_found_cancels_the_way_back(self):
        """사람에게 갔으면 되돌아가지 않는다."""
        logic = MissionLogic()
        seek_and_finish_turn(logic, t0=1.0)
        actions, reason = logic.on_approach_request(
            make_approach(), BOUNDS, True, 3.0)
        assert reason == GateReason.OK
        assert logic.state == State.APPROACHING
        # 창이 만료될 시각을 지나도 복귀 회전이 없다.
        later = logic.on_tick(30.0, NavStatus.RUNNING)
        assert not any(isinstance(a, SpinInPlace) for a in later)

    def test_nobody_found_turns_back_quietly(self):
        logic = MissionLogic(wake_doa_sign=1.0)
        seek_and_finish_turn(logic, doa=90.0, t0=1.0)
        actions = logic.on_tick(2.0 + SEEK_LOOK_SEC, NavStatus.NONE)
        spins = [a for a in actions if isinstance(a, SpinInPlace)]
        assert len(spins) == 1
        assert spins[0].yaw_rad == pytest.approx(-math.pi / 2)
        assert logic.state == State.SEEKING
        # 조용히. 복도 소음 오인에 로봇이 말을 걸면 주변을 놀래킨다.
        assert not any(isinstance(a, Say) for a in actions)

    def test_back_home_ends_in_idle(self):
        logic = MissionLogic()
        seek_and_finish_turn(logic, t0=1.0)
        logic.on_tick(2.0 + SEEK_LOOK_SEC, NavStatus.NONE)   # 복귀 회전 시작
        logic.on_tick(20.0, NavStatus.SUCCEEDED)             # 복귀 회전 완료
        assert logic.state == State.IDLE
        assert logic._seek_deadline is None
        assert logic._seek_return_yaw is None

    def test_calling_again_accumulates_the_way_back(self):
        """두 번 부르면 두 번 돈다 — 두 번째 SpinInPlace 발행 자체를 단언한다
        (상태·필드만 보면 액션이 안 나가도 통과해버린다, 2026-09-10 재검토).
        복귀각은 덮어쓰지 않고 더한다."""
        logic = MissionLogic(wake_doa_sign=1.0)
        seek_and_finish_turn(logic, doa=90.0, t0=1.0)        # +90도
        actions = logic.on_wake_doa(90.0, True, 3.0)         # 또 +90도
        spins = [a for a in actions if isinstance(a, SpinInPlace)]
        assert len(spins) == 1
        assert spins[0].yaw_rad == pytest.approx(math.pi / 2)
        assert logic.state == State.SEEKING
        assert logic._seek_return_yaw == pytest.approx(-math.pi)

    def test_a_new_errand_wins(self):
        """탐색 창 중에 할 일이 생기면 제자리 돌기를 시작하지 않는다."""
        logic = MissionLogic()
        seek_and_finish_turn(logic, t0=1.0)
        logic.on_intent(make_intent(), make_dest(), BOUNDS, True, 3.0)
        assert logic.state == State.NAVIGATING
        actions = logic.on_tick(2.0 + SEEK_LOOK_SEC, NavStatus.RUNNING)
        assert not any(isinstance(a, SpinInPlace) for a in actions)

    def test_new_errand_clears_the_seek_window(self):
        """낡은 창이 안내 한 판을 살아남으면 안 된다 — test_a_new_errand_wins
        는 그 시점 상태가 NAVIGATING 이라 창이 안 비워져도 통과해버린다. 이 창이
        남으면 이번 안내가 끝나고 한참 뒤 IDLE 에서 낡은 복귀각으로 갑자기 돈다."""
        logic = MissionLogic()
        seek_and_finish_turn(logic, t0=1.0)
        assert logic._seek_deadline is not None
        logic.on_intent(make_intent(), make_dest(), BOUNDS, True, 3.0)
        assert logic._seek_deadline is None
        assert logic._seek_return_yaw is None

    def test_spin_that_never_started_escapes_by_clock(self):
        logic = MissionLogic()
        logic.on_wake_doa(90.0, True, 1.0)
        logic.on_tick(1.0 + SEEK_TURN_TIMEOUT_SEC, NavStatus.NONE)
        assert logic.state == State.IDLE

    def test_failed_turn_still_looks(self):
        """회전이 거부돼도 찾아는 본다 — 카메라가 이미 사람을 볼 수도 있다."""
        logic = MissionLogic()
        logic.on_wake_doa(90.0, True, 1.0)
        logic.on_tick(2.0, NavStatus.FAILED)
        assert logic.state == State.IDLE
        assert logic._seek_deadline is not None

    def test_small_accumulated_back_does_not_spin(self):
        """설계 5요점: 복귀 회전도 10도 미만이면 생략한다 — 정면 근처 호출이
        반복돼 잔여각이 작게 남아도 창 만료에 SpinInPlace 가 나가면 안 된다.
        회귀하면 사람 옆에서 5도짜리 잔여 회전이 튀어나온다."""
        logic = MissionLogic(wake_doa_sign=1.0)
        logic.on_wake_doa(3.0, True, 1.0)     # 정면 근처(10도 미만): 회전 없음
        assert logic.state == State.IDLE
        logic.on_wake_doa(3.0, True, 2.0)     # 다시 정면 근처: 누적 back -6도
        assert logic.state == State.IDLE
        assert abs(logic._seek_return_yaw) < SEEK_MIN_YAW_RAD
        actions = logic.on_tick(2.0 + SEEK_LOOK_SEC, NavStatus.NONE)
        assert not any(isinstance(a, SpinInPlace) for a in actions)
        assert logic.state == State.IDLE
        assert logic._seek_deadline is None
        assert logic._seek_return_yaw is None


class TestNearCallApproach:
    """부른 사람이 코앞(near_call_max_m 안)이면 접근 goal(1.1 m)이 이미 지나간
    자리다 — 걸어가지 않고 그 자리에서 바로 질문한다. detection_gate 가 신뢰도·
    추적·안정·정지 관문을 전부 통과시킨 뒤 거리 하나만으로 TOO_NEAR 거절한 결과
    (stable=true·approachable=false·distance_m)를 Mission 이 직접 받는다."""

    def test_near_person_skips_navigate_and_asks(self):
        logic = MissionLogic()
        seek_and_finish_turn(logic, t0=1.0)
        actions = logic.on_person_detection(
            track_id=7, distance_m=1.2, stable=True, approachable=False, now=3.0)
        assert logic.state == State.AWAITING_USER
        assert not any(isinstance(a, Navigate) for a in actions)
        says = [a for a in actions if isinstance(a, Say)]
        assert says and says[0].text == MSG_APPROACH_QUESTION
        assert says[0].expects_reply is True

    def test_near_person_accept_turns_then_onboards(self):
        """1.0~1.5 m: 걸어가지 않고 질문 -> 수락 시 180도 회전 -> 온보딩."""
        logic = MissionLogic()
        seek_and_finish_turn(logic, t0=1.0)
        logic.on_person_detection(
            track_id=7, distance_m=1.2, stable=True, approachable=False, now=3.0)
        actions = logic.on_approach_answer(True, 4.0)
        assert logic.state == State.TURNING
        spins = [a for a in actions if isinstance(a, SpinInPlace)]
        assert len(spins) == 1
        assert spins[0].yaw_rad == pytest.approx(math.pi)
        onboarding_actions = logic.on_tick(6.0, NavStatus.SUCCEEDED)
        assert logic.state == State.IDLE
        says = [a for a in onboarding_actions if isinstance(a, Say)]
        assert says and says[0].text == MSG_APPROACH_ONBOARDING

    def test_very_near_person_accept_skips_spin(self):
        """1.0 m 미만: 수락해도 회전 없이 바로 온보딩 (손잡이가 사람을 칠 위험,
        2026-09-10 사용자 결정)."""
        logic = MissionLogic()
        seek_and_finish_turn(logic, t0=1.0)
        logic.on_person_detection(
            track_id=7, distance_m=0.6, stable=True, approachable=False, now=3.0)
        actions = logic.on_approach_answer(True, 4.0)
        assert logic.state == State.IDLE
        assert not any(isinstance(a, SpinInPlace) for a in actions)
        says = [a for a in actions if isinstance(a, Say)]
        assert says and says[0].text == MSG_APPROACH_ONBOARDING

    def test_approachable_person_not_handled_here(self):
        """approachable=true 는 기존 접근 요청 service 경로가 처리한다 —
        이 새 경로는 관여하지 않는다."""
        logic = MissionLogic()
        seek_and_finish_turn(logic, t0=1.0)
        actions = logic.on_person_detection(
            track_id=7, distance_m=1.2, stable=True, approachable=True, now=3.0)
        assert actions == []
        assert logic.state == State.IDLE

    def test_ignored_outside_seek_window(self):
        """탐색 창 밖(그냥 IDLE)에서 같은 감지가 와도 아무 일도 없다."""
        logic = MissionLogic()
        assert logic.state == State.IDLE
        assert logic._seek_deadline is None
        actions = logic.on_person_detection(
            track_id=7, distance_m=1.2, stable=True, approachable=False, now=1.0)
        assert actions == []
        assert logic.state == State.IDLE

    def test_ignored_while_still_seeking(self):
        """회전이 아직 끝나지 않은 SEEKING 중에는 관여하지 않는다."""
        logic = MissionLogic()
        logic.on_wake_doa(90.0, True, 1.0)
        assert logic.state == State.SEEKING
        actions = logic.on_person_detection(
            track_id=7, distance_m=1.2, stable=True, approachable=False, now=1.5)
        assert actions == []
        assert logic.state == State.SEEKING

    def test_no_return_spin_once_conversation_starts(self):
        """새 경로로 대화가 시작되면 복귀 회전이 발행되지 않는다."""
        logic = MissionLogic(wake_doa_sign=1.0)
        seek_and_finish_turn(logic, doa=90.0, t0=1.0)
        assert logic._seek_deadline is not None
        logic.on_person_detection(
            track_id=7, distance_m=1.2, stable=True, approachable=False, now=3.0)
        assert logic._seek_deadline is None
        assert logic._seek_return_yaw is None
        later = logic.on_tick(2.0 + SEEK_LOOK_SEC, NavStatus.NONE)
        assert not any(isinstance(a, SpinInPlace) for a in later)

    def test_nan_distance_ignored(self):
        logic = MissionLogic()
        seek_and_finish_turn(logic, t0=1.0)
        actions = logic.on_person_detection(
            track_id=7, distance_m=float("nan"), stable=True, approachable=False,
            now=3.0)
        assert actions == []
        assert logic.state == State.IDLE

    def test_suppressed_track_ignored(self):
        logic = MissionLogic()
        seek_and_finish_turn(logic, t0=1.0)
        logic._suppress_track(7, 2.0)
        actions = logic.on_person_detection(
            track_id=7, distance_m=1.2, stable=True, approachable=False, now=3.0)
        assert actions == []
        assert logic.state == State.IDLE

    def test_too_far_for_near_call_ignored(self):
        """near_call_max_m(1.5) 이상은 접근 goal 을 만들 수 있는 거리다 —
        이 경로가 관여하지 않는다."""
        logic = MissionLogic()
        seek_and_finish_turn(logic, t0=1.0)
        actions = logic.on_person_detection(
            track_id=7, distance_m=1.5, stable=True, approachable=False, now=3.0)
        assert actions == []
        assert logic.state == State.IDLE

    def test_unstable_ignored(self):
        logic = MissionLogic()
        seek_and_finish_turn(logic, t0=1.0)
        actions = logic.on_person_detection(
            track_id=7, distance_m=1.2, stable=False, approachable=False, now=3.0)
        assert actions == []
        assert logic.state == State.IDLE

    def test_track_id_none_ignored(self):
        logic = MissionLogic()
        seek_and_finish_turn(logic, t0=1.0)
        actions = logic.on_person_detection(
            track_id=0, distance_m=1.2, stable=True, approachable=False, now=3.0)
        assert actions == []
        assert logic.state == State.IDLE

    def test_estop_ignored(self):
        logic = MissionLogic()
        seek_and_finish_turn(logic, t0=1.0)
        logic.estop_active = True
        actions = logic.on_person_detection(
            track_id=7, distance_m=1.2, stable=True, approachable=False, now=3.0)
        assert actions == []
        assert logic.state == State.IDLE

    def test_declined_suppresses_track(self):
        logic = MissionLogic()
        seek_and_finish_turn(logic, t0=1.0)
        logic.on_person_detection(
            track_id=7, distance_m=1.2, stable=True, approachable=False, now=3.0)
        logic.on_approach_answer(False, 4.0)
        logic.on_tick(4.5, NavStatus.NONE)   # RETURNING -> IDLE (목적지 없음)
        _, reason = logic.on_approach_request(
            make_approach(track_id=7), BOUNDS, True, 5.0)
        assert reason == GateReason.TRACK_SUPPRESSED

    def test_no_answer_still_works(self):
        """무응답 사다리는 기존 그대로 재사용된다."""
        logic = MissionLogic(return_destination=make_home(),
                             approach_response_timeout_sec=8.0)
        seek_and_finish_turn(logic, t0=1.0)
        logic.on_person_detection(
            track_id=7, distance_m=1.2, stable=True, approachable=False, now=3.0)
        actions = logic.on_tick(3.0 + APPROACH_QUESTION_STUCK_SEC, NavStatus.NONE)
        assert logic.state == State.RETURNING
        says = [a for a in actions if isinstance(a, Say)]
        assert says and says[0].text == MSG_APPROACH_NO_ANSWER

    def test_custom_thresholds(self):
        logic = MissionLogic(near_call_max_m=2.0, near_call_no_spin_m=1.5)
        seek_and_finish_turn(logic, t0=1.0)
        actions = logic.on_person_detection(
            track_id=7, distance_m=1.8, stable=True, approachable=False, now=3.0)
        assert logic.state == State.AWAITING_USER
        assert not any(isinstance(a, Navigate) for a in actions)

    def test_default_thresholds_match_module_constants(self):
        logic = MissionLogic()
        assert logic.near_call_max_m == NEAR_CALL_MAX_M
        assert logic.near_call_no_spin_m == NEAR_CALL_NO_SPIN_M
