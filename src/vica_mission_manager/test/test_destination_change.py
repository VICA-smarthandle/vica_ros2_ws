"""주행 중 목적지 바꾸기 · 도착 질문 중 목적지 관문 (2026-10-07 사용자 결정).

작업 계획 탭 '주행 중 목적지 변경 흐름':
  주행 중 "비카야" → "네?" → "화장실 가고 싶어" → 로봇이 멈추고 LLM 이 묻는다
  ("지금 409호로 가는 중이에요. 화장실로 바꿀까요?"). 미션은 물은 목적지를 기억하고
  말하지 않는다. "네"면 화장실로, "아니요"·무응답이면 원래 목적지로 다시 출발.
옛 v1 정책(무조건 MSG_BUSY)은 09-24 11:30 실기에서 음성의 확인 질문과 거절이 연달아
나와 로봇이 자기 말을 다시 들었다.
"""
from vica_mission_manager.mission_logic import (
    MSG_APPROACH_BUSY,
    MSG_BUSY,
    MSG_CANCELED,
    MSG_CANCEL_CONFIRM,
    MSG_CONFIRM_TIMEOUT,
    MSG_NAV_NOT_READY,
    MSG_UNKNOWN_DEST,
    MSG_PAUSED,
    MSG_POSE_INVALID,
    MSG_PRIVATE_DEST,
    MSG_WAKE_GREETING,
    NAV_TREE_GUIDED,
    CancelNav,
    Destination,
    GoalEvent,
    GateReason,
    IntentData,
    MapBounds,
    MissionLogic,
    Navigate,
    NavStatus,
    Pose2D,
    Say,
    State,
    WakeReply,
)

BOUNDS = MapBounds(min_x=-50, min_y=-50, max_x=50, max_y=50)
ROOM = Destination(id="r409", name="409호", pose=Pose2D(x=5, y=1, yaw_deg=0, frame_id="map"),
                   calibrated=True)
TOILET = Destination(id="wc", name="화장실", pose=Pose2D(x=-3, y=2, yaw_deg=90, frame_id="map"),
                     calibrated=True)
CAFE = Destination(id="cafe", name="식당", pose=Pose2D(x=8, y=-2, yaw_deg=0, frame_id="map"),
                   calibrated=True, confirm_prompt="식당으로 모실까요?")


def _intent(dest_id, need_confirm=False, intent="navigate"):
    return IntentData(intent=intent, matched_destination_id=dest_id,
                      need_confirm=need_confirm, safety_flag="normal")


def _say(actions):
    return [a.text for a in actions if isinstance(a, Say)]


def _navs(actions):
    return [a for a in actions if isinstance(a, Navigate)]


def guiding(t=0.0):
    """409호로 안내 주행 중인 logic."""
    logic = MissionLogic(arrival_dialog=True)
    logic.on_intent(_intent(ROOM.id), ROOM, BOUNDS, True, t)
    assert logic.state == State.NAVIGATING
    return logic


def asked_change(t=1.0):
    """409호로 가다가 '화장실' 제안 — 멈추고 바꿀지 묻는 중."""
    logic = guiding()
    actions = logic.on_intent(_intent(TOILET.id, need_confirm=True), TOILET, BOUNDS, True, t)
    assert logic.state == State.CONFIRMING
    return logic, actions


class TestProposalStopsSilently:
    def test_stops_remembers_and_says_nothing(self):
        logic, actions = asked_change()
        # 제안 단계 침묵 — 질문은 음성 쪽(LLM)이 이미 했다.
        assert _say(actions) == []
        cancels = [a for a in actions if isinstance(a, CancelNav)]
        assert len(cancels) == 1 and cancels[0].event == "goal_paused"
        assert cancels[0].destination == ROOM
        assert logic.confirming_dest_id == TOILET.id
        assert logic.active_destination is None

    def test_same_destination_keeps_going(self):
        logic = guiding()
        assert logic.on_intent(_intent(ROOM.id, need_confirm=True), ROOM, BOUNDS, True, 1.0) == []
        assert logic.state == State.NAVIGATING

    def test_unknown_destination_keeps_going(self):
        """미션이 모르는 곳이면 멈추지 않는다 — 확인 답이 와도 갈 곳이 없어 30초를 선다."""
        logic = guiding()
        assert logic.on_intent(_intent("ghost", need_confirm=True), None, BOUNDS, True, 1.0) == []
        assert logic.state == State.NAVIGATING


class TestAnswers:
    def test_yes_goes_to_new_destination(self):
        logic, _ = asked_change()
        actions = logic.on_confirm_answer(True, TOILET, BOUNDS, True, 2.0)
        assert logic.state == State.NAVIGATING
        assert logic.active_destination == TOILET
        assert _say(actions) == ["화장실로 안내를 시작합니다."]
        navs = _navs(actions)
        assert len(navs) == 1 and navs[0].destination == TOILET
        assert navs[0].tree == NAV_TREE_GUIDED
        # 확정 뒤 '아니오'가 와도 옛 목적지로 되돌아가지 않는다.
        assert logic.on_confirm_answer(False, None, BOUNDS, True, 3.0) == []

    def test_reproposing_the_asked_destination_is_yes(self):
        logic, _ = asked_change()
        actions = logic.on_intent(_intent(TOILET.id, need_confirm=True), TOILET, BOUNDS, True, 2.0)
        assert logic.state == State.NAVIGATING and logic.active_destination == TOILET
        assert _navs(actions)[0].destination == TOILET

    def test_no_resumes_original_with_resumed_message(self):
        logic, _ = asked_change()
        actions = logic.on_confirm_answer(False, None, BOUNDS, True, 2.0)
        assert logic.state == State.NAVIGATING
        assert logic.active_destination == ROOM
        says = [a for a in actions if isinstance(a, Say)]
        assert [s.text for s in says] == ["409호로 다시 출발합니다."]
        assert says[0].priority == "response"
        assert MSG_CONFIRM_TIMEOUT not in _say(actions)
        navs = _navs(actions)
        assert len(navs) == 1 and navs[0].destination == ROOM
        assert navs[0].tree == NAV_TREE_GUIDED

    def test_silence_resumes_original(self):
        """15초 조용하면 한 번 다시 묻고(2026-10-08 다시 묻기), 30초면 원래 목적지로."""
        logic, _ = asked_change(t=1.0)
        assert logic.on_tick(15.9, NavStatus.NONE) == []
        assert _say(logic.on_tick(16.0, NavStatus.NONE)) == ["화장실로 안내해드릴까요?"]
        assert logic.on_tick(1.0 + logic.confirm_timeout_sec - 0.1, NavStatus.NONE) == []
        actions = logic.on_tick(1.0 + logic.confirm_timeout_sec, NavStatus.NONE)
        assert logic.state == State.NAVIGATING and logic.active_destination == ROOM
        assert _say(actions) == ["409호로 다시 출발합니다."]
        assert _navs(actions)[0].destination == ROOM

    def test_new_proposal_while_asking_keeps_original_memory(self):
        """"아니 식당으로 가자"(새 제안) — 질문이 식당으로 바뀌고, 거절하면 여전히 409호."""
        logic, _ = asked_change()
        assert logic.on_intent(_intent(CAFE.id, need_confirm=True), CAFE, BOUNDS, True, 2.0) == []
        assert logic.confirming_dest_id == CAFE.id
        actions = logic.on_confirm_answer(False, None, BOUNDS, True, 3.0)
        assert logic.active_destination == ROOM
        assert _navs(actions)[0].destination == ROOM

    def test_other_confirmed_destination_is_asked_again(self):
        """바꾸기 질문 중 또 다른 목적지를 확정하면 그 목적지로 다시 묻는다 — 바꾸기 질문은
        그대로다(2026-10-08 결정 4, 옛 09-01 동작은 말없이 원래 목적지로 다시 출발)."""
        logic, _ = asked_change()
        actions = logic.on_intent(_intent(CAFE.id), CAFE, BOUNDS, True, 2.0)
        assert logic.state == State.CONFIRMING and logic.confirming_dest_id == CAFE.id
        assert _say(actions) == ["네, 식당으로 모실까요?"]   # CAFE 의 확인 문장 앞에 "네, "
        assert logic._change_from == ROOM

    def test_gate_failure_on_yes_rejects_then_resumes(self):
        lost = Destination(id="lost", name="창고", pose=Pose2D(x=1, y=1, yaw_deg=0, frame_id="map"),
                           calibrated=False)
        logic = guiding()
        logic.on_intent(_intent(lost.id, need_confirm=True), lost, BOUNDS, True, 1.0)
        assert logic.state == State.CONFIRMING
        actions = logic.on_confirm_answer(True, lost, BOUNDS, True, 2.0)
        assert _say(actions) == [MSG_POSE_INVALID, "409호로 다시 출발합니다."]
        assert logic.state == State.NAVIGATING and logic.active_destination == ROOM

    def test_wake_while_asking_resumes_without_extra_words(self):
        """호출이면 질문을 접고 원래 주행으로 — "네?"가 나가는 참이라 출발 멘트는 없다."""
        logic, _ = asked_change()
        actions = logic.on_wake_call(2.0)
        assert _say(actions) == [MSG_WAKE_GREETING]
        assert _navs(actions)[0].destination == ROOM
        assert logic.state == State.NAVIGATING
        assert [a for a in actions if isinstance(a, WakeReply)] == [WakeReply(listen=True)]


class TestConfirmedWithoutProposal:
    def test_mission_asks_with_default_prompt(self):
        logic = guiding()
        actions = logic.on_intent(_intent(TOILET.id), TOILET, BOUNDS, True, 1.0)
        assert logic.state == State.CONFIRMING
        asks = [a for a in actions if isinstance(a, Say)]
        # 빈 확인 문구는 음성 destination_loader 기본 문구와 같은 글자(미리 합성됨).
        assert [a.text for a in asks] == ["화장실로 안내해드릴까요?"]
        assert asks[0].expects_reply

    def test_mission_asks_with_registered_prompt(self):
        logic = guiding()
        actions = logic.on_intent(_intent(CAFE.id), CAFE, BOUNDS, True, 1.0)
        assert _say(actions) == ["식당으로 모실까요?"]

    def test_gate_failure_rejects_and_keeps_going(self):
        private = Destination(id="lab", name="연구실", pose=Pose2D(x=2, y=2, yaw_deg=0, frame_id="map"),
                              calibrated=True, authorization="private")
        logic = guiding()
        actions = logic.on_intent(_intent(private.id), private, BOUNDS, True, 1.0)
        assert _say(actions) == [MSG_PRIVATE_DEST]
        assert not any(isinstance(a, CancelNav) for a in actions)
        assert logic.state == State.NAVIGATING and logic.active_destination == ROOM


class TestAppDriveIsNotChanged:
    """관리자 주행(원격·배송)은 음성으로 바꾸지 않는다 — 지나가던 사람이 가로채면 안 된다."""

    def test_proposal_is_silent_and_confirmed_is_busy(self):
        logic = MissionLogic()
        logic.on_app_destination(ROOM, BOUNDS, True, 0.0)
        assert logic.on_intent(_intent(TOILET.id, need_confirm=True), TOILET, BOUNDS, True, 1.0) == []
        assert _say(logic.on_intent(_intent(TOILET.id), TOILET, BOUNDS, True, 2.0)) == [MSG_BUSY]
        assert logic.state == State.NAVIGATING and logic.active_destination == ROOM


class TestCommandsWhileAsking:
    def test_pause_turns_into_pause_and_resume_continues(self):
        logic, _ = asked_change()
        actions, reason = logic.on_pause_request(2.0)
        assert reason == GateReason.OK and _say(actions) == [MSG_PAUSED]
        assert logic.state == State.PAUSED and logic.paused_destination == ROOM
        actions, reason = logic.on_resume_request(True, 3.0)
        assert reason == GateReason.OK
        assert logic.state == State.NAVIGATING and _navs(actions)[0].destination == ROOM

    def test_resume_goes_back_to_original(self):
        logic, _ = asked_change()
        actions, reason = logic.on_resume_request(True, 2.0)
        assert reason == GateReason.OK
        assert logic.state == State.NAVIGATING and logic.active_destination == ROOM
        assert _say(actions) == ["409호로 다시 출발합니다."]

    def test_resume_needs_nav_ready(self):
        logic, _ = asked_change()
        assert logic.on_resume_request(False, 2.0) == ([], GateReason.NAV_NOT_READY)
        assert logic.state == State.CONFIRMING

    def test_cancel_is_allowed(self):
        logic, _ = asked_change()
        actions, reason = logic.on_cancel_confirm_request(2.0)
        assert reason == GateReason.OK and _say(actions) == [MSG_CANCEL_CONFIRM]
        actions = logic.on_cancel_confirm_answer(True, 3.0)
        assert logic.state == State.IDLE and MSG_CANCELED in _say(actions)
        assert not _navs(actions)

    def test_estop_forgets_the_original(self):
        logic, _ = asked_change()
        logic.on_estop(True, 2.0)
        assert logic.state == State.ESTOPPED
        logic.on_estop(False, 3.0)
        logic.on_tick(30.0, NavStatus.NONE)
        # 해제 뒤 평범한 확인 질문의 거절이 옛 목적지 재출발이 되면 안 된다.
        logic.on_intent(_intent(TOILET.id, need_confirm=True), TOILET, BOUNDS, True, 31.0)
        assert logic.state == State.CONFIRMING
        actions = logic.on_confirm_answer(False, None, BOUNDS, True, 32.0)
        assert logic.state == State.IDLE and not _navs(actions)


class TestApproachProposalSilence:
    def test_proposal_silent_confirmed_rejected(self):
        logic = MissionLogic()
        logic.state = State.APPROACHING
        assert logic.on_intent(_intent(TOILET.id, need_confirm=True), TOILET, BOUNDS, True, 1.0) == []
        assert _say(logic.on_intent(_intent(TOILET.id), TOILET, BOUNDS, True, 2.0)) == [MSG_APPROACH_BUSY]


class TestArrivalAnswerGate:
    """도착 질문 중 확정 목적지도 평소 관문을 거친다(10-06 발견·10-07 수리)."""

    def arrived(self):
        logic = MissionLogic(arrival_dialog=True)
        logic.on_intent(_intent(ROOM.id), ROOM, BOUNDS, True, 0.0)
        logic.on_tick(1.0, NavStatus.SUCCEEDED)
        assert logic.state == State.ASKING_NEXT
        return logic

    def test_private_destination_is_rejected_and_dialog_stays(self):
        private = Destination(id="lab", name="연구실", pose=Pose2D(x=2, y=2, yaw_deg=0, frame_id="map"),
                              calibrated=True, authorization="private")
        logic = self.arrived()
        actions = logic.on_arrival_answer(_intent(private.id), 2.0, next_dest=private,
                                          bounds=BOUNDS, nav_ready=True)
        assert _say(actions) == [MSG_PRIVATE_DEST] and not _navs(actions)
        assert logic.state == State.ASKING_NEXT

    def test_nav_not_ready_is_rejected(self):
        logic = self.arrived()
        actions = logic.on_arrival_answer(_intent(TOILET.id), 2.0, next_dest=TOILET,
                                          bounds=BOUNDS, nav_ready=False)
        assert _say(actions) == [MSG_NAV_NOT_READY]
        assert logic.state == State.ASKING_NEXT

    def test_pose_outside_map_is_rejected(self):
        far = Destination(id="far", name="별관", pose=Pose2D(x=99, y=99, yaw_deg=0, frame_id="map"),
                          calibrated=True)
        logic = self.arrived()
        actions = logic.on_arrival_answer(_intent(far.id), 2.0, next_dest=far,
                                          bounds=BOUNDS, nav_ready=True)
        assert _say(actions) == [MSG_POSE_INVALID]

    def test_valid_destination_goes(self):
        logic = self.arrived()
        actions = logic.on_arrival_answer(_intent(TOILET.id), 2.0, next_dest=TOILET,
                                          bounds=BOUNDS, nav_ready=True)
        assert logic.state == State.NAVIGATING
        assert _navs(actions)[0].destination == TOILET


class TestReviewFixes:
    """2026-10-07 독립 검토가 찾은 것들."""

    def test_yes_while_nav2_is_down_pauses_instead_of_driving(self):
        """원래 목적지로도 못 간다 — 일시정지로 두고 "다시 가자"를 기다린다."""
        logic, _ = asked_change()
        actions = logic.on_confirm_answer(True, TOILET, BOUNDS, False, 2.0)
        assert _say(actions) == [MSG_NAV_NOT_READY]
        assert not _navs(actions)
        assert logic.state == State.PAUSED and logic.paused_destination == ROOM

    def test_pending_cancel_question_is_replaced_by_the_change_question(self):
        logic = guiding()
        logic.on_cancel_confirm_request(0.5)
        assert logic.cancel_confirm_pending
        logic.on_intent(_intent(TOILET.id, need_confirm=True), TOILET, BOUNDS, True, 1.0)
        assert not logic.cancel_confirm_pending
        logic.on_confirm_answer(True, TOILET, BOUNDS, True, 2.0)
        # 새 안내 중 "취소"는 다시 묻는다(바로 취소하지 않는다).
        actions, reason = logic.on_cancel_confirm_request(3.0)
        assert reason == GateReason.OK and _say(actions) == [MSG_CANCEL_CONFIRM]
        assert logic.state == State.NAVIGATING

    def test_new_destination_starts_with_a_fresh_retry_budget(self):
        logic, _ = asked_change()
        logic._nav_retry_count = 2
        logic.on_confirm_answer(True, TOILET, BOUNDS, True, 2.0)
        assert logic._nav_retry_count == 0

    def test_user_request_after_an_app_drive_is_a_user_guide(self):
        logic = MissionLogic(arrival_dialog=True)
        logic.on_app_destination(ROOM, BOUNDS, True, 0.0)
        logic.on_pause_request(1.0)
        assert logic.state == State.PAUSED
        logic.on_intent(_intent(TOILET.id, need_confirm=True), TOILET, BOUNDS, True, 2.0)
        logic.on_confirm_answer(True, TOILET, BOUNDS, True, 3.0)
        assert logic.state == State.NAVIGATING and logic._nav_from_app is False

    def test_cancel_while_asking_tells_the_app_and_ledger(self):
        logic, _ = asked_change()
        logic.on_cancel_confirm_request(2.0)
        actions = logic.on_cancel_confirm_answer(True, 3.0)
        events = [a for a in actions if isinstance(a, GoalEvent)]
        assert [(e.event, e.destination) for e in events] == [("goal_canceled", ROOM)]

    def test_cancel_while_paused_tells_the_app_and_ledger(self):
        logic = guiding()
        logic.on_pause_request(1.0)
        actions, reason = logic.on_cancel_request(2.0)
        assert reason == GateReason.OK
        events = [a for a in actions if isinstance(a, GoalEvent)]
        assert [(e.event, e.destination) for e in events] == [("goal_canceled", ROOM)]

    def test_confirmed_unknown_place_is_refused_out_loud(self):
        logic = guiding()
        actions = logic.on_intent(_intent("ghost"), None, BOUNDS, True, 1.0)
        assert _say(actions) == [MSG_UNKNOWN_DEST]
        assert logic.state == State.NAVIGATING and logic.active_destination == ROOM
