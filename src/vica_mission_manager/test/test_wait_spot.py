"""대기 장소 시나리오 (2026-10-07, 인수인계 문서 '작업 계획' 탭).

도착은 위치만 맞추고(M1 로 입구 방향을 알린다), 대기를 고르면 대기 장소가 있는
목적지는 M2 → 손 놓기 → 혼자 대기 장소로 → 10초마다 M3 로 기다린다. 막히면 M6 을
말하고 목적지로 돌아와 기다리고, 시간이 다 되면 M7 을 말하고 홈으로 간다.
"""
import pytest

from vica_mission_manager.mission_logic import (
    Destination, GateReason, GoalEvent, IntentData, MapBounds, MissionLogic,
    Navigate, NavStatus, Pose2D, Say, State, WaitSpot, CancelNav,
    MSG_WAIT_BEACON, MSG_WAIT_CONFIRM, MSG_WAIT_DEFAULT, MSG_WAIT_EXPIRED,
    MSG_WAIT_SPOT_BLOCKED, NAV_TREE_DEFAULT, NAV_TREE_GUIDED, NAV_TREE_WAIT,
    WAIT_BACK_DESTINATION_PREFIX, WAIT_BEACON_INTERVAL_SEC,
    WAIT_RELEASE_SEC, WAIT_RELEASE_SPEECH_FALLBACK_SEC,
    WAIT_SPOT_DESTINATION_PREFIX, door_side_word, josa_eun_neun,
    Haptic, HAPTIC_PATTERN_WAKE_LOCATE, MSG_WAKE_GREETING,
)

BOUNDS = MapBounds(min_x=-50, min_y=-50, max_x=50, max_y=50)
HOME = Destination(id="__home__", name="홈", pose=Pose2D(0, 0, 0, "map"))
SPOT = WaitSpot(x=1.97, y=-1.42, yaw_deg=0.0, side="right")


def _dest(wait_spot=SPOT, door_yaw=270.0, category="", **kw):
    d = dict(id="d1", name="화장실 입구", pose=Pose2D(3.21, -1.05, 90.0, "map"),
             calibrated=True, arrival_message="화장실 입구 앞에 도착했습니다.",
             category=category, door_yaw_deg=door_yaw, wait_spot=wait_spot)
    d.update(kw)
    return Destination(**d)


def _intent(intent="navigate", **kw):
    d = dict(intent=intent, matched_destination_id="d1", need_confirm=False,
             safety_flag="normal")
    d.update(kw)
    return IntentData(**d)


def _say(actions):
    return [a.text for a in actions if isinstance(a, Say)]


def _navs(actions):
    return [a for a in actions if isinstance(a, Navigate)]


def _events(actions):
    return [a.event for a in actions if isinstance(a, GoalEvent)]


def arrive(dest=None, robot_yaw=0.0, **logic_kw):
    """주행 → 도착 → 질문 재생완료. ASKING_NEXT 의 logic 과 도착 발화를 준다."""
    logic = MissionLogic(return_destination=HOME, arrival_dialog=True, **logic_kw)
    logic.on_intent(_intent(), dest or _dest(), BOUNDS, True, 0.0)
    logic.robot_yaw_deg = robot_yaw
    acts = logic.on_tick(1.0, NavStatus.SUCCEEDED)
    assert logic.state == State.ASKING_NEXT, _say(acts)
    logic.on_arrival_question_spoken(2.0)
    return logic, acts


def wait_minutes(logic, minutes=10, now=3.0):
    return logic.on_arrival_answer(_intent("wait", wait_minutes=minutes), now)


def released(logic, now=3.0, minutes=10):
    """대기 확정 → M2 재생 완료 → (센서 없음) 대기 장소로 출발."""
    acts = wait_minutes(logic, minutes, now)
    text = _say(acts)[0]
    logic.on_wait_speech_spoken(text, now + 4.0)
    move = logic.on_tick(now + 4.1, NavStatus.NONE)
    assert logic.state == State.MOVING_TO_WAIT_SPOT
    return move


# ---- 조사·방향 순수 함수 -----------------------------------------------------


@pytest.mark.parametrize("word, josa", [
    ("화장실", "은"), ("안내센터", "는"), ("407호", "는"), ("회의실B1", "은"),
    ("2", "는"), ("세미나실3", "은"), ("Lab", "는"), ("", "는"),
])
def test_josa_eun_neun(word, josa):
    assert josa_eun_neun(word) == josa


@pytest.mark.parametrize("door, robot, side", [
    (270.0, 0.0, "오른쪽"),    # 동쪽을 보고 섰는데 입구가 남쪽 = 오른쪽
    (90.0, 0.0, "왼쪽"),
    (20.0, 0.0, "앞"),
    (0.0, 30.0, "앞"),         # 경계 ±30° 는 앞
    (0.0, 31.0, "오른쪽"),
    (180.0, 0.0, "뒤"),
    (-150.0, 0.0, "뒤"),
    (359.0, 1.0, "앞"),        # 360° 를 넘나드는 각도
])
def test_door_side_word(door, robot, side):
    assert door_side_word(door, robot) == side


# ---- M1: 도착 순간 입구 방향 ---------------------------------------------------


class TestDoorSideOnArrival:
    def test_m1_follows_arrival_message_in_one_utterance(self):
        logic, acts = arrive(robot_yaw=0.0)
        text = _say(acts)[0]
        assert text.startswith("화장실 입구 앞에 도착했습니다. 화장실 입구는 오른쪽에 있습니다.")
        assert logic.door_side == "오른쪽"

    def test_no_door_yaw_means_no_m1(self):
        logic, acts = arrive(_dest(door_yaw=None))
        assert "있습니다" not in _say(acts)[0]
        assert logic.door_side == ""

    def test_unknown_robot_heading_means_no_m1(self):
        logic, acts = arrive(robot_yaw=None)
        assert "있습니다" not in _say(acts)[0]

    def test_guided_navigation_uses_position_only_tree(self):
        logic = MissionLogic(arrival_dialog=True)
        acts = logic.on_intent(_intent(), _dest(), BOUNDS, True, 0.0)
        assert _navs(acts)[0].tree == NAV_TREE_GUIDED


# ---- 대기 확정 → 손 놓기 → 대기 장소 ---------------------------------------------


class TestWaitSpotFlow:
    def test_m2_and_waiting_release(self):
        logic, _ = arrive()
        acts = wait_minutes(logic, 10, 3.0)
        assert _say(acts) == [
            "10분 동안 입구 오른쪽에서 기다리겠습니다. 돌아오시면 '비카야'라고 불러 주세요."]
        assert logic.state == State.WAITING_RELEASE
        assert logic.wait_place == "입구 오른쪽"
        # 대기 시간은 M2 를 말한 순간부터 흐른다.
        assert logic.wait_left_sec(3.0) == 600

    def test_default_minutes_is_m2_prime(self):
        logic, _ = arrive(_dest(category="restroom"))
        acts = logic.on_arrival_answer(_intent("affirm"), 3.0)
        assert _say(acts) == [
            "최대 30분 동안 입구 오른쪽에서 기다리겠습니다. 돌아오시면 '비카야'라고 불러 주세요."]

    def test_across_side_phrase(self):
        logic, _ = arrive(_dest(wait_spot=WaitSpot(1.0, 1.0, 0.0, "across")))
        acts = wait_minutes(logic, 5)
        assert _say(acts)[0].startswith("5분 동안 입구 맞은편에서 기다리겠습니다.")

    def test_does_not_leave_while_m2_is_playing(self):
        logic, _ = arrive()
        wait_minutes(logic, 10, 3.0)
        assert logic.on_tick(5.0, NavStatus.NONE) == []
        assert logic.state == State.WAITING_RELEASE

    def test_leaves_right_after_m2_without_sensor(self):
        logic, _ = arrive()
        move = released(logic)
        nav = _navs(move)[0]
        assert nav.tree == NAV_TREE_WAIT
        assert nav.destination.id == WAIT_SPOT_DESTINATION_PREFIX + "d1"
        assert nav.destination.name == "화장실 입구-대기"
        assert (nav.destination.pose.x, nav.destination.pose.y) == (1.97, -1.42)

    def test_speech_done_fallback(self):
        logic, _ = arrive()
        wait_minutes(logic, 10, 3.0)
        logic.on_tick(3.0 + WAIT_RELEASE_SPEECH_FALLBACK_SEC + 0.1, NavStatus.NONE)
        assert logic.state == State.MOVING_TO_WAIT_SPOT

    def test_waits_for_hand_release_when_sensor_alive(self):
        logic, _ = arrive()
        acts = wait_minutes(logic, 10, 3.0)
        logic.on_handle_state(True, True, 3.0)
        logic.on_wait_speech_spoken(_say(acts)[0], 7.0)
        logic.on_handle_state(True, True, 7.5)
        assert logic.on_tick(7.6, NavStatus.NONE) == []          # 아직 잡고 있다
        logic.on_handle_state(False, True, 8.0)                  # 손을 뗐다
        for t in (9.0, 10.0, 11.0, 12.0):
            logic.on_handle_state(False, True, t)
            assert logic.on_tick(t, NavStatus.NONE) == []         # 5초가 안 됐다
        logic.on_handle_state(False, True, 13.05)
        logic.on_tick(13.05, NavStatus.NONE)
        assert logic.state == State.MOVING_TO_WAIT_SPOT
        assert WAIT_RELEASE_SEC == 5.0

    def test_demo_switch_leaves_right_after_m2(self):
        logic, _ = arrive(grip_assume_held=True)
        acts = wait_minutes(logic, 10, 3.0)
        logic.on_handle_state(True, True, 3.0)                   # 센서가 '잡음'이어도
        logic.on_wait_speech_spoken(_say(acts)[0], 7.0)
        logic.on_handle_state(True, True, 7.0)
        logic.on_tick(7.1, NavStatus.NONE)
        assert logic.state == State.MOVING_TO_WAIT_SPOT

    def test_arrives_then_beacons_every_10s(self):
        logic, _ = arrive()
        released(logic, now=3.0)                                 # 7.1 초에 출발
        assert logic.on_tick(8.0, NavStatus.SUCCEEDED) == []
        assert logic.state == State.WAITING
        assert logic.wait_place == "입구 오른쪽"
        first = 7.1 + WAIT_BEACON_INTERVAL_SEC
        assert logic.on_tick(first - 0.1, NavStatus.NONE) == []
        assert _say(logic.on_tick(first, NavStatus.NONE)) == [MSG_WAIT_BEACON]
        assert logic.on_tick(first + 5.0, NavStatus.NONE) == []
        acts = logic.on_tick(first + WAIT_BEACON_INTERVAL_SEC, NavStatus.NONE)
        # 가장 낮은 ambient 등급 — 다른 말 중이면 TTS 가 기다리지 않고 버린다(2026-10-07).
        assert [(a.text, a.priority) for a in acts] == [(MSG_WAIT_BEACON, "ambient")]

    def test_beacon_skipped_while_talking(self):
        logic, _ = arrive()
        released(logic, now=3.0)
        logic.on_tick(8.0, NavStatus.SUCCEEDED)
        logic.on_listen_state("speech", 16.0)
        assert logic.on_tick(17.2, NavStatus.NONE) == []         # 대화 중엔 건너뜀
        logic.on_listen_state("empty", 18.0)
        assert _say(logic.on_tick(27.2, NavStatus.NONE)) == [MSG_WAIT_BEACON]

    def test_blocked_says_m6_and_goes_back(self):
        logic, _ = arrive()
        released(logic, now=3.0)
        acts = logic.on_tick(40.0, NavStatus.FAILED)
        assert MSG_WAIT_SPOT_BLOCKED in _say(acts)
        assert _events(acts) == ["wait_spot_blocked"]
        assert [a for a in acts if isinstance(a, GoalEvent)][0].wait_place == "spot"
        back = _navs(acts)[0]
        assert back.tree == NAV_TREE_WAIT
        assert back.destination.id == WAIT_BACK_DESTINATION_PREFIX + "d1"
        assert back.destination.pose.yaw_deg == 270.0            # 입구 쪽을 보고 선다
        assert logic.state == State.MOVING_BACK_TO_DEST
        assert logic.wait_place == "입구 앞"
        logic.on_tick(50.0, NavStatus.FAILED)                     # 복귀는 재시도 없음
        assert logic.state == State.WAITING
        assert logic.wait_place == "입구 앞"
        # 막혀 목적지에서 기다려도 10초 알림은 이어진다.
        assert _say(logic.on_tick(60.0, NavStatus.NONE)) == [MSG_WAIT_BEACON]

    def test_no_wait_spot_waits_in_place_quietly(self):
        logic, _ = arrive(_dest(wait_spot=None))
        acts = wait_minutes(logic, 10, 3.0)
        assert _say(acts) == [MSG_WAIT_CONFIRM.format(minutes=10)]
        assert logic.state == State.WAITING
        assert logic.wait_place == ""
        assert logic.on_tick(100.0, NavStatus.NONE) == []          # M3 없음
        assert "불러 주세요" in MSG_WAIT_DEFAULT


# ---- 대기 중 "비카야"·말 ---------------------------------------------------------


class TestTalkingWhileWaiting:
    def test_wake_keeps_release_and_moving(self):
        logic, _ = arrive()
        wait_minutes(logic, 10, 3.0)
        assert logic.on_wake(4.0) == []
        assert logic.state == State.WAITING_RELEASE
        logic2, _ = arrive()
        released(logic2)
        assert logic2.on_wake(9.0) == []
        assert logic2.state == State.MOVING_TO_WAIT_SPOT

    def test_moving_ignores_destination(self):
        logic, _ = arrive()
        released(logic)
        nxt = _dest(id="d2", name="안내소")
        acts = logic.on_intent(_intent(matched_destination_id="d2"), nxt, BOUNDS, True, 9.0)
        assert acts == []
        assert logic.state == State.MOVING_TO_WAIT_SPOT

    def _waiting(self):
        logic, _ = arrive()
        released(logic, now=3.0)
        logic.on_tick(8.0, NavStatus.SUCCEEDED)
        return logic

    # ---- 대기 중 "비카야" → 손잡이 위치 진동 (2026-10-09 사용자) ----------------------
    # 볼일을 마친 사용자가 대기 장소의 비카를 손으로도 찾게 1초씩 두 번 떤다.
    # WAITING 일 때 /vica/wake 를 받은 경우에만 — 다른 상태는 떨지 않는다.

    @staticmethod
    def _haptics(actions):
        return [a.pattern for a in actions if isinstance(a, Haptic)]

    def test_wake_while_waiting_vibrates_locate(self):
        logic = self._waiting()
        left = logic.wait_left_sec(20.0)
        assert HAPTIC_PATTERN_WAKE_LOCATE == "locate"
        assert logic.on_wake(20.0) == [Haptic(HAPTIC_PATTERN_WAKE_LOCATE)]
        # 대기는 이어 간다 — 진동만 더해졌다.
        assert logic.state == State.WAITING
        assert logic.wait_left_sec(20.0) == left

    def test_wake_call_while_waiting_answers_and_vibrates_once(self):
        """일반 호출(on_wake_call)은 "네?"와 진동 한 번. 창 안에서 건진 호출은 노드가
        on_wake 만 부르므로 위 시험이 그 길이다 — 어느 길이든 호출 한 번에 한 번 떤다."""
        logic = self._waiting()
        acts = logic.on_wake_call(20.0)
        assert _say(acts) == [MSG_WAKE_GREETING]
        assert self._haptics(acts) == ["locate"]
        assert logic.state == State.WAITING

    def test_every_wake_while_waiting_vibrates_again(self):
        logic = self._waiting()
        assert self._haptics(logic.on_wake_call(20.0)) == ["locate"]
        assert self._haptics(logic.on_wake_call(40.0)) == ["locate"]
        assert logic.state == State.WAITING

    def test_no_vibration_outside_waiting(self):
        # 손 놓기 기다림 — 사용자가 아직 손잡이 곁이라 떨지 않는다.
        logic, _ = arrive()
        wait_minutes(logic, 10, 3.0)
        assert logic.state == State.WAITING_RELEASE
        assert self._haptics(logic.on_wake_call(4.0)) == []
        # 대기 장소로 가는 중 — 호출 자체를 무시한다.
        logic2, _ = arrive()
        released(logic2)
        assert self._haptics(logic2.on_wake_call(9.0)) == []
        # 그냥 서 있는 IDLE.
        idle = MissionLogic(return_destination=HOME, arrival_dialog=True)
        assert self._haptics(idle.on_wake_call(1.0)) == []

    def test_no_vibration_while_estopped(self):
        logic = self._waiting()
        logic.on_estop(True, 20.0)
        assert self._haptics(logic.on_wake_call(21.0)) == []

    def test_denied_proposal_returns_to_waiting(self):
        logic = self._waiting()
        nxt = _dest(id="d2", name="안내소")
        logic.on_intent(_intent(matched_destination_id="d2", need_confirm=True),
                        nxt, BOUNDS, True, 20.0)
        assert logic.state == State.CONFIRMING
        assert logic.wait_place == "입구 오른쪽"                  # 확인 중에도 대기 정보 유지
        logic.on_confirm_answer(False, nxt, BOUNDS, True, 21.0)
        assert logic.state == State.WAITING
        assert logic.wait_left_sec(21.0) == 600 - 18

    def test_confirm_timeout_and_wake_return_to_waiting(self):
        logic = self._waiting()
        nxt = _dest(id="d2", name="안내소")
        logic.on_intent(_intent(matched_destination_id="d2", need_confirm=True),
                        nxt, BOUNDS, True, 20.0)
        logic.on_tick(20.0 + 15.0, NavStatus.NONE)     # 15초: 같은 질문 한 번 더(2026-10-08)
        logic.on_tick(20.0 + 31.0, NavStatus.NONE)
        assert logic.state == State.WAITING
        logic.on_intent(_intent(matched_destination_id="d2", need_confirm=True),
                        nxt, BOUNDS, True, 60.0)
        logic.on_wake(61.0)
        assert logic.state == State.WAITING

    def test_confirmed_destination_ends_waiting(self):
        logic = self._waiting()
        nxt = _dest(id="d2", name="안내소", wait_spot=None)
        logic.on_intent(_intent(matched_destination_id="d2", need_confirm=True),
                        nxt, BOUNDS, True, 20.0)
        acts = logic.on_confirm_answer(True, nxt, BOUNDS, True, 21.0)
        assert logic.state == State.NAVIGATING
        assert _navs(acts)[0].tree == NAV_TREE_GUIDED
        assert logic.wait_place == "" and logic.wait_left_sec(21.0) == -1
        # 안내가 시작됐으니 10초 알림도 끝이다.
        assert MSG_WAIT_BEACON not in _say(logic.on_tick(40.0, NavStatus.RUNNING, 5.0))


# ---- 대기 시간 만료 ----------------------------------------------------------


class TestWaitExpired:
    def test_expiry_says_m7_alerts_app_and_goes_home(self):
        logic, _ = arrive()
        released(logic, now=3.0, minutes=1)
        logic.on_tick(8.0, NavStatus.SUCCEEDED)
        acts = logic.on_tick(3.0 + 61.0, NavStatus.NONE)
        assert _say(acts)[0] == MSG_WAIT_EXPIRED
        assert _events(acts) == ["wait_expired"]
        event = [a for a in acts if isinstance(a, GoalEvent)][0]
        assert event.destination.id == "d1"
        assert "입구 오른쪽" in event.reason
        assert (event.wait_place, event.wait_minutes) == ("spot", 1)   # 앱 팝업 칸
        assert logic.state == State.RETURNING
        assert _navs(acts)[0].destination.id == "__home__"

    def test_expiry_waits_for_conversation(self):
        logic, _ = arrive()
        released(logic, now=3.0, minutes=1)
        logic.on_tick(8.0, NavStatus.SUCCEEDED)
        logic.on_listen_state("speech", 60.0)
        assert MSG_WAIT_EXPIRED not in _say(logic.on_tick(64.0, NavStatus.NONE))
        assert logic.state == State.WAITING
        logic.on_listen_state("empty", 65.0)
        assert _say(logic.on_tick(66.0, NavStatus.NONE))[0] == MSG_WAIT_EXPIRED


# ---- 관리자 '대기 장소로 가보기' -------------------------------------------------


class TestAppWaitSpot:
    def test_goes_silently_with_wait_tree(self):
        logic = MissionLogic()
        acts, reason = logic.on_app_wait_spot(_dest(), BOUNDS, True, 0.0)
        assert reason == GateReason.OK
        assert _say(acts) == []
        assert _navs(acts)[0].tree == NAV_TREE_WAIT
        assert _say(logic.on_tick(5.0, NavStatus.SUCCEEDED)) == []   # 도착도 말하지 않는다
        assert logic.state == State.ARRIVED

    def test_failure_is_silent_and_not_retried(self):
        logic = MissionLogic()
        logic.on_app_wait_spot(_dest(), BOUNDS, True, 0.0)
        acts = logic.on_tick(5.0, NavStatus.FAILED)
        assert _say(acts) == []
        logic.on_tick(10.0, NavStatus.NONE)
        assert logic.state == State.IDLE

    @pytest.mark.parametrize("setup, dest, reason", [
        ("busy", _dest(), GateReason.BUSY_NAVIGATING),
        ("idle", _dest(wait_spot=None), GateReason.NO_WAIT_SPOT),
        ("idle", None, GateReason.UNKNOWN_DESTINATION),
    ])
    def test_rejections(self, setup, dest, reason):
        logic = MissionLogic()
        if setup == "busy":
            logic.on_intent(_intent(), _dest(), BOUNDS, True, 0.0)
        acts, got = logic.on_app_wait_spot(dest, BOUNDS, True, 1.0)
        assert got == reason and acts == []


# ---- 배송·원격 주행 ---------------------------------------------------------


class TestAppDestinationYaw:
    def test_delivery_faces_door(self):
        logic = MissionLogic()
        acts, _ = logic.on_app_destination(_dest(), BOUNDS, True, 0.0,
                                           allow_private=True, is_delivery=True)
        nav = _navs(acts)[0]
        assert nav.destination.pose.yaw_deg == 270.0
        assert nav.tree == NAV_TREE_DEFAULT

    def test_remote_drive_keeps_saved_yaw(self):
        logic = MissionLogic()
        acts, _ = logic.on_app_destination(_dest(), BOUNDS, True, 0.0)
        assert _navs(acts)[0].destination.pose.yaw_deg == 90.0

    def test_delivery_without_door_yaw_uses_pose_yaw(self):
        logic = MissionLogic()
        acts, _ = logic.on_app_destination(_dest(door_yaw=None), BOUNDS, True, 0.0,
                                           allow_private=True, is_delivery=True)
        assert _navs(acts)[0].destination.pose.yaw_deg == 90.0


# ---- 비상 정지 --------------------------------------------------------------


def test_estop_while_moving_cancels_and_clears_hold():
    logic, _ = arrive()
    released(logic)
    acts = logic.on_estop(True, 9.0)
    assert any(isinstance(a, CancelNav) for a in acts)
    assert logic.state == State.ESTOPPED
    logic.on_estop(False, 10.0)
    logic.on_tick(12.0, NavStatus.NONE)
    assert logic.state == State.IDLE
    assert logic.wait_place == ""


# ---- 독립 검토 뒤 고친 것(2026-10-07) ---------------------------------------


class TestReviewFixes:
    def test_release_waits_while_talking(self):
        # 손 놓기를 기다리다 "비카야"로 대화가 열렸으면 떠나지 않는다.
        logic, _ = arrive()
        acts = wait_minutes(logic, 10, 3.0)
        logic.on_listen_state("speech", 4.0)
        logic.on_wait_speech_spoken(_say(acts)[0], 7.0)
        assert _navs(logic.on_tick(7.1, NavStatus.NONE)) == []
        assert logic.state == State.WAITING_RELEASE
        logic.on_listen_state("empty", 8.0)
        assert _navs(logic.on_tick(30.0, NavStatus.NONE))
        assert logic.state == State.MOVING_TO_WAIT_SPOT

    def test_release_state_still_expires(self):
        # 손을 끝내 놓지 않아도(센서가 계속 '잡힘') 대기 시간이 끝나면 M7 + 앱 알림 + 홈.
        logic, _ = arrive()
        acts = wait_minutes(logic, 1, 3.0)
        logic.on_handle_state(True, True, 3.0)
        logic.on_wait_speech_spoken(_say(acts)[0], 7.0)
        for t in (10.0, 30.0, 60.0):
            logic.on_handle_state(True, True, t)
            assert logic.on_tick(t, NavStatus.NONE) == []
        logic.on_handle_state(True, True, 64.0)
        acts = logic.on_tick(64.0, NavStatus.NONE)
        assert MSG_WAIT_EXPIRED in _say(acts)
        assert _events(acts) == ["wait_expired"]
        assert logic.state == State.RETURNING

    def test_leaving_home_forgets_destination_and_door_side(self):
        logic, _ = arrive()
        logic.door_side = "오른쪽"
        logic._enter_returning(10.0, dialog_finish=True)
        assert logic._arrived_destination is None
        assert logic.door_side == ""


def test_door_side_is_forgotten_when_moving_to_the_wait_spot():
    """입구 방향은 도착한 자리의 로봇 기준 — 대기 장소로 움직이면 낡은 말이다(10-07 검토)."""
    logic, _ = arrive(robot_yaw=0.0)
    assert logic.door_side != ""
    released(logic, now=3.0)
    assert logic.door_side == ""
