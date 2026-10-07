"""로봇 대장: 좌표→장소 라벨, JSON 보존, 메시지 칸 파생, 사건→대장 전이. 판단 없음."""
from vica_mission_manager.ledger import (
    HOME_ID,
    Ledger,
    LedgerStore,
    apply_goal_event,
    confirm_abort_name,
    place_here,
    state_fields,
)
from vica_mission_manager.mission_logic import Destination, Pose2D, State


def _dest(id_, name, x, y, calibrated=True):
    return Destination(id=id_, name=name, pose=Pose2D(x=x, y=y, yaw_deg=0.0), calibrated=calibrated)


# 좌표는 (0,0)을 피한다 — place_here 후보 필터가 (0,0)을 미캘리브레이션
# 플레이스홀더로 취급해서 뺀다(최종 리뷰 4절 m2). 상대 거리는 원래 값 그대로
# +100 을 더해 옮겼을 뿐이다.
DESTS = {
    "a": _dest("a", "407호", 100.0, 100.0),
    "b": _dest("b", "화장실", 110.0, 100.0),
    "__home__": _dest("__home__", "홈", 150.0, 150.0),
}


class TestPlaceHere:
    def test_within_near_is_front_of(self):
        label, dist = place_here(Pose2D(101.0, 100.0, 0.0), DESTS)
        assert label == "407호 앞" and abs(dist - 1.0) < 1e-6

    def test_far_is_between_two_nearest(self):
        label, dist = place_here(Pose2D(105.0, 100.0, 0.0), DESTS)
        assert label == "407호와 화장실 사이" and abs(dist - 5.0) < 1e-6

    def test_josa_gwa_after_consonant(self):
        dests = {"a": _dest("a", "화장실", 100.0, 100.0), "b": _dest("b", "407호", 110.0, 100.0)}
        label, _ = place_here(Pose2D(104.0, 100.0, 0.0), dests)
        assert label == "화장실과 407호 사이"

    def test_home_is_ignored_for_label(self):
        label, _ = place_here(Pose2D(149.0, 150.0, 0.0), DESTS)
        assert "홈" not in label

    def test_no_pose_or_big_covariance_is_unknown(self):
        assert place_here(None, DESTS) == ("", -1.0)
        assert place_here(Pose2D(101.0, 100.0, 0.0), DESTS, cov_xy=5.0) == ("", -1.0)

    def test_single_destination_far_is_near(self):
        label, _ = place_here(Pose2D(20.0, 0.0, 0.0), {"a": DESTS["a"]})
        assert label == "407호 근처"

    def test_uncalibrated_is_not_a_candidate(self):
        dests = {**DESTS, "z": _dest("z", "미캘리브레이션", 101.0, 100.0, calibrated=False)}
        label, _ = place_here(Pose2D(101.0, 100.0, 0.0), dests)
        assert label == "407호 앞"          # z 가 더 가깝지만(0 m) 후보에서 빠진다

    def test_zero_origin_placeholder_is_not_a_candidate(self):
        dests = {**DESTS, "z": _dest("z", "원점 플레이스홀더", 0.0, 0.0)}
        label, _ = place_here(Pose2D(0.1, 0.0, 0.0), dests)
        assert "원점 플레이스홀더" not in label


class TestStore:
    def test_roundtrip_restores_only_past_facts(self, tmp_path):
        store = LedgerStore(str(tmp_path / "destinations.yaml"))
        led = Ledger(active_destination="화장실", last_destination="407호", last_arrived_at=1000.0,
                     aborted_destination="세미나실")
        assert store.write(led) is True
        back = store.read()
        assert back.last_destination == "407호" and back.last_arrived_at == 1000.0
        assert back.aborted_destination == "세미나실"
        assert back.active_destination == ""          # 가는 중은 복원하지 않는다
        assert store.path.name == "ledger.json"

    def test_missing_or_broken_file_is_empty(self, tmp_path):
        store = LedgerStore(str(tmp_path / "destinations.yaml"))
        assert store.read() == Ledger()
        store.path.write_text("{not json", encoding="utf-8")
        assert store.read() == Ledger()

    def test_write_failure_returns_false(self, tmp_path):
        store = LedgerStore(str(tmp_path / "nope" / "destinations.yaml"))
        store.path = tmp_path            # 디렉터리에 쓰기 → 실패
        assert store.write(Ledger()) is False

    def test_invalid_utf8_is_empty(self, tmp_path):
        store = LedgerStore(str(tmp_path / "destinations.yaml"))
        store.path.write_bytes(b'{"last_destination": "\xff\xfe"}')
        assert store.read() == Ledger()

    def test_nan_last_arrived_is_ignored(self, tmp_path):
        store = LedgerStore(str(tmp_path / "destinations.yaml"))
        store.path.write_text('{"last_arrived_at": NaN, "last_destination": "407호"}',
                              encoding="utf-8")
        back = store.read()
        assert back.last_arrived_at is None
        assert back.last_destination == "407호"       # 나머지 칸은 그대로 복원


class TestStateFields:
    def test_fields_are_derived_not_judged(self):
        led = Ledger(active_destination="", last_destination="407호", last_arrived_at=1000.0,
                     aborted_destination="화장실")
        f = state_fields(led, "waiting", Pose2D(101.0, 100.0, 0.0), 0.0, DESTS, now_epoch=1660.0,
                         wait_minutes=10, wait_left_sec=340)
        assert f == {
            "dialog_state": "waiting", "place_here": "407호 앞", "place_here_dist_m": 1.0,
            "active_destination": "", "last_destination": "407호", "last_arrived_age_sec": 660,
            "aborted_destination": "화장실", "wait_minutes": 10, "wait_left_sec": 340, "battery_pct": -1,
            "door_side": "", "wait_place": "",   # 2026-10-07 대기 장소 칸(기본 빈 값)
        }

    def test_unknowns_are_minus_one_or_empty(self):
        f = state_fields(Ledger(), "idle", None, 0.0, {}, now_epoch=5.0, wait_minutes=-1, wait_left_sec=-1)
        assert f["place_here"] == "" and f["place_here_dist_m"] == -1.0
        assert f["last_arrived_age_sec"] == -1 and f["battery_pct"] == -1

    def test_huge_finite_age_is_clamped_to_int32(self):
        led = Ledger(last_arrived_at=-1e300)
        f = state_fields(led, "idle", None, 0.0, {}, now_epoch=0.0, wait_minutes=-1, wait_left_sec=-1)
        assert f["last_arrived_age_sec"] == 2 ** 31 - 1


class TestApplyGoalEvent:
    """사건 → 대장 전이(최종 리뷰 4절·6절 M4). 노드는 결과만 복사·저장한다."""

    def test_sent_then_succeeded(self):
        led = Ledger(aborted_destination="화장실")
        assert apply_goal_event(led, "goal_sent", "a", "407호", now=100.0) is True
        assert led.active_destination == "407호" and led.aborted_destination == ""
        assert apply_goal_event(led, "goal_succeeded", "a", "407호", now=200.0) is True
        assert led.last_destination == "407호" and led.last_arrived_at == 200.0
        assert led.active_destination == ""

    def test_sent_then_canceled(self):
        led = Ledger()
        apply_goal_event(led, "goal_sent", "a", "407호", now=100.0)
        dirty = apply_goal_event(led, "goal_canceled", "a", "407호", now=150.0)
        assert dirty is True
        assert led.active_destination == "" and led.aborted_destination == "407호"

    def test_rejected(self):
        led = Ledger()
        dirty = apply_goal_event(led, "goal_rejected", "a", "407호", now=10.0)
        assert dirty is True
        assert led.aborted_destination == "407호" and led.active_destination == ""

    def test_return_home_excludes_home_name(self):
        led = Ledger()
        # __home__ 은 goal_sent 로 잘못 넘어와도(방어) 이름이 안 적힌다
        dirty = apply_goal_event(led, "goal_sent", HOME_ID, "홈", now=10.0)
        assert dirty is False
        assert led.active_destination == ""
        # 정상 경로: return_home_succeeded 는 last_destination 을 안 건드린다
        dirty2 = apply_goal_event(led, "return_home_succeeded", HOME_ID, "홈", now=20.0)
        assert dirty2 is False
        assert led.last_destination == "" and led.active_destination == ""

    def test_approach_destination_does_not_touch_ledger(self):
        led = Ledger()
        apply_goal_event(led, "goal_sent", "approach:42", "접근 대상", now=5.0)
        assert led.active_destination == ""
        dirty = apply_goal_event(led, "goal_succeeded", "approach:42", "접근 대상", now=6.0)
        assert dirty is False
        assert led.last_destination == "" and led.active_destination == ""

    def test_goal_sent_clears_aborted_and_saves(self):
        led = Ledger(aborted_destination="화장실")
        dirty = apply_goal_event(led, "goal_sent", "a", "407호", now=1.0)
        assert dirty is True
        assert led.aborted_destination == "" and led.active_destination == "407호"


class TestConfirmAbortName:
    """CONFIRMING 에서 출발 없이 접힌 목적지 이름(최종 리뷰 4절·6절 M4)."""

    def test_confirming_to_idle_is_recorded(self):
        name = confirm_abort_name("a", None, State.CONFIRMING, State.IDLE, None, DESTS)
        assert name == "407호"

    def test_confirming_to_navigating_is_not_recorded(self):
        name = confirm_abort_name("a", None, State.CONFIRMING, State.NAVIGATING, None, DESTS)
        assert name is None


def test_wait_spot_goals_do_not_pollute_ledger():
    # 대기 장소·목적지 복귀·가보기 goal 은 실목적지가 아니다(2026-10-07).
    from vica_mission_manager.mission_logic import (
        WAIT_BACK_DESTINATION_PREFIX, WAIT_SPOT_DESTINATION_PREFIX)
    led = Ledger(last_destination="화장실 입구")
    apply_goal_event(led, "goal_succeeded", WAIT_SPOT_DESTINATION_PREFIX + "d1",
                     "화장실 입구-대기", 10.0)
    apply_goal_event(led, "goal_failed", WAIT_BACK_DESTINATION_PREFIX + "d1",
                     "화장실 입구", 11.0)
    assert led.last_destination == "화장실 입구"
    assert led.aborted_destination == ""


def test_state_fields_carry_door_side_and_wait_place():
    f = state_fields(Ledger(), "waiting", None, 0.0, {}, now_epoch=0.0,
                     wait_minutes=10, wait_left_sec=500,
                     door_side="오른쪽", wait_place="입구 오른쪽")
    assert f["door_side"] == "오른쪽" and f["wait_place"] == "입구 오른쪽"
    f = state_fields(Ledger(), "idle", None, 0.0, {}, now_epoch=0.0,
                     wait_minutes=-1, wait_left_sec=-1)
    assert f["door_side"] == "" and f["wait_place"] == ""
