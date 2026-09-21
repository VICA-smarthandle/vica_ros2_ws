"""로봇 대장: 좌표→장소 라벨, JSON 보존, 메시지 칸 파생. 판단 없음."""
import json

from vica_mission_manager.ledger import Ledger, LedgerStore, NEAR_M, place_here, state_fields
from vica_mission_manager.mission_logic import Destination, Pose2D


def _dest(id_, name, x, y):
    return Destination(id=id_, name=name, pose=Pose2D(x=x, y=y, yaw_deg=0.0), calibrated=True)


DESTS = {
    "a": _dest("a", "407호", 0.0, 0.0),
    "b": _dest("b", "화장실", 10.0, 0.0),
    "__home__": _dest("__home__", "홈", 50.0, 50.0),
}


class TestPlaceHere:
    def test_within_near_is_front_of(self):
        label, dist = place_here(Pose2D(1.0, 0.0, 0.0), DESTS)
        assert label == "407호 앞" and abs(dist - 1.0) < 1e-6

    def test_far_is_between_two_nearest(self):
        label, dist = place_here(Pose2D(5.0, 0.0, 0.0), DESTS)
        assert label == "407호와 화장실 사이" and abs(dist - 5.0) < 1e-6

    def test_josa_gwa_after_consonant(self):
        dests = {"a": _dest("a", "화장실", 0.0, 0.0), "b": _dest("b", "407호", 10.0, 0.0)}
        label, _ = place_here(Pose2D(4.0, 0.0, 0.0), dests)
        assert label == "화장실과 407호 사이"

    def test_home_is_ignored_for_label(self):
        label, _ = place_here(Pose2D(49.0, 50.0, 0.0), DESTS)
        assert "홈" not in label

    def test_no_pose_or_big_covariance_is_unknown(self):
        assert place_here(None, DESTS) == ("", -1.0)
        assert place_here(Pose2D(1.0, 0.0, 0.0), DESTS, cov_xy=5.0) == ("", -1.0)

    def test_single_destination_far_is_near(self):
        label, _ = place_here(Pose2D(20.0, 0.0, 0.0), {"a": DESTS["a"]})
        assert label == "407호 근처"


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


class TestStateFields:
    def test_fields_are_derived_not_judged(self):
        led = Ledger(active_destination="", last_destination="407호", last_arrived_at=1000.0,
                     aborted_destination="화장실")
        f = state_fields(led, "waiting", Pose2D(1.0, 0.0, 0.0), 0.0, DESTS, now_epoch=1660.0,
                         wait_minutes=10, wait_left_sec=340)
        assert f == {
            "dialog_state": "waiting", "place_here": "407호 앞", "place_here_dist_m": 1.0,
            "active_destination": "", "last_destination": "407호", "last_arrived_age_sec": 660,
            "aborted_destination": "화장실", "wait_minutes": 10, "wait_left_sec": 340, "battery_pct": -1,
        }

    def test_unknowns_are_minus_one_or_empty(self):
        f = state_fields(Ledger(), "idle", None, 0.0, {}, now_epoch=5.0, wait_minutes=-1, wait_left_sec=-1)
        assert f["place_here"] == "" and f["place_here_dist_m"] == -1.0
        assert f["last_arrived_age_sec"] == -1 and f["battery_pct"] == -1
