"""home_storage 순수 로직 unit test."""
import pytest

from vica_mission_manager.home_storage import (
    HOME_UNREACHABLE_MESSAGE,
    HOME_VISIT_RADIUS_M,
    SOURCE_MAP_PICK,
    SOURCE_ROBOT_STANDING,
    HomeStorage,
    build_home,
    near_home,
    normalize_yaw_deg,
    validate_map_id,
)


@pytest.fixture()
def storage(tmp_path):
    return HomeStorage(tmp_path)


def test_map_id_rejects_path_characters():
    """`../` 같은 경로 문자로 저장 폴더 밖을 건드리지 못하게 한다."""
    for bad in ("../etc", "a/b", "", "-leading", "한글"):
        with pytest.raises(ValueError):
            validate_map_id(bad)


def test_map_id_accepts_normal_names():
    assert validate_map_id("vica_map_0630") == "vica_map_0630"
    assert validate_map_id("  map-1  ") == "map-1"


def test_yaw_is_normalized_to_0_360():
    """destinations.yaml 의 pose 와 같은 규약을 쓴다."""
    assert normalize_yaw_deg(-90.0) == 270.0
    assert normalize_yaw_deg(450.0) == 90.0
    assert normalize_yaw_deg(0.0) == 0.0


def test_unknown_source_is_rejected():
    """source 는 '어떻게 정했나'의 기록이라 임의 문자열을 받으면 뜻을 잃는다."""
    with pytest.raises(ValueError):
        build_home("m1", 1.0, 2.0, 0.0, source="guessed")


def test_non_finite_pose_is_rejected():
    with pytest.raises(ValueError):
        build_home("m1", float("nan"), 2.0, 0.0, source=SOURCE_MAP_PICK)
    with pytest.raises(ValueError):
        build_home("m1", 1.0, float("inf"), 0.0, source=SOURCE_MAP_PICK)


def test_score_out_of_range_is_rejected():
    with pytest.raises(ValueError):
        build_home("m1", 1.0, 2.0, 0.0, source=SOURCE_ROBOT_STANDING, score=120.0)


def test_new_home_is_never_visited_even_when_robot_was_standing_there():
    """로봇을 실제로 세워놓고 잡았어도 '가 봤다'로 치지 않는다."""
    home = build_home(
        "m1", 1.0, 2.0, 90.0, source=SOURCE_ROBOT_STANDING, score=84.2
    )
    assert home.visited_ok is False


def test_mark_visited_flips_only_the_flag(storage):
    saved = storage.write(
        build_home("m1", 1.0, 2.0, 90.0, source=SOURCE_MAP_PICK)
    )
    updated = storage.mark_visited("m1", True)

    assert updated is not None
    assert updated.visited_ok is True
    assert (updated.x, updated.y, updated.yaw) == (saved.x, saved.y, saved.yaw)
    assert updated.source == saved.source
    assert updated.saved_at == saved.saved_at


def test_failed_return_keeps_the_visit_and_marks_unreachable(storage):
    """한 번 가 본 홈은 복귀가 실패해도 '가 본 적 없음'으로 돌아가지 않는다(11-19 사용자)."""
    storage.write(build_home("m1", 1.0, 2.0, 0.0, source=SOURCE_MAP_PICK))
    storage.record_reach("m1", True)

    updated = storage.record_reach("m1", False)

    assert updated is not None
    assert updated.visited_ok is True
    assert updated.reach_failed is True
    assert storage.read("m1").reach_failed is True


def test_failed_return_on_a_never_visited_home_is_unreachable_too(storage):
    storage.write(build_home("m1", 1.0, 2.0, 0.0, source=SOURCE_MAP_PICK))

    updated = storage.record_reach("m1", False)

    assert (updated.visited_ok, updated.reach_failed) == (False, True)


def test_reaching_home_clears_unreachable(storage):
    storage.write(build_home("m1", 1.0, 2.0, 0.0, source=SOURCE_MAP_PICK))
    storage.record_reach("m1", False)

    updated = storage.record_reach("m1", True)

    assert (updated.visited_ok, updated.reach_failed) == (True, False)


def test_old_file_without_reach_failed_reads_as_reachable(storage, tmp_path):
    storage.write(build_home("m1", 1.0, 2.0, 0.0, source=SOURCE_MAP_PICK))
    path = storage.path_for("m1")
    path.write_text(path.read_text(encoding="utf-8").replace("reach_failed: false\n", ""), encoding="utf-8")
    assert "reach_failed" not in path.read_text(encoding="utf-8")

    assert storage.read("m1").reach_failed is False


def test_near_home_is_within_thirty_centimetres():
    """홈 30 cm 안에 한 번이라도 들어오면 가 본 것이다 — Nav2 가 방향을 못 맞춰 '성공'을 못 내도"""
    assert HOME_VISIT_RADIUS_M == 0.30
    assert near_home(1.0, 2.0, 1.2, 2.2)
    assert not near_home(1.0, 2.0, 1.25, 2.2)
    assert HOME_UNREACHABLE_MESSAGE == "홈에 접근할 수 없습니다."


def test_new_map_keeps_coordinates_but_requires_recheck(storage):
    """지도를 다시 그리면 좌표는 남기고 확인만 되돌린다."""
    storage.write(build_home("m1", 1.5, -2.5, 180.0, source=SOURCE_ROBOT_STANDING))
    storage.mark_visited("m1", True)

    storage.record_reach("m1", False)

    updated = storage.invalidate_for_new_map("m1")

    assert updated is not None
    assert updated.visited_ok is False
    assert updated.reach_failed is False
    assert (updated.x, updated.y, updated.yaw) == (1.5, -2.5, 180.0)


def test_mark_visited_on_missing_home_returns_none(storage):
    assert storage.mark_visited("m1", True) is None
    assert storage.invalidate_for_new_map("m1") is None


def test_missing_home_reads_as_none_not_error(storage):
    """홈이 없는 것은 오류가 아니라 아직 지정하지 않은 정상 상태다."""
    assert storage.read("m1") is None


def test_write_then_read_roundtrip(storage):
    written = storage.write(
        build_home(
            "vica_map_0630",
            -5.83,
            -0.04,
            90.0,
            source=SOURCE_ROBOT_STANDING,
            score=84.2,
            label="충전 스테이션 앞",
        )
    )
    loaded = storage.read("vica_map_0630")

    assert loaded is not None
    assert loaded.x == pytest.approx(-5.83)
    assert loaded.y == pytest.approx(-0.04)
    assert loaded.yaw == pytest.approx(90.0)
    assert loaded.source == SOURCE_ROBOT_STANDING
    assert loaded.score == pytest.approx(84.2)
    assert loaded.label == "충전 스테이션 앞"
    assert loaded.visited_ok is False
    assert loaded.saved_at == written.saved_at


def test_home_lives_next_to_destinations(storage, tmp_path):
    """목적지 카탈로그와 같은 폴더에 둔다. 지도별 자료가 흩어지지 않게 한다."""
    storage.write(build_home("m1", 0.0, 0.0, 0.0, source=SOURCE_MAP_PICK))
    assert (tmp_path / "m1" / "home.yaml").exists()


def test_saving_again_overwrites_so_there_is_only_one_home(storage, tmp_path):
    """파일이 하나라서 '지도당 1개'가 저절로 지켜진다."""
    storage.write(build_home("m1", 1.0, 1.0, 0.0, source=SOURCE_MAP_PICK))
    storage.write(build_home("m1", 9.0, 9.0, 0.0, source=SOURCE_MAP_PICK))

    loaded = storage.read("m1")
    assert loaded is not None
    assert (loaded.x, loaded.y) == (9.0, 9.0)
    assert len(list((tmp_path / "m1").glob("home*.yaml"))) == 1


def test_corrupt_file_reads_as_none_instead_of_raising(storage):
    """읽기 실패로 노드를 세우지 않는다."""
    path = storage.path_for("m1")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("pose: [this is not a mapping\n", encoding="utf-8")

    assert storage.read("m1") is None


def test_home_without_pose_reads_as_none(storage):
    path = storage.path_for("m1")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("map_id: m1\nvisited_ok: true\n", encoding="utf-8")

    assert storage.read("m1") is None


def test_delete_removes_the_file(storage):
    storage.write(build_home("m1", 0.0, 0.0, 0.0, source=SOURCE_MAP_PICK))

    assert storage.delete("m1") is True
    assert storage.read("m1") is None
    assert storage.delete("m1") is False


def test_write_leaves_no_temporary_file(storage, tmp_path):
    """원자적 저장의 흔적이 남지 않아야 다음 읽기가 헷갈리지 않는다."""
    storage.write(build_home("m1", 0.0, 0.0, 0.0, source=SOURCE_MAP_PICK))
    assert list((tmp_path / "m1").glob(".home-*")) == []
