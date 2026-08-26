"""home_storage 순수 로직 unit test.

가장 중요한 계약 두 가지를 못박는다.
  1. 새로 저장한 홈은 visited_ok 가 False 다 — 좌표를 정한 것과 갈 수 있는 것은 다르다
  2. 지도를 다시 그리면 좌표는 남고 visited_ok 만 False 로 돌아간다
"""
import pytest

from vica_mission_manager.home_storage import (
    SOURCE_MAP_PICK,
    SOURCE_ROBOT_STANDING,
    HomeStorage,
    build_home,
    normalize_yaw_deg,
    validate_map_id,
)


@pytest.fixture()
def storage(tmp_path):
    return HomeStorage(tmp_path)


# ----------------------------------------------------------------------
# 값 검증
# ----------------------------------------------------------------------


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


# ----------------------------------------------------------------------
# 핵심 계약 — visited_ok
# ----------------------------------------------------------------------


def test_new_home_is_never_visited_even_when_robot_was_standing_there():
    """로봇을 실제로 세워놓고 잡았어도 '가 봤다'로 치지 않는다.

    그 자리에 서 있다는 것은 "지금 여기가 지도와 맞다"만 말하고,
    "Nav2 가 거기까지 경로를 그릴 수 있다"는 말하지 않는다.
    """
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
    # 좌표와 출처는 그대로여야 한다.
    assert (updated.x, updated.y, updated.yaw) == (saved.x, saved.y, saved.yaw)
    assert updated.source == saved.source
    assert updated.saved_at == saved.saved_at


def test_failed_visit_is_recorded_as_false(storage):
    """한 번 확인된 홈도 나중에 못 가는 자리가 될 수 있다.

    가구가 놓이거나 문이 잠기면 좌표는 그대로인데 도달할 수 없게 된다.
    """
    storage.write(build_home("m1", 1.0, 2.0, 0.0, source=SOURCE_MAP_PICK))
    storage.mark_visited("m1", True)

    updated = storage.mark_visited("m1", False)

    assert updated is not None
    assert updated.visited_ok is False


def test_new_map_keeps_coordinates_but_requires_recheck(storage):
    """지도를 다시 그리면 좌표는 남기고 확인만 되돌린다.

    지워 버리면 관리자가 처음부터 다시 잡아야 하고, 그대로 두면 벽 속에
    들어간 홈을 확인된 것으로 믿게 된다. 둘 다 나쁘므로 가운데를 택한다.
    """
    storage.write(build_home("m1", 1.5, -2.5, 180.0, source=SOURCE_ROBOT_STANDING))
    storage.mark_visited("m1", True)

    updated = storage.invalidate_for_new_map("m1")

    assert updated is not None
    assert updated.visited_ok is False
    assert (updated.x, updated.y, updated.yaw) == (1.5, -2.5, 180.0)


def test_mark_visited_on_missing_home_returns_none(storage):
    assert storage.mark_visited("m1", True) is None
    assert storage.invalidate_for_new_map("m1") is None


# ----------------------------------------------------------------------
# 파일 입출력
# ----------------------------------------------------------------------


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
    """읽기 실패로 노드를 세우지 않는다.

    홈이 없으면 자동 복귀만 꺼지고 안내는 정상 동작한다. 예외를 던져
    Mission Manager 를 못 뜨게 하는 쪽이 훨씬 나쁘다.
    """
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
    # 없는 것을 지우는 것은 실패가 아니라 '지울 게 없었다'이다.
    assert storage.delete("m1") is False


def test_write_leaves_no_temporary_file(storage, tmp_path):
    """원자적 저장의 흔적이 남지 않아야 다음 읽기가 헷갈리지 않는다."""
    storage.write(build_home("m1", 0.0, 0.0, 0.0, source=SOURCE_MAP_PICK))
    assert list((tmp_path / "m1").glob(".home-*")) == []
