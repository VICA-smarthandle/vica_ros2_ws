"""홈 '가 봄'·'접근할 수 없음' 기록 — 11-19 사용자 요청.

"주행에서 어떻게든 홈으로 한 번 가 봤으면 '홈에 아직 가 본 적이 없습니다' 문구가 없어지게" ·
"길 막힘 등으로 홈 접근에 실패하면 '홈에 접근할 수 없습니다.' 로".

배경: 11-19 11:06 홈 복귀에서 로봇은 11:09:34 에 홈 xy 허용 범위 안에 들어왔지만 도크 옆이라
방향을 못 맞춰 Nav2 '성공'을 못 냈고, 4분 뒤 취소돼 '가 봄'이 기록되지 않았다.

test_nav_poll_wait.py 처럼 노드를 rclpy 없이 __new__ 로 만든다.
"""
from types import SimpleNamespace

import pytest

mm = pytest.importorskip("vica_mission_manager.mission_manager_node")

from vica_mission_manager.home_storage import (  # noqa: E402
    HOME_UNREACHABLE_MESSAGE,
    SOURCE_MAP_PICK,
    HomeStorage,
    build_home,
)

HOME = (-6.22, 2.31)


def _node(tmp_path, robot=None, cov=0.1):
    storage = HomeStorage(tmp_path)
    storage.write(build_home("m1", HOME[0], HOME[1], 0.0, source=SOURCE_MAP_PICK))
    node = mm.MissionManagerNode.__new__(mm.MissionManagerNode)
    node._home_storage = storage
    node._map_id = "m1"
    node._home_watch = HOME
    node._robot_pose = None if robot is None else SimpleNamespace(x=robot[0], y=robot[1])
    node._pose_cov_xy = cov
    node.logic = SimpleNamespace(
        return_destination=SimpleNamespace(pose=SimpleNamespace(x=HOME[0], y=HOME[1])))
    quiet = SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None)
    node.get_logger = lambda: quiet
    return node


def test_coming_within_thirty_centimetres_counts_as_a_visit(tmp_path):
    node = _node(tmp_path, robot=(HOME[0] + 0.2, HOME[1] + 0.1))   # 0.22 m

    node._check_home_reached()

    home = node._home_storage.read("m1")
    assert (home.visited_ok, home.reach_failed) == (True, False)
    assert node._home_watch is None          # 그 뒤로는 위치마다 파일을 읽거나 쓰지 않는다


def test_passing_further_away_does_not_count(tmp_path):
    node = _node(tmp_path, robot=(HOME[0] + 0.5, HOME[1]))

    node._check_home_reached()

    assert node._home_storage.read("m1").visited_ok is False
    assert node._home_watch == HOME


def test_unknown_position_does_not_count(tmp_path):
    """초기 위치를 잡기 전(분산이 큼)에는 AMCL 이 홈 자리를 가리켜도 믿지 않는다."""
    node = _node(tmp_path, robot=HOME, cov=mm.COV_UNKNOWN + 1.0)

    node._check_home_reached()

    assert node._home_storage.read("m1").visited_ok is False


def test_failed_return_far_from_home_is_unreachable_not_unvisited(tmp_path):
    node = _node(tmp_path, robot=(HOME[0] + 3.0, HOME[1]))
    node._home_storage.record_reach("m1", True)                     # 전에 한 번 가 봤다

    node._record_home_visit(False)

    home = node._home_storage.read("m1")
    assert (home.visited_ok, home.reach_failed) == (True, True)
    assert node._home_watch == HOME          # 다시 들어오면 막힘을 지운다


def test_failed_return_right_at_home_counts_as_a_visit(tmp_path):
    """방향만 못 맞춰 Nav2 가 실패·취소로 끝나도, 홈 30 cm 안이면 다녀온 것이다."""
    node = _node(tmp_path, robot=(HOME[0] + 0.1, HOME[1]))

    node._record_home_visit(False)

    home = node._home_storage.read("m1")
    assert (home.visited_ok, home.reach_failed) == (True, False)


def test_get_home_reports_unreachable_in_message(tmp_path):
    node = _node(tmp_path, robot=(HOME[0] + 3.0, HOME[1]))
    node._record_home_visit(False)
    response = SimpleNamespace()

    node._on_get_home(SimpleNamespace(map_id="m1"), response)

    assert response.exists is True
    assert response.message == HOME_UNREACHABLE_MESSAGE == "홈에 접근할 수 없습니다."

    node._robot_pose = SimpleNamespace(x=HOME[0], y=HOME[1])
    node._check_home_reached()
    node._on_get_home(SimpleNamespace(map_id="m1"), response)
    assert response.message == ""
