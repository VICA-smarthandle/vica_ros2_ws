"""중간 재측정 노드 배선 — _on_person_detection 이 접근 중 검출을 재측정으로 넘기는지.

08-25 교훈: 함수 안 이름 누락(SpinInPlace)은 구문·import·로직 시험을 다 통과하고 실기 첫 승인에서
노드를 죽였다. 그래서 노드 함수를 실제로 한 번 돌린다. test_nav_cancel_race.py 처럼 rclpy 없이
__new__ 로 맨몸 생성한다.
"""
from types import SimpleNamespace

import pytest

mm = pytest.importorskip("vica_mission_manager.mission_manager_node")

from vica_mission_manager.mission_logic import (  # noqa: E402
    ApproachRequest,
    GateReason,
    MapBounds,
    MissionLogic,
    Navigate,
    Pose2D,
    State,
)

BOUNDS = MapBounds(min_x=-15.1, min_y=-8.59, max_x=10.0, max_y=8.0)


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, msg, **_):
        self.lines.append(msg)

    warn = warning = error = info


def _node(robot):
    node = mm.MissionManagerNode.__new__(mm.MissionManagerNode)
    node.logic = MissionLogic()
    node.logic.robot_pose = robot
    node._robot_pose = robot
    node.map_bounds = BOUNDS
    node._now = lambda: 1.0
    node.ran = []
    node._run_actions = node.ran.extend
    log = _Logger()
    node.get_logger = lambda: log
    node.log = log
    return node


def _msg(x, y, conf=0.9, track=16):
    return SimpleNamespace(
        track_id=track, distance_m=3.0, stable=True, approachable=False, confidence=conf,
        header=SimpleNamespace(frame_id="map"),
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=0.0)),
    )


def test_detections_during_approach_resend_the_goal_once():
    # 사람 (0, −4), 로봇 (0, 2.2) 에서 승인(6.2 m) → (0, −0.9) 에서 다시 재니 사람은 (0, −3.6).
    node = _node(Pose2D(0.0, 2.2, -90.0, "map"))
    _, reason = node.logic.on_approach_request(
        ApproachRequest(goal=Pose2D(0.0, -2.9, -90.0, "map"), track_id=12,
                        person=Pose2D(0.0, -4.0, 0.0, "map")), BOUNDS, True, 0.0)
    assert reason == GateReason.OK
    robot = Pose2D(0.0, -0.9, -90.0, "map")
    node.logic.robot_pose = robot
    node._robot_pose = robot
    for _ in range(7):                       # 다른 번호(16)여도 자리로 같은 사람
        node._on_person_detection(_msg(0.0, -3.6))
    sent = [a for a in node.ran if isinstance(a, Navigate)]
    assert len(sent) == 1
    assert sent[0].destination.pose.y == pytest.approx(-3.6 + 1.1)
    assert node.logic.state == State.APPROACHING
    assert any("다시 보냄" in line for line in node.log.lines)


def test_nan_detection_is_ignored_without_crashing():
    node = _node(Pose2D(0.0, 2.2, -90.0, "map"))
    node.logic.on_approach_request(
        ApproachRequest(goal=Pose2D(0.0, -2.9, -90.0, "map"), track_id=12,
                        person=Pose2D(0.0, -4.0, 0.0, "map")), BOUNDS, True, 0.0)
    node._on_person_detection(_msg(float("nan"), -3.6))
    assert node.ran == []


def test_idle_detection_still_goes_to_the_near_call_path():
    # 접근 중이 아니면 예전처럼 근접 호출 판정으로 간다(탐색 창 밖이면 아무 일도 없음).
    node = _node(Pose2D(0.0, 0.0, 0.0, "map"))
    node._on_person_detection(_msg(0.0, -1.2))
    assert node.logic.state == State.IDLE
    assert node.ran == []
