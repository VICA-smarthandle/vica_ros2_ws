"""사람 접근 goal 만 레일 없는 트리를 쓰는 규칙 (2026-10-02, run60).

run60: 사람은 시작 지점 정남쪽 2.6 m 였는데 접근 goal 이 레일 트리로 가서 레일을
8.5 m 돌고 87 s 맴돌았다. 접근 goal 은 vica_navigate_to_pose_approach.xml 로,
목적지·홈 복귀는 그대로 기본 트리(빈 문자열)로 보낸다.
"""
from vica_mission_manager.mission_logic import (
    APPROACH_BT_FILE,
    APPROACH_DESTINATION_PREFIX,
    ApproachRequest,
    Pose2D,
    approach_destination,
    nav_behavior_tree,
)

BT = f"/ws/share/vica_nav2/behavior_trees/{APPROACH_BT_FILE}"


def _approach_id(track_id=7):
    request = ApproachRequest(
        goal=Pose2D(x=1.0, y=0.5, yaw_deg=30.0, frame_id="map"),
        track_id=track_id,
        approachable=True,
    )
    return approach_destination(request).id


def test_approach_goal_uses_the_approach_tree():
    assert _approach_id().startswith(APPROACH_DESTINATION_PREFIX)
    assert nav_behavior_tree(_approach_id(), BT) == BT


def test_registered_destination_keeps_the_default_rail_tree():
    assert nav_behavior_tree("acee7771-920f-4c72-bd29-008d3b9e3b54", BT) == ""


def test_home_return_keeps_the_default_rail_tree():
    assert nav_behavior_tree("__home__", BT) == ""


def test_empty_setting_turns_it_off_for_approach_too():
    assert nav_behavior_tree(_approach_id(), "") == ""
