"""사람 접근 goal 만 레일 없는 트리를 쓰는 규칙 (2026-10-02, run60)."""
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
