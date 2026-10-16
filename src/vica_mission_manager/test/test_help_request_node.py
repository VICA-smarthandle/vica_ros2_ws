"""관리자 호출(도움 요청) — 노드 배선: 앱 알림 JSON 과 /vica/mission/help_ack 서비스.

test_delivery_arrival_flag.py 처럼 노드를 rclpy 없이 __new__ 로 맨몸 생성한다. 로직은
test_help_request.py 가 본다.
"""
import json
import uuid
from types import SimpleNamespace

import pytest

mm = pytest.importorskip("vica_mission_manager.mission_manager_node")

from reaction_states import navigating, intent, lookup, BOUNDS  # noqa: E402
from vica_mission_manager.mission_logic import (  # noqa: E402
    MSG_HELP_COMING,
    Destination,
    HelpEvent,
    Pose2D,
    Say,
)

HELP_ID = "6f1c2b0e-8a7d-4c1e-9b2a-3d4e5f607182"
ROOM_407 = Destination(id="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa", name="407호",
                       pose=Pose2D(12.0, -3.0, 0.0, "map"), calibrated=True)


class _Pub:
    def __init__(self):
        self.payloads = []

    def publish(self, msg):
        self.payloads.append(json.loads(msg.data))


class _Log:
    def info(self, *_a, **_k):
        pass

    warn = info


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(mm, "apply_goal_event", lambda *a, **k: False)
    n = mm.MissionManagerNode.__new__(mm.MissionManagerNode)
    n._ledger = object()
    n._map_id = "m1"
    n.pub_goal_event = _Pub()
    n._delivery_dest_id = None
    n.destinations = {ROOM_407.id: ROOM_407}
    n._pose_cov_xy = 0.0
    n._robot_pose = Pose2D(12.34, -3.21, 0.0, "map")
    n.get_logger = lambda: _Log()
    return n


def test_help_requested_carries_id_place_and_robot_position(node):
    node._publish_help_event(HelpEvent("help_requested", HELP_ID))
    msg = node.pub_goal_event.payloads[-1]
    assert msg["event"] == "help_requested" and msg["help_id"] == HELP_ID
    assert msg["place"] == "407호 앞"
    assert (msg["robot_x"], msg["robot_y"]) == (12.34, -3.21)
    # 목적지 칸은 빈 값 — x·y 는 목적지 좌표 칸이라 로봇 위치를 넣지 않는다.
    assert (msg["destination_id"], msg["name"], msg["x"], msg["y"]) == ("", "", 0.0, 0.0)


def test_help_requested_without_pose_sends_nulls(node):
    node._robot_pose = None
    node._publish_help_event(HelpEvent("help_requested", HELP_ID))
    msg = node.pub_goal_event.payloads[-1]
    assert msg["place"] == "" and msg["robot_x"] is None and msg["robot_y"] is None


@pytest.mark.parametrize("event", ["help_acknowledged", "help_canceled"])
def test_ack_and_cancel_carry_only_the_id(node, event):
    node._publish_help_event(HelpEvent(event, HELP_ID))
    msg = node.pub_goal_event.payloads[-1]
    assert msg["event"] == event and msg["help_id"] == HELP_ID
    assert "place" not in msg and "robot_x" not in msg


def _requested_logic():
    logic, t = navigating()
    logic.on_voice_intent(intent("help"), t, lookup, BOUNDS, True)
    logic.on_voice_intent(intent("affirm"), t + 1.0, lookup, BOUNDS, True)
    return logic


def _ack(node, request_id):
    response = SimpleNamespace(accepted=None, message="")
    return node._on_help_ack(SimpleNamespace(request_id=request_id), response)


@pytest.fixture
def ack_node(node):
    node.logic = _requested_logic()
    node._now = lambda: 100.0
    node.ran = []
    node._run_actions = node.ran.extend
    return node


def test_help_ack_service_accepts_the_current_id(ack_node):
    response = _ack(ack_node, ack_node.logic.help_id)
    assert (response.accepted, response.message) == (True, "ok")
    assert Say(MSG_HELP_COMING, priority="response") in ack_node.ran


def test_help_ack_service_reports_other_id(ack_node):
    response = _ack(ack_node, str(uuid.uuid4()))
    assert (response.accepted, response.message) == (False, "id_mismatch")


def test_help_ack_service_reports_no_request(ack_node):
    ack_node.logic, _ = navigating()
    response = _ack(ack_node, str(uuid.uuid4()))
    assert (response.accepted, response.message) == (False, "no_active_help")


def test_help_ack_service_rejects_non_uuid(ack_node):
    response = _ack(ack_node, "not-a-uuid")
    assert response.accepted is False and ack_node.ran == []
