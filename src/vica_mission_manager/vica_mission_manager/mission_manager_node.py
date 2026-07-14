#!/usr/bin/env python3
"""vica_mission_manager — VICA 음성→주행 통합의 유일한 신규 노드 (진행순서 ②).

역할: /vica/intent(LLM 의 '제안')를 게이트 5조건으로 심사해, 통과한 경우에만
nav2_simple_commander 로 NavigateToPose 를 보낸다. 판단 로직은 전부
mission_logic.py(순수 함수/상태 머신)에 있고, 이 파일은 ROS 배선만 담당한다.

안전 원칙(불변):
- LLM/음성 파트는 /cmd_vel·Nav2 goal 을 직접 발행하지 않는다 — 여기가 유일한 관문.
- 모터 정지의 권위는 /emergency_stop 래치 체인. 여기서의 goal 취소는 보조 경로.

구독: /vica/intent, /vica/emergency(전용 callback group), /emergency_stop
발행: /vica/tts_request(std_msgs/String), /vica/robot_state(1Hz)
"""
from __future__ import annotations

import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from vica_interfaces.msg import EmergencyEvent, RobotState, VicaIntent

from .destinations import load_destinations, load_map_bounds
from .mission_logic import (
    CancelNav,
    IntentData,
    MissionLogic,
    Navigate,
    NavStatus,
    Say,
    State,
    yaw_deg_to_quaternion,
)

try:
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
except ImportError:  # nav2 미설치 환경(예: CI)에서도 import 에러로 죽지 않게
    BasicNavigator = None
    TaskResult = None


class MissionManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("vica_mission_manager")

        # 목적지/지도 파일 경로는 launch 의 ROS parameter(절대경로)로 받는다.
        self.declare_parameter("destinations_yaml", "")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("confirm_timeout_sec", 30.0)
        self.declare_parameter("estop_release_grace_sec", 2.0)
        self.declare_parameter("tick_hz", 5.0)
        # RobotState 층/건물 값 소스는 미결 사항 #5 — 일단 파라미터.
        self.declare_parameter("current_floor", -1)
        self.declare_parameter("current_building", "")

        dest_path = str(self.get_parameter("destinations_yaml").value)
        if not dest_path:
            raise RuntimeError("destinations_yaml parameter 가 비어 있습니다 (launch 에서 절대경로 지정)")
        self.destinations = load_destinations(dest_path)
        self.get_logger().info(f"목적지 {len(self.destinations)}개 로드: {dest_path}")

        map_yaml = str(self.get_parameter("map_yaml").value)
        if map_yaml:
            self.map_bounds = load_map_bounds(map_yaml)
            self.get_logger().info(f"지도 경계: {self.map_bounds}")
        else:
            self.map_bounds = None
            self.get_logger().warn(
                "map_yaml 미지정 — 게이트 ⑤의 지도 경계 검증이 생략됩니다. "
                "실기 운용 전 반드시 지정할 것."
            )

        self.logic = MissionLogic(
            confirm_timeout_sec=float(self.get_parameter("confirm_timeout_sec").value),
            estop_release_grace_sec=float(self.get_parameter("estop_release_grace_sec").value),
        )

        # Nav2 커맨더. 별도 노드로 두고 executor 에 넣지 않는다.
        # 호출은 _nav_lock 으로 직렬화 (goToPose/cancelTask/isTaskComplete 는 짧게 끝남).
        if BasicNavigator is None:
            raise RuntimeError("nav2_simple_commander 를 import 할 수 없습니다")
        self.navigator = BasicNavigator("vica_mission_navigator")
        self._nav_lock = threading.Lock()
        self._nav_active = False  # goToPose 수락 후 완료 전까지 True

        # emergency 계열은 전용 callback group — intent/tick 처리가
        # 긴급 취소를 블로킹하지 않게 한다 (MultiThreadedExecutor 전제).
        self._main_group = MutuallyExclusiveCallbackGroup()
        self._emergency_group = MutuallyExclusiveCallbackGroup()

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(
            VicaIntent, "/vica/intent", self._on_intent, 10, callback_group=self._main_group
        )
        self.create_subscription(
            EmergencyEvent,
            "/vica/emergency",
            self._on_emergency,
            reliable_qos,
            callback_group=self._emergency_group,
        )
        self.create_subscription(
            Bool,
            "/emergency_stop",
            self._on_estop,
            10,
            callback_group=self._emergency_group,
        )

        self.pub_tts = self.create_publisher(String, "/vica/tts_request", 10)
        self.pub_state = self.create_publisher(RobotState, "/vica/robot_state", 10)

        tick_hz = float(self.get_parameter("tick_hz").value)
        self.create_timer(1.0 / tick_hz, self._tick, callback_group=self._main_group)
        self.create_timer(1.0, self._publish_robot_state, callback_group=self._main_group)

        self.get_logger().info("vica_mission_manager 시작 (상태: idle)")

    # -- 콜백 -------------------------------------------------------------------

    def _on_intent(self, msg: VicaIntent) -> None:
        intent = IntentData(
            intent=msg.intent,
            matched_destination_id=msg.matched_destination_id,
            need_confirm=msg.need_confirm,
            safety_flag=msg.safety_flag,
        )
        dest = self.destinations.get(msg.matched_destination_id) or None
        actions = self.logic.on_intent(
            intent, dest, self.map_bounds, self._nav2_ready(), self._now()
        )
        self.get_logger().info(
            f"intent={msg.intent} dest={msg.matched_destination_id or '-'} "
            f"confirm={msg.need_confirm} -> state={self.logic.state.value}"
        )
        self._run_actions(actions)

    def _on_emergency(self, msg: EmergencyEvent) -> None:
        self.get_logger().warn(f"긴급어 수신: '{msg.keyword}' (원문: {msg.source_text})")
        actions = self.logic.on_emergency(msg.keyword, self._now())
        self._run_actions(actions)

    def _on_estop(self, msg: Bool) -> None:
        actions = self.logic.on_estop(bool(msg.data), self._now())
        if actions:
            self.get_logger().warn(f"/emergency_stop={msg.data} -> state={self.logic.state.value}")
        self._run_actions(actions)

    def _tick(self) -> None:
        actions = self.logic.on_tick(self._now(), self._poll_nav_status())
        self._run_actions(actions)

    def _publish_robot_state(self) -> None:
        msg = RobotState()
        msg.current_floor = int(self.get_parameter("current_floor").value)
        msg.current_building = str(self.get_parameter("current_building").value)
        msg.is_moving = self.logic.state == State.NAVIGATING
        self.pub_state.publish(msg)

    # -- Action 실행 -------------------------------------------------------------

    def _run_actions(self, actions) -> None:
        for action in actions:
            if isinstance(action, Say):
                out = String()
                # ros_tts_node 큐 우선순위 접두어 (vica-voice-llm src/tts_queue.py 계약)
                out.data = f"{action.priority}:{action.text}"
                self.pub_tts.publish(out)
                self.get_logger().info(f"TTS[{action.priority}]: {action.text}")
            elif isinstance(action, CancelNav):
                self._cancel_nav()
            elif isinstance(action, Navigate):
                self._start_nav(action)

    def _start_nav(self, action: Navigate) -> None:
        dest = action.destination
        goal = PoseStamped()
        goal.header.frame_id = dest.pose.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = dest.pose.x
        goal.pose.position.y = dest.pose.y
        qx, qy, qz, qw = yaw_deg_to_quaternion(dest.pose.yaw_deg)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        with self._nav_lock:
            accepted = self.navigator.goToPose(goal)
            self._nav_active = bool(accepted)
        if accepted:
            self.get_logger().info(
                f"NavigateToPose 전송: {dest.id} ({dest.pose.x:.2f}, {dest.pose.y:.2f}, "
                f"{dest.pose.yaw_deg:.1f}deg)"
            )
        else:
            # goal 거부 → 다음 tick 에서 FAILED 처리되도록 상태를 만든다.
            self.get_logger().error(f"NavigateToPose goal 거부됨: {dest.id}")
            self._run_actions(self.logic.on_tick(self._now(), NavStatus.FAILED))

    def _cancel_nav(self) -> None:
        with self._nav_lock:
            if self._nav_active:
                self.navigator.cancelTask()
                self._nav_active = False
        self.get_logger().warn("Nav2 goal 취소 (보조 경로 — 모터 정지 권위는 래치 체인)")

    def _poll_nav_status(self) -> NavStatus:
        if not self._nav_active:
            return NavStatus.NONE
        with self._nav_lock:
            if not self.navigator.isTaskComplete():
                return NavStatus.RUNNING
            self._nav_active = False
            result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            return NavStatus.SUCCEEDED
        if result == TaskResult.CANCELED:
            return NavStatus.CANCELED
        return NavStatus.FAILED

    # -- 유틸 --------------------------------------------------------------------

    def _nav2_ready(self) -> bool:
        """Nav2 미준비 시 goal 거부용. goToPose 내부의 무한 대기를 피하려고
        action server 준비 여부만 즉시 확인한다."""
        try:
            return bool(self.navigator.nav_to_pose_client.server_is_ready())
        except AttributeError:
            return False

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
