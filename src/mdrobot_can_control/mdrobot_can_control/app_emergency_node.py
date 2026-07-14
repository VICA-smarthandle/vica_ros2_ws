#!/usr/bin/env python3
"""VICA 앱 비상정지 노드 (서비스 기반).

앱(Flutter, rosbridge)의 비상정지 요청을 받아서
  1) /app_emergency_stop(Bool) 을 발행해 emergency_stop_node -> /emergency_stop
     경로로 모터 노드(keyboard_knob)를 래치 정지시키고,
  2) 현재 Nav2 목적지를 취소해 정지 해제 후에도 자동 재출발하지 않게 하며,
  3) 해제(reset) 시 keyboard_knob 의 /estop_reset 서비스를 호출해 래치를 풀어
     이후 정상 주행 명령이 그대로 반영되게 한다.

이 노드는 소프트웨어 주행 차단기이며, 인증된 물리 비상정지 회로를
대체하지 않는다. 물리 버튼 로직(keyboard_knob 의 F1 비트 감지/래치/
/estop_reset 서비스 본체)은 이 노드가 건드리지 않고 호출만 한다.

인터페이스:
  서비스(std_srvs/Trigger) - 앱이 호출:
    ~activate_service (기본 /app_estop_activate) : 비상정지 활성화
    ~reset_service    (기본 /app_estop_reset)    : 비상정지 해제
  발행:
    /app_emergency_stop (std_msgs/Bool)  : emergency_stop_node 로 넘기는 앱 E-stop 입력
    ~state_topic (기본 /app_estop_state, std_msgs/String JSON, 1Hz)
        앱 재접속 시 현재 활성 여부를 복구하기 위한 상태 브로드캐스트
  호출(클라이언트):
    ~estop_reset_service (기본 /estop_reset, std_srvs/Trigger)
        keyboard_knob 의 래치 해제
    Nav2 <navigate_action>/_action/cancel_goal (action_msgs/CancelGoal)
        현재 활성 목적지 취소

동시성:
  activate/reset 핸들러 안에서 다른 서비스(estop_reset, cancel_goal)를 동기적으로
  기다리므로 MultiThreadedExecutor + ReentrantCallbackGroup 로 실행해야 한다.
  (main() 에서 그렇게 구성한다.)
"""

import json
import threading
import time
from datetime import datetime, timezone
from typing import Any, Optional

import rclpy
from action_msgs.srv import CancelGoal
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class AppEmergencyNode(Node):
    def __init__(self) -> None:
        super().__init__("app_emergency_node")

        self.declare_parameter("activate_service", "/app_estop_activate")
        self.declare_parameter("reset_service", "/app_estop_reset")
        self.declare_parameter("app_emergency_stop_topic", "/app_emergency_stop")
        self.declare_parameter("state_topic", "/app_estop_state")
        self.declare_parameter("estop_reset_service", "/estop_reset")
        self.declare_parameter("navigate_action_name", "/navigate_to_pose")
        # 비상정지 발행 주기(Hz). emergency_stop_node 가 마지막 값을 래치하므로
        # 한 번만 보내도 되지만, 재시작/재접속에도 상태가 유지되도록 반복 발행한다.
        self.declare_parameter("app_estop_publish_hz", 10.0)
        # 상태 브로드캐스트 주기(Hz). 앱 재접속 시 활성 여부 복구용.
        self.declare_parameter("state_publish_hz", 1.0)
        # reset 시 /app_emergency_stop=False 를 emergency_stop_node -> /emergency_stop
        # -> keyboard_knob.ext_estop 까지 전파시킨 뒤 /estop_reset 을 호출하기 위한
        # 대기 시간(초). 이 시간을 두지 않으면 keyboard_knob 가 ext_estop=True 로 보고
        # 해제를 거부한다.
        self.declare_parameter("reset_settle_sec", 0.3)
        # 내부 서비스 호출 대기 제한(초).
        self.declare_parameter("call_timeout_sec", 2.0)

        self.activate_service = str(self.get_parameter("activate_service").value)
        self.reset_service = str(self.get_parameter("reset_service").value)
        app_estop_topic = str(self.get_parameter("app_emergency_stop_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        estop_reset_service = str(self.get_parameter("estop_reset_service").value)
        action_name = str(self.get_parameter("navigate_action_name").value).rstrip("/")
        self.app_estop_publish_hz = float(self.get_parameter("app_estop_publish_hz").value)
        self.state_publish_hz = float(self.get_parameter("state_publish_hz").value)
        self.reset_settle_sec = float(self.get_parameter("reset_settle_sec").value)
        self.call_timeout_sec = float(self.get_parameter("call_timeout_sec").value)

        # 모든 콜백을 같은 reentrant 그룹에 두어, 핸들러가 다른 서비스 응답을
        # 기다리는 동안에도 executor 의 다른 스레드가 그 응답을 처리할 수 있게 한다.
        self.cb_group = ReentrantCallbackGroup()

        self.active = False
        self.last_message = "비상정지가 비활성 상태입니다."

        self.app_estop_publisher = self.create_publisher(Bool, app_estop_topic, 10)
        self.state_publisher = self.create_publisher(String, state_topic, 10)

        self.estop_reset_client = self.create_client(
            Trigger, estop_reset_service, callback_group=self.cb_group
        )
        cancel_service = f"{action_name}/_action/cancel_goal"
        self.cancel_client = self.create_client(
            CancelGoal, cancel_service, callback_group=self.cb_group
        )

        self.create_service(
            Trigger,
            self.activate_service,
            self.handle_activate,
            callback_group=self.cb_group,
        )
        self.create_service(
            Trigger,
            self.reset_service,
            self.handle_reset,
            callback_group=self.cb_group,
        )

        self.create_timer(
            1.0 / self.app_estop_publish_hz,
            self.publish_app_estop,
            callback_group=self.cb_group,
        )
        self.create_timer(
            1.0 / self.state_publish_hz,
            self.publish_state,
            callback_group=self.cb_group,
        )

        self.get_logger().warn(
            "app_emergency_node 는 소프트웨어 비상정지 계층입니다. "
            "물리 비상정지 회로를 대체하지 않습니다."
        )
        self.get_logger().info(
            f"services: activate={self.activate_service}, reset={self.reset_service}"
        )
        self.get_logger().info(
            f"publish: {app_estop_topic} (Bool), {state_topic} (state JSON)"
        )
        self.get_logger().info(
            f"clients: {estop_reset_service} (Trigger), {cancel_service} (CancelGoal)"
        )

    # ------------------------------------------------------------------
    # 앱 서비스 핸들러
    # ------------------------------------------------------------------
    def handle_activate(self, request, response):
        del request
        self.active = True
        self.publish_app_estop()  # 즉시 /app_emergency_stop=True 발행

        cancelled, cancel_note = self.cancel_navigation_goals()
        self.last_message = "비상정지가 활성화되었고 기존 목적지가 취소되었습니다."
        if not cancelled:
            # Nav2 미실행 등으로 취소할 목적지가 없는 경우도 정상 활성화로 본다.
            self.last_message = f"비상정지가 활성화되었습니다. ({cancel_note})"
        self.get_logger().warn(f"E-STOP activate: {self.last_message}")

        response.success = True
        response.message = self.last_message
        return response

    def handle_reset(self, request, response):
        del request
        # 1) 앱 E-stop 입력을 내린다. 타이머도 이후 False 를 계속 발행한다.
        self.active = False
        self.publish_app_estop()

        # 2) /app_emergency_stop=False 가 emergency_stop_node -> /emergency_stop ->
        #    keyboard_knob.ext_estop 까지 전파되도록 잠시 기다린다.
        time.sleep(max(0.0, self.reset_settle_sec))

        # 3) 해제 후 자동 재출발 방지를 위해 남은 목적지를 한 번 더 취소한다.
        self.cancel_navigation_goals()

        # 4) keyboard_knob 래치를 해제한다.
        reset_ok, reset_msg = self.call_estop_reset()
        if not reset_ok:
            self.last_message = f"비상정지 해제 실패: {reset_msg}"
            self.get_logger().error(self.last_message)
            response.success = False
            response.message = self.last_message
            return response

        self.last_message = "비상정지가 해제되었습니다. 기존 목적지는 취소되었습니다."
        self.get_logger().info(f"E-STOP reset: {reset_msg}")
        response.success = True
        response.message = self.last_message
        return response

    # ------------------------------------------------------------------
    # 내부 서비스 호출 (동기 대기)
    # ------------------------------------------------------------------
    def call_estop_reset(self) -> tuple[bool, str]:
        if not self.estop_reset_client.wait_for_service(timeout_sec=1.0):
            return False, "keyboard_knob /estop_reset 서비스를 찾을 수 없습니다."
        result = self._call_sync(self.estop_reset_client, Trigger.Request())
        if result is None:
            return False, "/estop_reset 응답이 시간 내에 오지 않았습니다."
        return bool(result.success), str(result.message)

    def cancel_navigation_goals(self) -> tuple[bool, str]:
        """Nav2 의 모든 활성 목적지를 취소한다. Nav2 미실행은 실패로 보지 않는다."""
        if not self.cancel_client.wait_for_service(timeout_sec=0.5):
            return False, "Nav2 취소 서비스 없음(취소할 목적지 없음)"
        # 빈 goal_info + 0 stamp => 모든 활성 goal 취소.
        result = self._call_sync(self.cancel_client, CancelGoal.Request())
        if result is None:
            return False, "Nav2 취소 응답 시간 초과"
        if result.return_code == CancelGoal.Response.ERROR_NONE:
            return True, "기존 목적지 취소됨"
        if result.return_code == CancelGoal.Response.ERROR_REJECTED:
            # 취소할 활성 goal 이 없을 때도 rejected 로 온다.
            return False, "취소할 활성 목적지 없음"
        return False, f"Nav2 취소 반환코드 {result.return_code}"

    def _call_sync(self, client, request) -> Optional[Any]:
        """call_async 후 응답을 동기적으로 기다린다.

        MultiThreadedExecutor + ReentrantCallbackGroup 환경에서만 안전하다.
        핸들러가 이 함수에서 블로킹하는 동안 다른 executor 스레드가 응답 future 를
        완료시킨다.
        """
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=self.call_timeout_sec):
            return None
        try:
            return future.result()
        except Exception as exc:  # ROS future 예외는 배포판마다 다르다.
            self.get_logger().error(f"service call failed: {exc}")
            return None

    # ------------------------------------------------------------------
    # 주기 발행
    # ------------------------------------------------------------------
    def publish_app_estop(self) -> None:
        msg = Bool()
        msg.data = self.active
        self.app_estop_publisher.publish(msg)

    def publish_state(self) -> None:
        payload = {
            "node": "app_emergency_node",
            "active": self.active,
            "message": self.last_message,
            "timestamp": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.state_publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = AppEmergencyNode()
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
