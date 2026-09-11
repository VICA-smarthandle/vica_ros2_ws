#!/usr/bin/env python3
"""긴급어 → E-stop 래치 체인 배선 (통합 진행순서 ③)."""
from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from vica_interfaces.msg import EmergencyEvent

from .estop_pulse import EstopPulse
from .mission_logic import HARD_EMERGENCY_KEYWORDS


class EmergencyEstopBridge(Node):
    def __init__(self) -> None:
        super().__init__("vica_emergency_estop_bridge")
        self.declare_parameter("pulse_sec", 3.0)
        self.declare_parameter("publish_hz", 5.0)

        self._pulse = EstopPulse(float(self.get_parameter("pulse_sec").value))

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            EmergencyEvent, "/vica/emergency", self._on_emergency, reliable_qos
        )
        self._pub = self.create_publisher(Bool, "/voice_emergency_stop", 10)

        publish_hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / publish_hz, self._publish_loop)
        self.get_logger().info(
            "긴급어 E-stop 브리지 시작: /vica/emergency → /voice_emergency_stop "
            f"(하드 키워드: {sorted(HARD_EMERGENCY_KEYWORDS)}, "
            f"펄스 {self._pulse.pulse_sec}s)"
        )

    def _on_emergency(self, msg: EmergencyEvent) -> None:
        if msg.keyword not in HARD_EMERGENCY_KEYWORDS:
            self.get_logger().info(f"긴급어 '{msg.keyword}' — 하드 정지 대상 아님 (v2 예정)")
            return
        self._pulse.trigger(self._now())
        self.get_logger().warn(
            f"하드 긴급어 '{msg.keyword}' → /voice_emergency_stop 펄스 (원문: {msg.source_text})"
        )
        self._publish_loop()

    def _publish_loop(self) -> None:
        out = Bool()
        out.data = self._pulse.active(self._now())
        self._pub.publish(out)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EmergencyEstopBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
