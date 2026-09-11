"""EKF /odom yaw 변화량으로 회전을 판정해 TurnGuide cue를 발행한다."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

from vica_interfaces.msg import TurnGuide

from .timebase import sec_to_ns
from .turn_detector import TurnDetector, yaw_from_quaternion


class TurnGuideNode(Node):
    """/odom을 구독해 /vica/turn_guide를 발행한다."""

    def __init__(self) -> None:
        super().__init__("turn_guide_node")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("window_sec", 1.5)
        self.declare_parameter("enter_threshold_deg", 20.0)
        self.declare_parameter("exit_threshold_deg", 10.0)
        self.declare_parameter("min_duration_sec", 0.6)
        self.declare_parameter("odom_timeout_sec", 0.5)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("cue_valid_sec", 2.0)

        odom_topic = self.get_parameter("odom_topic").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.cue_valid_sec = float(self.get_parameter("cue_valid_sec").value)

        self.detector = TurnDetector(
            window_ns=sec_to_ns(float(self.get_parameter("window_sec").value)),
            enter_threshold_rad=math.radians(
                float(self.get_parameter("enter_threshold_deg").value)
            ),
            exit_threshold_rad=math.radians(
                float(self.get_parameter("exit_threshold_deg").value)
            ),
            min_duration_ns=sec_to_ns(
                float(self.get_parameter("min_duration_sec").value)
            ),
            odom_timeout_ns=sec_to_ns(
                float(self.get_parameter("odom_timeout_sec").value)
            ),
        )

        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self.pub_guide = self.create_publisher(TurnGuide, "/vica/turn_guide", 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.create_timer(
            1.0 / publish_rate_hz,
            self.publish_loop,
            clock=self.steady_clock,
        )

        self.get_logger().info(f"Subscribed: {odom_topic}")
        self.get_logger().info("Publishing: /vica/turn_guide")
        self.get_logger().info(
            "This node publishes guidance cues only; it never commands motion."
        )

    def now_ns(self) -> int:
        """Return the current STEADY_TIME instant as integer nanoseconds."""
        return self.steady_clock.now().nanoseconds

    def odom_callback(self, msg: Odometry) -> None:
        """yaw를 누적한다."""
        q = msg.pose.pose.orientation
        self.detector.add_odom(yaw_from_quaternion(q.x, q.y, q.z, q.w), self.now_ns())

    def publish_loop(self) -> None:
        decision = self.detector.evaluate(self.now_ns())
        self.pub_guide.publish(self._to_msg(decision))

    def _to_msg(self, decision) -> TurnGuide:
        msg = TurnGuide()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "base_footprint"
        msg.direction = decision.direction
        msg.phase = decision.phase
        msg.distance_m = float("nan")
        msg.turn_angle_deg = decision.turn_angle_deg
        msg.sequence_id = decision.sequence_id
        msg.valid_until = (
            now + rclpy.duration.Duration(seconds=self.cue_valid_sec)
        ).to_msg()
        msg.source_stale = decision.source_stale
        return msg

    def publish_idle_once(self) -> None:
        """종료 직전 IDLE을 1회 발행해 소비자가 회전 상태에 갇히지 않게 한다."""
        msg = TurnGuide()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "base_footprint"
        msg.direction = TurnGuide.DIRECTION_NONE
        msg.phase = TurnGuide.PHASE_IDLE
        msg.distance_m = float("nan")
        msg.turn_angle_deg = 0.0
        msg.sequence_id = self.detector.sequence_id
        msg.valid_until = now.to_msg()
        msg.source_stale = True
        self.pub_guide.publish(msg)


def main(args=None) -> None:
    """Run the VICA turn guide node."""
    rclpy.init(args=args)
    node = TurnGuideNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_idle_once()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
