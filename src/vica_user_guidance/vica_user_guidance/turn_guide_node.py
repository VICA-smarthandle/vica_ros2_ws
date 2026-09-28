"""EKF /odom yaw 변화량으로 회전을 판정해 TurnGuide cue를 발행한다.

[안전 경계] 이 노드는 어떤 구동 명령도 발행하지 않는다. /cmd_vel_req, /cmd_vel_safe,
Nav2 goal에 일절 관여하지 않는 순수 판정 계층이다. 판정 결과는 사용자 안내 신호일
뿐이며 로봇의 주행 방향에 영향을 주지 않는다.

[레일 예고 — 2026-09-28] ``enable_rail_forecast`` 가 true 면 Route Server 레일
(``/rail_plan``)에서 45° 이상 꺾이는 코너를 몸이 돌기 약 2초 전에 PREPARE 로 알린다.
yaw 사후 판정은 그대로 돌고, 둘은 ``rail_turn_forecast.RailTurnArbiter`` 가 합친다.
false 면 2026-09-28 이전과 완전히 같다(레일 구독·TF 조회를 아예 만들지 않는다).
레일도 읽기만 한다 — 경로를 바꾸거나 컨트롤러에 무엇을 보내지 않는다.
"""

import math

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from vica_interfaces.msg import TurnGuide

from .rail_turn_forecast import (
    RailTurnArbiter,
    cumulative_lengths,
    find_corners,
    project_to_path,
)
from .timebase import is_fresh_ns, sec_to_ns
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

        # 레일 예고. 기본값은 끔 — ros2 run 으로 띄우면 지금 동작 그대로다.
        self.declare_parameter("enable_rail_forecast", False)
        self.declare_parameter("rail_topic", "/rail_plan")
        self.declare_parameter("rail_timeout_sec", 2.5)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("rail_corner_threshold_deg", 45.0)
        self.declare_parameter("rail_lead_time_sec", 2.0)
        self.declare_parameter("rail_max_offtrack_m", 0.8)
        self.declare_parameter("rail_handoff_m", 2.0)
        self.declare_parameter("rail_merge_gap_m", 1.2)
        self.declare_parameter("rail_goal_change_m", 0.5)
        self.declare_parameter("controller_lookahead_time_sec", 2.5)
        self.declare_parameter("controller_lookahead_min_m", 0.6)
        self.declare_parameter("controller_lookahead_max_m", 1.2)

        odom_topic = self.get_parameter("odom_topic").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.cue_valid_sec = float(self.get_parameter("cue_valid_sec").value)
        exit_threshold_deg = float(self.get_parameter("exit_threshold_deg").value)

        # 파라미터는 도(deg)로 받고 내부는 라디안으로 통일한다. 변환을 여기 한 곳에서만
        # 하면 로직 안에 deg/rad 혼용이 생기지 않는다.
        self.detector = TurnDetector(
            window_ns=sec_to_ns(float(self.get_parameter("window_sec").value)),
            enter_threshold_rad=math.radians(
                float(self.get_parameter("enter_threshold_deg").value)
            ),
            exit_threshold_rad=math.radians(exit_threshold_deg),
            min_duration_ns=sec_to_ns(
                float(self.get_parameter("min_duration_sec").value)
            ),
            odom_timeout_ns=sec_to_ns(
                float(self.get_parameter("odom_timeout_sec").value)
            ),
        )

        # 모든 freshness 판정은 단일 STEADY_TIME clock과 정수 나노초를 쓴다.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self.speed_mps = 0.0
        self.rail_enabled = bool(self.get_parameter("enable_rail_forecast").value)
        if self.rail_enabled:
            self._setup_rail(exit_threshold_deg)

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

    # ── 레일 예고 ───────────────────────────────────────

    def _setup_rail(self, exit_threshold_deg: float) -> None:
        gp = self.get_parameter
        self.rail_topic = str(gp("rail_topic").value)
        self.rail_timeout_ns = sec_to_ns(float(gp("rail_timeout_sec").value))
        self.map_frame = str(gp("map_frame").value)
        self.base_frame = str(gp("base_frame").value)
        self.corner_threshold_deg = float(gp("rail_corner_threshold_deg").value)
        self.merge_gap_m = float(gp("rail_merge_gap_m").value)
        self.goal_change_m = float(gp("rail_goal_change_m").value)
        self.arbiter = RailTurnArbiter(
            lead_time_sec=float(gp("rail_lead_time_sec").value),
            max_offtrack_m=float(gp("rail_max_offtrack_m").value),
            handoff_m=float(gp("rail_handoff_m").value),
            still_turning_deg=exit_threshold_deg,
            controller_lookahead_time_sec=float(gp("controller_lookahead_time_sec").value),
            controller_lookahead_min_m=float(gp("controller_lookahead_min_m").value),
            controller_lookahead_max_m=float(gp("controller_lookahead_max_m").value),
        )
        self.rail_xy = []
        self.rail_cum = []
        self.rail_corners = []
        self.rail_last_ns = None
        self.rail_end = None
        self.s_hint = None
        self.tf_warned = False

        # TF 는 전용 스레드로 받는다. 단일 스레드 spin 에서 /tf(≈135 msg/s)가 밀리면
        # 조회가 늦는다(2026-09-24 run37 드라이버 사건과 같은 이유).
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.create_subscription(Path, self.rail_topic, self.rail_callback, 1)
        self.get_logger().info(
            f"Rail forecast ON: {self.rail_topic} "
            f"(코너 {self.corner_threshold_deg:.0f}° 이상, "
            f"몸이 돌기 {self.arbiter.lead_time_sec:.1f}s 전)"
        )

    def rail_callback(self, msg: Path) -> None:
        """레일 경로를 받는다. BT 가 1 Hz 로 다시 보내도 같은 목적지면 같은 선이다.

        [주의] frame 이 map 이 아니면 쓰지 않는다. 좌표계가 다른 선을 로봇 위치와
        비교하면 엉뚱한 코너를 예고한다.
        """
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            return
        xy = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(xy) < 2:
            return
        end = xy[-1]
        if self.rail_end is None or math.dist(end, self.rail_end) > self.goal_change_m:
            # 레일 끝이 옮겨졌다 = 새 목적지. 알린 코너 기억을 비운다.
            self.arbiter.reset_goal()
        self.rail_end = end
        self.rail_xy = xy
        self.rail_cum = cumulative_lengths(xy)
        self.rail_corners = find_corners(
            xy, threshold_deg=self.corner_threshold_deg, merge_gap_m=self.merge_gap_m
        )
        # 레일은 가까운 노드부터 다시 잘려 오므로 s 기준점이 매번 바뀐다. 힌트를 버린다.
        self.s_hint = None
        self.rail_last_ns = self.now_ns()

    def _robot_pose(self):
        """map 에서 로봇 (x, y, yaw). 못 구하면 None."""
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException as exc:
            if not self.tf_warned:
                self.get_logger().warn(f"레일 예고: TF 조회 실패 — 예고를 쉰다 ({exc})")
                self.tf_warned = True
            return None
        self.tf_warned = False
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def _rail_cue(self, decision, now_ns: int):
        pose = None
        yaw = None
        if len(self.rail_xy) >= 2:
            robot = self._robot_pose()
            if robot is not None:
                pose = project_to_path(self.rail_xy, robot[:2], self.rail_cum, self.s_hint)
                if pose is not None:
                    self.s_hint = pose.s
                yaw = robot[2]
        fresh = is_fresh_ns(self.rail_last_ns, now_ns, self.rail_timeout_ns)
        return self.arbiter.resolve(
            decision, self.rail_corners, pose, abs(self.speed_mps), fresh, yaw
        )

    # ── 공통 ─────────────────────────────────────────

    def now_ns(self) -> int:
        """Return the current STEADY_TIME instant as integer nanoseconds."""
        return self.steady_clock.now().nanoseconds

    def odom_callback(self, msg: Odometry) -> None:
        """yaw를 누적한다.

        [중요] msg.header.stamp가 아니라 수신 시각(STEADY_TIME)을 쓴다. header.stamp는
        SYSTEM_TIME이라 두 시간축을 빼는 것은 정의되지 않은 연산이다.
        """
        q = msg.pose.pose.orientation
        self.speed_mps = msg.twist.twist.linear.x
        self.detector.add_odom(yaw_from_quaternion(q.x, q.y, q.z, q.w), self.now_ns())

    def publish_loop(self) -> None:
        now = self.now_ns()
        decision = self.detector.evaluate(now)
        if self.rail_enabled:
            self.pub_guide.publish(self._cue_to_msg(self._rail_cue(decision, now)))
        else:
            self.pub_guide.publish(self._to_msg(decision))

    def _stamp(self, msg: TurnGuide) -> TurnGuide:
        # header.stamp와 valid_until은 SYSTEM_TIME이다. 로그·rosbag·앱 표시 전용이며
        # 소비자의 stale 판정에 쓰지 않는다.
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "base_footprint"
        msg.valid_until = (
            now + rclpy.duration.Duration(seconds=self.cue_valid_sec)
        ).to_msg()
        return msg

    def _to_msg(self, decision) -> TurnGuide:
        msg = TurnGuide()
        msg.direction = decision.direction
        msg.phase = decision.phase
        msg.distance_m = float("nan")   # 사후 판정에는 거리가 없다
        msg.turn_angle_deg = decision.turn_angle_deg
        msg.sequence_id = decision.sequence_id
        msg.source_stale = decision.source_stale
        return self._stamp(msg)

    def _cue_to_msg(self, cue) -> TurnGuide:
        msg = TurnGuide()
        msg.direction = cue.direction
        msg.phase = cue.phase
        msg.distance_m = float(cue.distance_m)   # PREPARE 에서만 코너까지 거리, 그 밖엔 NaN
        msg.turn_angle_deg = float(cue.turn_angle_deg)
        msg.sequence_id = cue.sequence_id
        msg.source_stale = cue.source_stale
        return self._stamp(msg)

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
        # 종료 통지는 최선 노력이다. 여기서 예외가 나면 종료가 막히므로 반드시 잡는다.
        try:
            node.publish_idle_once()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
