#!/usr/bin/env python3
"""Let the operator place Nav2's initial pose from the app, and check it first.

**왜 필요한가.** Nav2 를 켜면 AMCL 은 자기가 어디 있는지 모른다. 지금까지는 젯슨
화면의 RViz 에서 2D Pose Estimate 로 찍어 줬다. 관리자 앱만 들고 현장에 나가면
그 일을 할 방법이 없다.

**왜 그냥 /initialpose 를 발행하지 않는가.** 발행하는 순간 되돌릴 수 없다.
잘못 찍으면 로봇이 엉뚱한 곳에 있다고 믿고 주행을 시작한다. 그래서 확인과 확정을
나눴다. 확인(/vica/pose_check)은 AMCL 을 건드리지 않고 점수만 계산하고,
확정(/vica/pose_commit)에서만 반영한다.

**왜 앱이 아니라 노드가 /scan 을 받는가.** /scan 은 10 Hz x 721 점이다. rosbridge
로 앱까지 끌고 오면 대역폭도 낭비지만, 더 큰 문제는 앱에 지도 PNG 그림만 있고
원본 격자값이 없다는 것이다. 그림에서 회색 픽셀을 세는 것은 정확한 계산이 아니다.
앱은 숫자 셋(x, y, yaw)을 보내고 % 하나를 받는다. 오가는 데이터가 200 B 남짓이다.
지도 미리보기에서 JSON 703 KB 대신 PNG 4.5 KB 를 고른 것과 같은 판단이다.

채점 방식과 근거는 pose_score.py 에 적었다.
"""

import math
import threading
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import tf2_ros
from vica_interfaces.srv import PoseCheck, PoseCommit

from .pose_score import (
    build_likelihood_field,
    filter_beams,
    MapGrid,
    MIN_BEAMS,
    MIN_MARGIN,
    MIN_SCORE,
    score_pose,
    search_pose,
)

# 앱이 그대로 띄우는 문구다. 숫자가 아니라 **다음에 할 행동**으로 쓴다.
# 관리자는 % 의 의미를 모른다.
MESSAGES = {
    '': '이 위치로 확정할 수 있습니다.',
    'no_map': '지도가 없습니다. 주행(Nav2)을 먼저 시작하세요.',
    'no_scan': '라이다 값이 안 들어옵니다. 라이다가 켜져 있는지 확인하세요.',
    'no_tf': '로봇과 라이다의 위치 관계를 못 읽었습니다. 잠시 뒤 다시 시도하세요.',
    'few_beams': '라이다가 보는 것이 너무 적습니다. 벽이 보이는 곳으로 옮기세요.',
    'low_score': '지도의 다른 자리 같습니다. 다시 짚어 보세요.',
    # 대칭 복도는 움직여도 대칭이다. 대칭을 깨는 것은 거리가 아니라 비대칭한 물건이다.
    'ambiguous': '앞뒤가 비슷해 구분이 안 됩니다. 문이나 교차로가 보이는 곳에서 다시 잡으세요.',
}
YAW_EDGE_NOTE = ' 고른 방향의 끝자락입니다. 옆 방향으로도 확인해 보세요.'


class PoseBootstrapNode(Node):
    """Score initial-pose candidates and, on request, hand one to AMCL."""

    def __init__(self) -> None:
        """Wire the map/scan inputs, TF, and the two services."""
        super().__init__('pose_bootstrap_node')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('min_score', MIN_SCORE)
        self.declare_parameter('min_beams', MIN_BEAMS)
        self.declare_parameter('min_margin', MIN_MARGIN)
        # /initialpose 에 실어 보낼 신뢰도. recovery_alpha_fast/slow 가 0.0 이라
        # AMCL 은 스스로 전역 재초기화를 하지 않는다. 우리가 넣은 값이 곧 출발점이라
        # 넓게 퍼뜨릴 이유가 없다.
        self.declare_parameter('position_sigma_m', 0.1)
        self.declare_parameter('yaw_sigma_deg', 3.0)
        # update_min_d 가 0.10 이라 로봇이 10 cm 움직이기 전에는 AMCL 이 갱신하지
        # 않는다. 서 있는 채로 확정하면 아무 일도 안 일어난 것처럼 보인다.
        self.declare_parameter('nomotion_delay_sec', 0.5)
        self.declare_parameter('settle_sec', 2.0)

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.min_score = float(self.get_parameter('min_score').value)
        self.min_beams = int(self.get_parameter('min_beams').value)
        self.min_margin = float(self.get_parameter('min_margin').value)
        self.position_sigma = float(self.get_parameter('position_sigma_m').value)
        self.yaw_sigma = math.radians(float(self.get_parameter('yaw_sigma_deg').value))
        self.nomotion_delay_sec = float(self.get_parameter('nomotion_delay_sec').value)
        self.settle_sec = float(self.get_parameter('settle_sec').value)

        self._lock = threading.Lock()
        self._grid = None
        self._field = None
        self._map_key = None
        self._scan = None
        self._amcl_pose = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        map_topic = str(self.get_parameter('map_topic').value)
        # QoS 함정. map_server 는 TRANSIENT_LOCAL 로 지도를 한 번만 걸어 두고,
        # cartographer 의 occupancy_grid_node 는 VOLATILE 로 계속 흘린다.
        # 한쪽만 구독하면 다른 쪽에서는 **에러 없이 조용히** 한 건도 못 받는다.
        # 이 프로젝트에 같은 사고가 두 번 있었다(map_preview_node 주석 참고).
        # 그래서 양쪽을 다 건다. 지도는 자주 바뀌지 않아 비용이 없다.
        for durability in (DurabilityPolicy.TRANSIENT_LOCAL, DurabilityPolicy.VOLATILE):
            self.create_subscription(
                OccupancyGrid, map_topic, self._on_map,
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=durability,
                    history=HistoryPolicy.KEEP_LAST,
                ),
            )

        # 라이다 드라이버는 BEST_EFFORT 다. RELIABLE 로 구독하면 매칭이 안 된다.
        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value), self._on_scan,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10,
        )

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10,
        )

        # 확인은 0.1초 안에 끝나지만 확정은 2.5초 넘게 기다린다. 그동안 지도·스캔
        # 구독이 멈추면 안 되므로 서비스만 따로 묶고 MultiThreadedExecutor 로 돈다.
        service_group = MutuallyExclusiveCallbackGroup()
        self.create_service(
            PoseCheck, '/vica/pose_check', self._on_check, callback_group=service_group,
        )
        self.create_service(
            PoseCommit, '/vica/pose_commit', self._on_commit, callback_group=service_group,
        )
        self.nomotion_client = self.create_client(
            Empty, '/request_nomotion_update', callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            'pose_bootstrap_node 준비. 합격선 점수 %.0f · 빔 %d · 격차 %.0f'
            % (self.min_score, self.min_beams, self.min_margin)
        )

    # --- 입력 --------------------------------------------------------------

    def _on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        key = (info.width, info.height, info.resolution,
               info.origin.position.x, info.origin.position.y, len(msg.data))
        with self._lock:
            if key == self._map_key and self._field is not None:
                return
        data = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
        grid = MapGrid(data, info.resolution, info.origin.position.x, info.origin.position.y)
        # 거리표는 여기서 한 번만 만든다. 이후 채점은 배열 인덱싱이라 사실상 공짜다.
        field = build_likelihood_field(grid)
        with self._lock:
            self._grid, self._field, self._map_key = grid, field, key
        self.get_logger().info('지도 %dx%d 를 받아 거리표를 만들었다.' % (info.width, info.height))

    def _on_scan(self, msg: LaserScan) -> None:
        with self._lock:
            self._scan = msg

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        with self._lock:
            self._amcl_pose = msg

    # --- 센서 위치 ---------------------------------------------------------

    def _sensor_offset(self, frame_id: str):
        """base_footprint 에서 본 라이다 위치. 18.5 cm 앞이라 빼먹으면 그만큼 밀린다."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, frame_id, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().warning('TF %s -> %s 실패: %s' % (self.base_frame, frame_id, exc))
            return None
        t = tf.transform.translation
        return (t.x, t.y, _yaw_of(tf.transform.rotation))

    def _inputs(self):
        """Return (grid, field, beams, sensor) or (reason, None)."""
        with self._lock:
            grid, field, scan = self._grid, self._field, self._scan
        if grid is None or field is None:
            return None, 'no_map'
        if scan is None:
            return None, 'no_scan'
        sensor = self._sensor_offset(scan.header.frame_id)
        if sensor is None:
            return None, 'no_tf'
        beams = filter_beams(scan.ranges, scan.angle_min, scan.angle_increment)
        return (grid, field, beams, sensor), ''

    # --- 확인 --------------------------------------------------------------

    def _on_check(self, request, response):
        bundle, reason = self._inputs()
        if bundle is None:
            response.ok = False
            response.reason = reason
            response.message = MESSAGES[reason]
            return response

        grid, field, beams, sensor = bundle
        hint = float(request.yaw_hint) if request.has_yaw_hint else None
        started = time.monotonic()
        got = search_pose(
            field, grid, beams, float(request.x), float(request.y),
            yaw_hint=hint, sensor=sensor,
            min_score=self.min_score, min_beams=self.min_beams, min_margin=self.min_margin,
        )
        elapsed = time.monotonic() - started

        response.ok = got.ok
        response.reason = got.reason
        response.message = MESSAGES.get(got.reason, got.reason)
        if got.at_yaw_edge:
            response.message += YAW_EDGE_NOTE
        response.score = got.score
        response.x, response.y, response.yaw = got.x, got.y, got.yaw
        response.used_beams = int(got.used_beams)
        response.total_beams = int(got.total_beams)
        response.runner_up_score = got.runner_up_score
        response.margin = got.margin
        response.moved_m = got.moved_m
        response.moved_deg = got.moved_deg

        self.get_logger().info(
            '확인 (%.2f, %.2f) -> (%.2f, %.2f, %.1f도) 점수 %.1f 격차 %.1f 빔 %d/%d %s (%.0f ms)'
            % (request.x, request.y, got.x, got.y, math.degrees(got.yaw), got.score,
               got.margin, got.used_beams, got.total_beams,
               'OK' if got.ok else got.reason, elapsed * 1000.0)
        )
        return response

    # --- 확정 --------------------------------------------------------------

    def _on_commit(self, request, response):
        bundle, reason = self._inputs()
        if bundle is None:
            response.accepted = False
            response.message = MESSAGES[reason]
            return response
        grid, field, beams, sensor = bundle

        self.initial_pose_pub.publish(
            self._initial_pose_msg(float(request.x), float(request.y), float(request.yaw))
        )
        self.get_logger().info(
            '/initialpose 발행 (%.2f, %.2f, %.1f도)'
            % (request.x, request.y, math.degrees(request.yaw))
        )

        time.sleep(self.nomotion_delay_sec)
        if self.nomotion_client.service_is_ready():
            self.nomotion_client.call_async(Empty.Request())
        else:
            self.get_logger().warning(
                '/request_nomotion_update 가 없다. 로봇이 10 cm 움직여야 AMCL 이 갱신된다.'
            )

        time.sleep(self.settle_sec)
        with self._lock:
            amcl = self._amcl_pose
        if amcl is None:
            response.accepted = True
            response.message = '반영했습니다. AMCL 자세를 아직 못 읽어 검증은 못 했습니다.'
            return response

        got_x = amcl.pose.pose.position.x
        got_y = amcl.pose.pose.position.y
        got_yaw = _yaw_of(amcl.pose.pose.orientation)
        verify = score_pose(field, grid, beams, got_x, got_y, got_yaw, sensor=sensor)

        response.accepted = True
        response.verify_score = verify
        response.amcl_x, response.amcl_y, response.amcl_yaw = got_x, got_y, got_yaw
        drift = math.hypot(got_x - request.x, got_y - request.y)
        response.message = (
            '반영했습니다. AMCL 자세 재검 %.0f%%, 요청한 자리에서 %.0f cm 차이입니다.'
            % (verify, drift * 100.0)
        )
        self.get_logger().info(response.message)
        return response

    def _initial_pose_msg(self, x: float, y: float, yaw: float) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        covariance = [0.0] * 36
        covariance[0] = self.position_sigma ** 2      # x
        covariance[7] = self.position_sigma ** 2      # y
        covariance[35] = self.yaw_sigma ** 2          # yaw
        msg.pose.covariance = covariance
        return msg


def _yaw_of(quaternion) -> float:
    """Yaw from a quaternion. 2D 라 z·w 만 있으면 된다."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
    )


def main(args=None) -> None:
    """Spin the node on a multi-threaded executor."""
    rclpy.init(args=args)
    node = PoseBootstrapNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
