#!/usr/bin/env python3
"""Let the operator place Nav2's initial pose from the app, and check it first."""

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
    beam_hits,
    filter_beams,
    MapGrid,
    MIN_BEAMS,
    MIN_MARGIN,
    MIN_SCORE,
    score_pose,
    search_pose,
)

MESSAGES = {
    '': '이 위치로 확정할 수 있습니다.',
    'no_map': '지도가 없습니다. 주행(Nav2)을 먼저 시작하세요.',
    'no_scan': '라이다 값이 안 들어옵니다. 라이다가 켜져 있는지 확인하세요.',
    'no_tf': '로봇과 라이다의 위치 관계를 못 읽었습니다. 잠시 뒤 다시 시도하세요.',
    'few_beams': '라이다가 보는 것이 너무 적습니다. 벽이 보이는 곳으로 옮기세요.',
    'low_score': '지도의 다른 자리 같습니다. 다시 짚어 보세요.',
    'ambiguous': '앞뒤가 비슷해 구분이 안 됩니다. 문이나 교차로가 보이는 곳에서 다시 잡으세요.',
}
YAW_EDGE_NOTE = ' 고른 방향의 끝자락입니다. 옆 방향으로도 확인해 보세요.'
FLIP_NOTE = ' 반대 방향도 비슷하게 맞는 자리입니다. 로봇이 보는 쪽이 맞는지 다시 확인하세요.'


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
        self.declare_parameter('position_sigma_m', 0.25)
        self.declare_parameter('yaw_sigma_deg', 10.0)
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

    def _on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        key = (info.width, info.height, info.resolution,
               info.origin.position.x, info.origin.position.y, len(msg.data))
        with self._lock:
            if key == self._map_key and self._field is not None:
                return
        data = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
        grid = MapGrid(data, info.resolution, info.origin.position.x, info.origin.position.y)
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
        if got.ok and got.margin < self.min_margin:
            response.message += FLIP_NOTE
        response.score = got.score
        response.x, response.y, response.yaw = got.x, got.y, got.yaw
        response.used_beams = int(got.used_beams)
        response.total_beams = int(got.total_beams)
        response.runner_up_score = got.runner_up_score
        response.margin = got.margin
        response.moved_m = got.moved_m
        response.moved_deg = got.moved_deg
        hit_x, hit_y = beam_hits(beams, got.x, got.y, got.yaw, sensor=sensor)
        response.hit_x = [float(v) for v in hit_x]
        response.hit_y = [float(v) for v in hit_y]

        self.get_logger().info(
            '확인 (%.2f, %.2f) -> (%.2f, %.2f, %.1f도) 점수 %.1f 격차 %.1f 빔 %d/%d %s (%.0f ms)'
            % (request.x, request.y, got.x, got.y, math.degrees(got.yaw), got.score,
               got.margin, got.used_beams, got.total_beams,
               'OK' if got.ok else got.reason, elapsed * 1000.0)
        )
        return response

    def _on_commit(self, request, response):
        bundle, reason = self._inputs()
        if bundle is None:
            response.accepted = False
            response.message = MESSAGES[reason]
            return response
        grid, field, beams, sensor = bundle

        if self.initial_pose_pub.get_subscription_count() == 0:
            response.accepted = False
            response.message = '주행(Nav2)이 꺼져 있어 반영할 수 없습니다. Nav2 를 켠 뒤 다시 확정하세요.'
            self.get_logger().warning('확정 거부: /initialpose 구독자가 없다 (Nav2 꺼짐).')
            return response

        with self._lock:
            self._amcl_pose = None

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
        hit_x, hit_y = beam_hits(beams, got_x, got_y, got_yaw, sensor=sensor)
        response.hit_x = [float(v) for v in hit_x]
        response.hit_y = [float(v) for v in hit_y]
        drift = math.hypot(got_x - request.x, got_y - request.y)
        response.message = (
            '반영했습니다. AMCL 재검 %.0f%%, 짚어 준 자세 대비 %.0f cm '
            '(짚은 자리가 틀렸으면 그대로 틀립니다).'
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
        covariance[0] = self.position_sigma ** 2
        covariance[7] = self.position_sigma ** 2
        covariance[35] = self.yaw_sigma ** 2
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
