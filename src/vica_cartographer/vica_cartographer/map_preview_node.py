#!/usr/bin/env python3
"""Publish a small PNG preview of the map being drawn."""

from datetime import datetime
import json
import os
from pathlib import Path

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .map_preview import grid_to_png

PREVIEW_DIRNAME = '_live'
PREVIEW_FILENAME = 'preview.png'
PREVIEW_URL_PREFIX = '/maps/' + PREVIEW_DIRNAME + '/'


class MapPreviewNode(Node):
    """Write /map to a PNG on disk and announce it on /vica/map_preview."""

    def __init__(self) -> None:
        """Read parameters, wire the map subscription and the clear service."""
        super().__init__('map_preview_node')

        self.declare_parameter('output_dir', '')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('period_sec', 2.0)
        self.declare_parameter('compress_level', 6)

        self.period_sec = float(self.get_parameter('period_sec').value)
        self.compress_level = int(self.get_parameter('compress_level').value)
        self.output_dir = self._resolve_output_dir()

        self.seq = 0
        self.last_written_ns = None

        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self.handle_map,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )

        self.publisher = self.create_publisher(
            String,
            '/vica/map_preview',
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )

        self.create_service(Trigger, '/vica/map_preview/clear', self.handle_clear)

        if self.output_dir is None:
            self.get_logger().error(
                '미리보기를 저장할 위치를 정하지 못했습니다. '
                'output_dir 파라미터를 주거나 VICA_ROS_WS 환경변수를 설정하세요. '
                '노드는 계속 돌지만 파일을 쓰지 않습니다.'
            )
        else:
            self.get_logger().info(
                f'map_preview_node ready: {self.output_dir} '
                f'({self.period_sec:.1f}초마다, level {self.compress_level})'
            )

    def _resolve_output_dir(self):
        """Decide where to write. 개인 경로를 코드에 박지 않는다."""
        explicit = str(self.get_parameter('output_dir').value).strip()
        if explicit:
            return Path(explicit)
        workspace = os.environ.get('VICA_ROS_WS', '').strip()
        if workspace:
            return Path(workspace) / 'maps' / PREVIEW_DIRNAME
        return None

    def handle_map(self, msg: OccupancyGrid) -> None:
        """Write a preview when the throttle period has elapsed."""
        if self.output_dir is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        if self.last_written_ns is not None:
            elapsed = (now_ns - self.last_written_ns) / 1e9
            if 0 <= elapsed < self.period_sec:
                return
        self.last_written_ns = now_ns

        info = msg.info
        try:
            png = grid_to_png(
                msg.data, info.width, info.height, self.compress_level
            )
        except ValueError as error:
            self.get_logger().warn(f'미리보기를 만들지 못했습니다: {error}')
            return

        try:
            self._write_atomically(png)
        except OSError as error:
            self.get_logger().warn(f'미리보기를 저장하지 못했습니다: {error}')
            return

        self.seq += 1
        self._announce(info, len(png))

    def _write_atomically(self, png: bytes) -> None:
        """Write a temp file then rename."""
        self.output_dir.mkdir(parents=True, exist_ok=True)
        target = self.output_dir / PREVIEW_FILENAME
        temp = self.output_dir / (PREVIEW_FILENAME + '.tmp')
        temp.write_bytes(png)
        os.replace(temp, target)

    def _announce(self, info, size_bytes: int) -> None:
        """Publish the metadata the app needs."""
        message = String()
        message.data = json.dumps(
            {
                'image_url': PREVIEW_URL_PREFIX + PREVIEW_FILENAME,
                'seq': self.seq,
                'width': int(info.width),
                'height': int(info.height),
                'resolution': float(info.resolution),
                'origin_x': float(info.origin.position.x),
                'origin_y': float(info.origin.position.y),
                'bytes': size_bytes,
                'timestamp': datetime.now().isoformat(timespec='seconds'),
            },
            ensure_ascii=False,
        )
        self.publisher.publish(message)

    def handle_clear(self, _request, response):
        """Delete the preview file."""
        if self.output_dir is None:
            response.success = False
            response.message = '저장 위치가 정해지지 않아 지울 것이 없습니다.'
            return response
        removed = 0
        for name in (PREVIEW_FILENAME, PREVIEW_FILENAME + '.tmp'):
            path = self.output_dir / name
            try:
                path.unlink()
                removed += 1
            except FileNotFoundError:
                pass
            except OSError as error:
                response.success = False
                response.message = f'{path} 를 지우지 못했습니다: {error}'
                return response
        self.seq = 0
        self.last_written_ns = None
        response.success = True
        response.message = f'미리보기 {removed}개를 지웠습니다.'
        self.get_logger().info(response.message)
        return response


def main(args=None) -> None:
    """Run the preview node until shutdown."""
    rclpy.init(args=args)
    node = MapPreviewNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
