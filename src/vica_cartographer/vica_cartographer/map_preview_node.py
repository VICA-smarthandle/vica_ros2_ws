#!/usr/bin/env python3
"""Publish a small PNG preview of the map being drawn.

**왜 이 노드가 있는가.** 지도를 그리는 동안 어디를 더 돌아야 하는지 보려면 지도가
자라는 것을 봐야 한다(`scripts/vica_terminator_layout.py` 의 vica_map 프로파일:
"rviz 는 ... 여기서는 필수다"). 그런데 그 확인 수단이 지금은 젯슨 화면의 RViz 뿐이고,
RViz 하나가 코어 1.5개를 쓴다. CPU 가 모자라면 EKF 가 30 Hz 에서 15.5 Hz 로 떨어지고
(devlog/2026-08-01-drive-tuning-and-duplicate-stack.md), Cartographer 는 오도메트리
예측에 기대므로 **보려는 행위가 지도를 직접 나쁘게 만든다.**

그래서 화면을 보내는 대신 지도만 보낸다. 실측 비교(vica_map_0630, 25만 칸):

    RViz 화면 스트리밍   코어 1.5개 + 인코딩
    /map JSON            703 KB/장
    /map cbor-raw        247 KB/장
    이 노드 (PNG)        4.5 KB/장,  1.1 ms/장

**갱신 주기 2초면 충분하다.** 매핑 주행 속도가 0.3 m/s 이므로
(docs/cartographer_corridor_mapping.md 4절) 2초에 60 cm 다. 지도가 그보다 빨리
변하지 않는다.

**파일은 하나만 둔다.** 여러 장으로 쌓으면 30분 매핑에 900장 22 MB 가 되고, 앱 쪽
이미지 캐시(디코딩하면 장당 약 1 MB)가 3분 만에 상한 100 MB 에 닿는다.
"""

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

# 앱이 이 경로로 받아 간다. supervisor_bringup 의 HTTP 서버가 vica_ros2_ws 를
# 서빙하므로 그대로 URL 이 된다.
#
# maps/ 바로 아래가 아니라 _live/ 하위인 것이 중요하다. Supervisor 의
# map_list_node 가 `maps_root.glob("*.png")` 로 목록을 만드는데 **비재귀**라,
# 하위 디렉터리에 두면 미리보기가 저장된 지도인 척 목록에 끼지 않는다.
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
        # 0.5 Hz. 위 주석의 "2초에 60 cm" 근거다. 올리면 젯슨 CPU 를 그만큼 더 쓴다.
        self.declare_parameter('period_sec', 2.0)
        # zlib level. 실측 1: 6,686 B/0.5 ms, 6: 4,533 B/1.1 ms, 9: 3,478 B/6.4 ms.
        self.declare_parameter('compress_level', 6)

        self.period_sec = float(self.get_parameter('period_sec').value)
        self.compress_level = int(self.get_parameter('compress_level').value)
        self.output_dir = self._resolve_output_dir()

        self.seq = 0
        self.last_written_ns = None

        # QoS 주의: TRANSIENT_LOCAL 로 **구독**하면 발행자가 VOLATILE 일 때 아예
        # 매칭되지 않아 한 건도 못 받는다. 이 프로젝트에 같은 사고 기록이 있다
        # (/scan 을 RELIABLE 로 구독해 한 건도 못 받은 건). 받는 쪽은 느슨하게 둔다.
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

        # 앱이 늦게 붙어도 마지막 상태를 받게 latch 한다. 다만 Humble rosbridge 의
        # subscribe op 는 durability 를 지정하지 않아 latch 샘플을 못 받을 수 있으므로
        # (RobotHealth.msg 주석과 같은 사정) 앱은 이것에 의존하지 않는다 —
        # 어차피 period_sec 마다 다시 나간다.
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

    # ------------------------------------------------------------------

    def _resolve_output_dir(self):
        """Decide where to write. 개인 경로를 코드에 박지 않는다."""
        explicit = str(self.get_parameter('output_dir').value).strip()
        if explicit:
            return Path(explicit)
        # scripts/bringup/vica_env.sh 가 세우는 값이다. 스크립트들이 이미 쓴다.
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
            # 시간이 역전되면(음수) 그냥 쓴다. 멈춰 있는 것보다 낫다.
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
        """Write a temp file then rename.

        반쯤 쓰인 파일을 HTTP 서버가 그대로 내보내면 앱에 **잘린 이미지**가 간다.
        같은 디렉터리 안의 rename 은 원자적이라, 보는 쪽은 항상 완성본만 본다.
        노드가 중간에 죽어도 마지막 완성본이 남는다.
        """
        self.output_dir.mkdir(parents=True, exist_ok=True)
        target = self.output_dir / PREVIEW_FILENAME
        temp = self.output_dir / (PREVIEW_FILENAME + '.tmp')
        temp.write_bytes(png)
        os.replace(temp, target)

    def _announce(self, info, size_bytes: int) -> None:
        """Publish the metadata the app needs.

        이미지만으로는 부족하다. 지도가 자라면 크기와 원점이 함께 바뀌므로 앱이
        좌표를 그리려면 매번 같이 받아야 한다.
        """
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
        """Delete the preview file.

        매핑 시작 때와 저장이 확인된 뒤에 mapping_supervisor_node 가 부른다.
        **저장에 실패했을 때는 부르지 않는다** — 2026-08-12 오전에 매핑 아홉 회차가
        저장 실패로 통째로 사라져 원인을 화면 캡쳐에서 겨우 찾은 적이 있다.
        그때 미리보기는 유일하게 남은 그림일 수 있다.
        """
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
