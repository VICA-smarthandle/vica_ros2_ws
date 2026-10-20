#!/usr/bin/env python3
"""Own the mapping stack so the app can start, stop and save it."""

from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vica_interfaces.srv import SaveMap

from .mapping_session import (
    blocking_reason,
    duplicated_names,
    is_stack_up,
    map_meta_document,
    MappingState,
    missing_prerequisites,
    parse_align_result,
    plan_map_save,
    running_stacks,
    save_label,
    save_script_command,
    StopEscalation,
)

DEFAULT_LAUNCH = [
    'ros2', 'launch', 'vica_cartographer', 'vica_mapping_bringup.launch.py',
]

DEFAULT_PREREQUISITES = [
    'camera/camera',
    'imu_base_link_adapter',
    'mdrobot_can_keyboard_knob_node',
]


class MappingSupervisorNode(Node):
    """Start, watch, stop and save one mapping session."""

    def __init__(self) -> None:
        """Declare parameters and wire services."""
        super().__init__('mapping_supervisor_node')

        self.declare_parameter('launch_command', DEFAULT_LAUNCH)
        self.declare_parameter('save_script', '')
        self.declare_parameter('maps_dir', '')
        self.declare_parameter('prerequisite_nodes', DEFAULT_PREREQUISITES)
        self.declare_parameter('status_period_sec', 1.0)
        self.declare_parameter('startup_timeout_sec', 40.0)
        self.declare_parameter('shutdown_grace_sec', 8.0)

        self.state = MappingState.IDLE
        self.detail = ''
        self.map_id = ''
        self.map_name = ''
        self.started_at = None
        self.save_align = {}
        self._process = None
        self._pgid = None
        self._stop = None
        self._lock = threading.Lock()

        self._main_group = MutuallyExclusiveCallbackGroup()

        self.status_publisher = self.create_publisher(
            String,
            '/vica/mapping_status',
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )

        self.create_service(
            Trigger, '/vica/mapping/start', self.handle_start,
            callback_group=self._main_group,
        )
        self.create_service(
            Trigger, '/vica/mapping/stop', self.handle_stop,
            callback_group=self._main_group,
        )
        self.create_service(
            SaveMap, '/vica/mapping/save', self.handle_save,
            callback_group=self._main_group,
        )

        self._preview_clear = self.create_client(
            Trigger, '/vica/map_preview/clear'
        )

        period = float(self.get_parameter('status_period_sec').value)
        self.create_timer(period, self._tick, callback_group=self._main_group)

        self.get_logger().info('mapping_supervisor_node ready')

    def _node_names(self):
        """Return every node name on the graph, duplicates included."""
        try:
            return [
                f'{namespace.rstrip("/")}/{name}'
                for name, namespace in self.get_node_names_and_namespaces()
            ]
        except Exception as error:  # noqa: BLE001 - 그래프 조회는 실패해도 계속 돈다
            self.get_logger().warn(f'노드 목록을 읽지 못했습니다: {error}')
            return []

    def _set_state(self, state: MappingState, detail: str = '') -> None:
        self.state = state
        self.detail = detail
        self._publish_status()

    def _publish_status(self) -> None:
        names = self._node_names()
        stacks = running_stacks(names)
        message = String()
        message.data = json.dumps(
            {
                'state': self.state.value,
                'detail': self.detail,
                'map_id': self.map_id,
                'map_name': self.map_name or self.map_id,
                'started_at': self.started_at or '',
                'save_align': self.save_align,
                'nav2_running': stacks['nav2'],
                'mapping_running': stacks['mapping'],
                'duplicated': duplicated_names(names),
                'prerequisites_missing': missing_prerequisites(
                    names,
                    list(self.get_parameter('prerequisite_nodes').value),
                ),
                'timestamp': datetime.now().isoformat(timespec='seconds'),
            },
            ensure_ascii=False,
        )
        self.status_publisher.publish(message)

    def _tick(self) -> None:
        with self._lock:
            self._advance()
        self._publish_status()

    def _advance(self) -> None:
        """Move the state machine forward. 반드시 _lock 을 잡고 부른다."""
        if self._stop is not None:
            self._advance_stop()
            return
        if self.state is MappingState.STARTING:
            if self._process is not None and self._process.poll() is not None:
                self._set_state(
                    MappingState.ERROR,
                    f'launch 가 곧바로 끝났습니다(코드 {self._process.returncode}).',
                )
                self._forget_process()
                return
            if is_stack_up(self._node_names()):
                self._set_state(MappingState.MAPPING, '지도를 그리는 중입니다.')
                return
            timeout = float(self.get_parameter('startup_timeout_sec').value)
            if self._elapsed_sec() > timeout:
                self._set_state(
                    MappingState.ERROR,
                    f'{timeout:.0f}초 안에 cartographer 가 올라오지 않았습니다.',
                )
                self._begin_stop()
        elif self.state is MappingState.MAPPING:
            if self._process is not None and self._process.poll() is not None:
                self._set_state(
                    MappingState.ERROR,
                    '매핑 프로세스가 예상치 못하게 종료됐습니다.',
                )
                self._forget_process()

    def _elapsed_sec(self) -> float:
        if not self.started_at:
            return 0.0
        return (datetime.now() - datetime.fromisoformat(self.started_at)).total_seconds()

    def handle_start(self, _request, response):
        """Start the mapping stack after checking nothing conflicts."""
        with self._lock:
            reason = blocking_reason(
                self.state,
                self._node_names(),
                list(self.get_parameter('prerequisite_nodes').value),
            )
            if reason:
                response.success = False
                response.message = reason
                self.get_logger().warn(f'매핑 시작 거부: {reason}')
                return response

            command = list(self.get_parameter('launch_command').value)
            command.append('start_motor:=false')
            try:
                self._process = subprocess.Popen(  # noqa: S603
                    command, start_new_session=True
                )
                self._pgid = os.getpgid(self._process.pid)
            except OSError as error:
                response.success = False
                response.message = f'매핑을 시작하지 못했습니다: {error}'
                self._set_state(MappingState.ERROR, response.message)
                return response

            self.map_id = ''
            self.map_name = ''
            self.save_align = {}
            self.started_at = datetime.now().isoformat(timespec='seconds')
            self._set_state(MappingState.STARTING, '스택을 띄우는 중입니다.')

        self._request_preview_clear()
        response.success = True
        response.message = '매핑을 시작했습니다.'
        self.get_logger().info(f'매핑 시작: {" ".join(command)}')
        return response

    def handle_stop(self, _request, response):
        """Stop the mapping stack. 저장하지 않는다 — 저장은 별도 요청이다."""
        with self._lock:
            if self._process is None:
                self._set_state(MappingState.IDLE, '')
                response.success = True
                response.message = '실행 중인 매핑이 없습니다.'
                return response
            if self._stop is not None:
                response.success = True
                response.message = '이미 정리하는 중입니다.'
                return response
            self._set_state(MappingState.STOPPING, '정리하는 중입니다.')
            self._begin_stop()
        response.success = True
        response.message = '매핑 종료를 시작했습니다. 완료는 상태로 알립니다.'
        return response

    def handle_save(self, request, response):
        """Accept a save request and run the script off the callback thread."""
        with self._lock:
            if self.state is not MappingState.MAPPING:
                response.accepted = False
                response.message = f'지금은 저장할 수 없습니다(상태: {self.state.value}).'
                return response

            now = datetime.now()
            map_id, display_name, error = plan_map_save(
                request.name, now.strftime('%m%d'), now.strftime('%H%M%S')
            )
            if map_id is None:
                response.accepted = False
                response.message = error
                return response

            script = self._resolve_save_script()
            if script is None:
                response.accepted = False
                response.message = (
                    '저장 스크립트를 찾지 못했습니다. save_script 파라미터를 주거나 '
                    'VICA_ROOT 환경변수를 설정하세요.'
                )
                return response

            align = bool(getattr(request, 'align', False))
            self.map_id = map_id
            self.map_name = display_name
            self.save_align = {}
            label = save_label(map_id, display_name)
            self._set_state(MappingState.SAVING, f'{label} 저장 중입니다.')

        threading.Thread(
            target=self._run_save,
            args=(str(script), map_id, display_name, align),
            name='vica_map_save',
            daemon=True,
        ).start()

        response.accepted = True
        response.message = f'{label} 저장을 시작했습니다.'
        response.map_id = map_id
        return response

    def _resolve_save_script(self):
        explicit = str(self.get_parameter('save_script').value).strip()
        if explicit:
            path = Path(explicit)
            return path if path.exists() else None
        root = os.environ.get('VICA_ROOT', '').strip()
        if not root:
            return None
        path = Path(root) / 'scripts' / 'vica_map_save.sh'
        return path if path.exists() else None

    def _resolve_maps_dir(self):
        explicit = str(self.get_parameter('maps_dir').value).strip()
        if explicit:
            return Path(explicit)
        workspace = os.environ.get('VICA_ROS_WS', '').strip()
        if workspace:
            return Path(workspace) / 'maps'
        root = os.environ.get('VICA_ROOT', '').strip()
        if root:
            return Path(root) / 'vica_ros2_ws' / 'maps'
        return None

    def _write_map_meta(self, map_id: str, display_name: str) -> None:
        """Leave maps/<id>.meta.json. 실패해도 저장은 성공이다 — id 는 그대로 쓴다."""
        if not display_name or display_name == map_id:
            return
        maps_dir = self._resolve_maps_dir()
        if maps_dir is None:
            self.get_logger().warn('maps 디렉터리를 몰라 표시 이름을 남기지 못했습니다.')
            return
        path = maps_dir / f'{map_id}.meta.json'
        document = map_meta_document(
            map_id, display_name, datetime.now().isoformat(timespec='seconds')
        )
        try:
            path.write_text(
                json.dumps(document, ensure_ascii=False, indent=2) + '\n', encoding='utf-8'
            )
        except OSError as error:
            self.get_logger().warn(f'{path} 를 쓰지 못했습니다: {error}')

    def _run_save(
        self, script: str, map_id: str, display_name: str, align: bool = False
    ) -> None:
        """Run vica_map_save.sh and report the result. 콜백 밖에서 돈다."""
        align_result = {}
        try:
            result = subprocess.run(  # noqa: S603
                save_script_command(script, map_id, align),
                capture_output=True,
                text=True,
                timeout=300,
            )
            ok = result.returncode == 0
            tail = (result.stdout or result.stderr or '').strip().splitlines()
            detail = tail[-1] if tail else ''
            align_result = parse_align_result(result.stdout or '')
        except subprocess.TimeoutExpired:
            ok, detail = False, '저장이 300초 안에 끝나지 않았습니다.'
        except OSError as error:
            ok, detail = False, f'저장 스크립트를 실행하지 못했습니다: {error}'

        if ok:
            self._write_map_meta(map_id, display_name)

        label = save_label(map_id, display_name)
        with self._lock:
            self.save_align = align_result if ok else {}
            if ok:
                self._set_state(MappingState.MAPPING, f'{label} 저장 완료. {detail}')
            else:
                self._set_state(
                    MappingState.MAPPING, f'{label} 저장 실패. {detail}'
                )
        if ok:
            self._request_preview_clear()
        self.get_logger().info(
            f'저장 {"성공" if ok else "실패"}: {map_id} {detail} '
            f'정렬={align_result.get("result", "-")}'
        )

    def _request_preview_clear(self) -> None:
        if not self._preview_clear.service_is_ready():
            return
        self._preview_clear.call_async(Trigger.Request())

    def _begin_stop(self) -> None:
        """정리를 시작한다 — SIGINT 를 바로 보내고 사다리를 세운다."""
        if self._process is None or self._stop is not None:
            return
        grace = float(self.get_parameter('shutdown_grace_sec').value)
        self._stop = StopEscalation(grace, time.monotonic())
        self._send_group_signal(self._stop.first_signal)

    def _advance_stop(self) -> None:
        """사다리를 한 칸씩 밟는다. _lock 을 잡고 부른다."""
        if self._process is None:
            self._stop = None
            return
        if self._process.poll() is not None:
            self._forget_process()
            self._stop = None
            if self.state is MappingState.STOPPING:
                self._set_state(MappingState.IDLE, '매핑을 종료했습니다.')
            else:
                self.get_logger().info('매핑 스택 정리 완료 (상태 유지).')
            return
        sig = self._stop.escalate_signal(time.monotonic())
        if sig is not None:
            self.get_logger().warn(
                f'유예 안에 끝나지 않았습니다. {sig.name} 로 올립니다.'
            )
            self._send_group_signal(sig)

    def _send_group_signal(self, sig) -> None:
        """자식 프로세스 그룹 전체에 신호를 보낸다. 실패해도 예외를 내지 않는다."""
        try:
            os.killpg(self._pgid, sig)
        except (ProcessLookupError, PermissionError):
            pass

    def _terminate_process(self) -> None:
        """Blocking 종료 — destroy_node 전용."""
        if self._process is None:
            return
        grace = float(self.get_parameter('shutdown_grace_sec').value)
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            if self._process.poll() is not None:
                break
            try:
                os.killpg(self._pgid, sig)
            except (ProcessLookupError, PermissionError):
                break
            try:
                self._process.wait(timeout=grace)
            except subprocess.TimeoutExpired:
                self.get_logger().warn(
                    f'{sig.name} 로 {grace:.0f}초 안에 끝나지 않았습니다. 더 강하게 보냅니다.'
                )
        self._forget_process()

    def _forget_process(self) -> None:
        self._process = None
        self._pgid = None

    def destroy_node(self) -> bool:
        """Never leave the mapping stack behind when this node goes away."""
        with self._lock:
            if self._process is not None:
                self.get_logger().warn('노드가 내려갑니다. 매핑 스택을 함께 정리합니다.')
                self._terminate_process()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the supervisor until shutdown."""
    rclpy.init(args=args)
    node = MappingSupervisorNode()
    executor = MultiThreadedExecutor(num_threads=2)
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
