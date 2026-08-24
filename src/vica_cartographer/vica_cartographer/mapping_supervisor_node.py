#!/usr/bin/env python3
"""Own the mapping stack so the app can start, stop and save it.

**왜 앱이 직접 띄우면 안 되는가.** 앱이 `ros2 launch` 를 직접 실행하면 앱이 꺼졌을 때
자식 프로세스가 고아로 남는다. 리모컨으로 에어컨을 켜 놓고 리모컨을 잃어버린 것과
같다. 그래서 젯슨에 상주하는 이 노드가 자식을 소유하고, 앱은 서비스만 부른다.

**무엇을 띄우고 무엇을 안 띄우는가.** `scripts/vica_terminator_layout.py` 의 vica_map
프로파일은 칸이 13개고 순서가 있다. 그중 앱이 대신할 수 있는 것만 launch 로 묶었다.

    앱이 띄운다   motor -> slam -> preview      (vica_mapping_bringup.launch.py)
    사람이 한다   전원·CAN, safety, d455(Docker), imu 자이로 보정 20초 정지

motor 를 뺄 수 없는 이유는 프로파일 설명에 있다 — "엔코더 피드백을 요청하는 쪽이
motor node 라서, 없으면 /wheel/odom 이 나오지 않는다". 다만 그 칸은 HOLD 라 사람이
먼저 눌렀을 수 있어서, 시작할 때 그래프를 보고 start_motor 인자를 정한다.

safety 는 아예 띄우지 않는다. 터미네이터의 safety 칸이 AUTO 라 창을 띄우는 순간
자동 실행되므로, 여기서 또 띄우면 항상 두 벌이 된다.

**저장은 콜백에서 기다리지 않는다.** vica_map_save.sh 는 map_saver 제한시간이
기본 120초라 그만큼 걸릴 수 있다. 콜백에서 기다리면 같은 콜백 그룹의 다른 요청이
전부 그 뒤로 줄을 서고, rosbridge 까지 막힌다 — mission_manager 의 _cancel_nav 가
정확히 그렇게 멈췄던 적이 있다(2026-08-21 수정). 그래서 서비스는 즉시 응답하고
실제 저장은 별도 스레드에서 돌며, 결과는 /vica/mapping_status 로 알린다.
"""

from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import threading

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
    is_motor_up,
    is_stack_up,
    MappingState,
    missing_prerequisites,
    normalise_map_name,
    running_stacks,
)

DEFAULT_LAUNCH = [
    'ros2', 'launch', 'vica_cartographer', 'vica_mapping_bringup.launch.py',
]

# 앱이 띄우지 않지만 없으면 회차가 무효가 되는 것들. 확인만 한다.
#   d455  Docker 컨테이너라 앱 범위 밖이다
#   imu   띄운 뒤 20초 완전 정지해야 'Gyro bias calibrated' 가 뜬다
#         (docs/cartographer_corridor_mapping.md 3절: 편향 -85 deg/hour 는
#          14.2분 주행에서 20도다 — 회차의 유효·무효를 가른다)
DEFAULT_PREREQUISITES = ['camera/camera', 'imu_base_link_adapter']


class MappingSupervisorNode(Node):
    """Start, watch, stop and save one mapping session."""

    def __init__(self) -> None:
        """Declare parameters and wire services."""
        super().__init__('mapping_supervisor_node')

        self.declare_parameter('launch_command', DEFAULT_LAUNCH)
        self.declare_parameter('save_script', '')
        self.declare_parameter('prerequisite_nodes', DEFAULT_PREREQUISITES)
        self.declare_parameter('status_period_sec', 1.0)
        # launch 를 띄우고 cartographer 가 올라오기를 기다리는 시한.
        self.declare_parameter('startup_timeout_sec', 40.0)
        # 자식에게 SIGINT 를 준 뒤 기다리는 시간. 넘으면 SIGTERM, 또 넘으면 SIGKILL.
        self.declare_parameter('shutdown_grace_sec', 8.0)

        self.state = MappingState.IDLE
        self.detail = ''
        self.map_id = ''
        self.started_at = None
        self._process = None
        self._pgid = None
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

    # -- 상태 -----------------------------------------------------------

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
                'started_at': self.started_at or '',
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
                self._terminate_process()
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

    # -- 서비스 ---------------------------------------------------------

    def handle_start(self, _request, response):
        """Start the mapping stack after checking nothing conflicts."""
        with self._lock:
            reason = blocking_reason(self.state, self._node_names())
            if reason:
                response.success = False
                response.message = reason
                self.get_logger().warn(f'매핑 시작 거부: {reason}')
                return response

            command = list(self.get_parameter('launch_command').value)
            # motor 가 이미 떠 있으면 launch 가 또 띄우지 않게 한다. 터미네이터의
            # motor 칸은 HOLD 라 사람이 먼저 눌렀을 수 있다. (safety 는 그 칸이
            # AUTO 라 항상 떠 있으므로 launch 에서 아예 뺐다.)
            motor_already_up = is_motor_up(self._node_names())
            command.append(
                f'start_motor:={"false" if motor_already_up else "true"}'
            )
            try:
                # start_new_session=True 로 자식에게 새 프로세스 그룹을 준다.
                # 그래야 launch 가 띄운 손자 노드들까지 한 번에 정리할 수 있다.
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
            self.started_at = datetime.now().isoformat(timespec='seconds')
            self._set_state(MappingState.STARTING, '스택을 띄우는 중입니다.')

        # 지난 회차의 미리보기가 이번 회차인 척하지 않게 지운다.
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
            self._set_state(MappingState.STOPPING, '정리하는 중입니다.')
            self._terminate_process()
            self._set_state(MappingState.IDLE, '')
        response.success = True
        response.message = '매핑을 종료했습니다.'
        return response

    def handle_save(self, request, response):
        """Accept a save request and run the script off the callback thread."""
        with self._lock:
            if self.state is not MappingState.MAPPING:
                response.accepted = False
                response.message = f'지금은 저장할 수 없습니다(상태: {self.state.value}).'
                return response

            map_id, error = normalise_map_name(
                request.name, datetime.now().strftime('%m%d')
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

            self.map_id = map_id
            self._set_state(MappingState.SAVING, f'{map_id} 저장 중입니다.')

        threading.Thread(
            target=self._run_save,
            args=(str(script), map_id),
            name='vica_map_save',
            daemon=True,
        ).start()

        response.accepted = True
        response.message = f'{map_id} 저장을 시작했습니다.'
        response.map_id = map_id
        return response

    # -- 저장 -----------------------------------------------------------

    def _resolve_save_script(self):
        explicit = str(self.get_parameter('save_script').value).strip()
        if explicit:
            path = Path(explicit)
            return path if path.exists() else None
        # scripts/bringup/vica_env.sh 가 세우는 값이다. 개인 경로를 코드에 박지 않는다.
        root = os.environ.get('VICA_ROOT', '').strip()
        if not root:
            return None
        path = Path(root) / 'scripts' / 'vica_map_save.sh'
        return path if path.exists() else None

    def _run_save(self, script: str, map_id: str) -> None:
        """Run vica_map_save.sh and report the result. 콜백 밖에서 돈다."""
        try:
            result = subprocess.run(  # noqa: S603
                ['bash', script, map_id],
                capture_output=True,
                text=True,
                timeout=300,
            )
            ok = result.returncode == 0
            tail = (result.stdout or result.stderr or '').strip().splitlines()
            detail = tail[-1] if tail else ''
        except subprocess.TimeoutExpired:
            ok, detail = False, '저장이 300초 안에 끝나지 않았습니다.'
        except OSError as error:
            ok, detail = False, f'저장 스크립트를 실행하지 못했습니다: {error}'

        with self._lock:
            if ok:
                self._set_state(MappingState.MAPPING, f'{map_id} 저장 완료. {detail}')
            else:
                # 실패하면 상태를 MAPPING 으로 되돌린다. 지도는 아직 메모리에 있고
                # 다시 저장할 수 있다.
                self._set_state(
                    MappingState.MAPPING, f'{map_id} 저장 실패. {detail}'
                )
        if ok:
            # **저장이 확인됐을 때만** 미리보기를 지운다. 실패했을 때 지우면
            # 2026-08-12 처럼(아홉 회차가 저장 실패로 사라짐) 유일한 그림까지 잃는다.
            self._request_preview_clear()
        self.get_logger().info(f'저장 {"성공" if ok else "실패"}: {map_id} {detail}')

    def _request_preview_clear(self) -> None:
        if not self._preview_clear.service_is_ready():
            return
        # 응답을 기다리지 않는다. 미리보기 정리는 실패해도 매핑에 지장이 없다.
        self._preview_clear.call_async(Trigger.Request())

    # -- 프로세스 -------------------------------------------------------

    def _terminate_process(self) -> None:
        """Send SIGINT to the whole group, then escalate. _lock 을 잡고 부른다."""
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
