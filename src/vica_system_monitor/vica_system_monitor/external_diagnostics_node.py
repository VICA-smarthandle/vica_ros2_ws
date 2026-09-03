"""Publish /diagnostics on behalf of things we cannot modify.

rplidar_ros, nvblox(Docker), D455는 외부 패키지라 fork하지 않으면 진단을 낼 수 없다. 이
노드가 대신 관측해 표준 `/diagnostics`로 올린다. 그러면 외부 대상도 우리 노드와 완전히
같은 경로(aggregator -> robot_health_monitor_node -> 앱)로 흐른다.

프로브 두 종류를 `config/probes.yaml`로 설정한다. 대상을 추가할 때 코드를 고치지 않는다.

    topic_rate   : 토픽이 기대 주기로 오는가 (diagnostic_updater의 FrequencyStatus)
    process_cpu  : /proc/<pid>/stat 기반 노드별 CPU %

설계상 중요한 것:

1. **카메라는 image가 아니라 camera_info를 구독한다.** 같은 주기로 나오면서 수백 바이트다.
   30 Hz depth 원본을 복사하면 감시 노드가 대역폭 소비자가 된다.
2. **QoS를 설정으로 뺀다.** 잘못 고르면 구독이 매칭되지 않아 메시지를 하나도 받지 못하고,
   감시 도구가 스스로 "센서가 죽었다"고 영구 오탐한다. 그래서 "구독자는 붙었는데 메시지
   0건" 상태를 진단 문구에 구분해 남긴다.
3. **optional 프로브는 타입 import 실패 시 그것만 건너뛴다.** nvblox_msgs symlink가 빠져도
   어댑터 전체가 기동 실패하지 않는다.
4. **정지 경로에 들어가지 않는다.** 진단은 보고 전용이다
   (vica_system_health_monitoring_draft.md 3.1절).
"""

import importlib
import os
from pathlib import Path

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import (
    DiagnosticStatusWrapper,
    FrequencyStatusParam,
    HeaderlessTopicDiagnostic,
    Updater,
)
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from .probe_config import (
    classify_zero_message,
    parse_link_probe,
    parse_process_probe,
    parse_topic_probe,
    QOS_SENSOR_DATA,
    ZERO_QOS_SUSPECTED,
)
from .process_cpu import match_cmdline, parse_stat_jiffies, select_probe_pid


PROC_ROOT = Path('/proc')

# probes.yaml에서 프로브별로 읽을 필드 이름.
_TOPIC_FIELDS = (
    ('component', ''),
    ('topic', ''),
    ('msg_type', ''),
    ('qos', 'default'),
    ('min_hz', 0.0),
    ('max_hz', 0.0),
    ('fault_code', ''),
    ('optional', False),
)
_LINK_FIELDS = (
    ('component', ''),
    ('topic', ''),
    ('min_publishers', 1.0),
    ('fault_code', ''),
)
_PROCESS_FIELDS = (
    ('component', ''),
    ('cmdline_pattern', ''),
    ('required', False),
    ('warn_percent', 0.0),
)


class ExternalDiagnosticsNode(Node):
    """Adapter that reports on external topics and processes."""

    def __init__(self) -> None:
        """Declare parameters, build probes and start the updater."""
        super().__init__('external_diagnostics_node')

        self.declare_parameter('diagnostic_period_sec', 1.0)
        self.declare_parameter('process_scan_period_sec', 5.0)
        self.declare_parameter('topic_probe_names', [''])
        self.declare_parameter('link_probe_names', [''])
        self.declare_parameter('process_probe_names', [''])

        # 노드가 죽지 않게 하는 것이 감시 도구의 첫 요건이다. 설정 오류는 로그로 남기고
        # 그 프로브만 건너뛴다.
        self.config_problems: list = []

        # 진단 시각은 Safety·motor와 같은 단일 STEADY_TIME clock을 쓴다.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self.updater = Updater(self)
        self.updater.setHardwareID('vica_external')
        self.updater.add('external adapter', self.diagnose_self)

        self.topic_diagnostics: dict = {}
        self.topic_counts: dict = {}
        self.skipped_probes: list = []

        self._build_topic_probes()
        self._build_link_probes()
        self._build_process_probes()

        period = float(self.get_parameter('process_scan_period_sec').value)
        if self.process_specs:
            self.create_timer(period, self.sample_processes)

        self.get_logger().info(
            f'external_diagnostics_node ready: '
            f'{len(self.topic_diagnostics)} topic probes, '
            f'{len(self.process_specs)} process probes, '
            f'{len(self.skipped_probes)} skipped'
        )
        for problem in self.config_problems:
            self.get_logger().error(f'probe 설정 오류: {problem}')

    # ------------------------------------------------------------------
    # 프로브 구성
    # ------------------------------------------------------------------
    def _read_probe_values(self, name: str, fields) -> dict:
        """Declare and read one probe's dotted parameters.

        ROS 2 파라미터는 중첩 리스트를 담을 수 없으므로 이름 목록 + dotted namespace를
        쓴다. diagnostic_aggregator의 analyzers 설정과 같은 방식이다.
        """
        values = {}
        for field, default in fields:
            key = f'{name}.{field}'
            if not self.has_parameter(key):
                self.declare_parameter(key, default)
            values[field] = self.get_parameter(key).value
        return values

    def _build_topic_probes(self) -> None:
        """Create a FrequencyStatus diagnostic and subscription per topic probe."""
        names = [n for n in self.get_parameter('topic_probe_names').value if n]

        for name in names:
            values = self._read_probe_values(name, _TOPIC_FIELDS)
            spec, problems = parse_topic_probe(name, values)
            if problems:
                self.config_problems.extend(problems)
                continue

            msg_class = self._import_msg_type(spec.msg_type)
            if msg_class is None:
                # optional이면 이 프로브만 건너뛴다. 아니면 설정 오류로 남긴다.
                if spec.optional:
                    self.skipped_probes.append(
                        f'{spec.name}({spec.msg_type} 미설치)'
                    )
                else:
                    self.config_problems.append(
                        f"'{spec.name}': {spec.msg_type}를 import할 수 없습니다."
                    )
                continue

            param = FrequencyStatusParam(
                {'min': spec.min_hz, 'max': spec.max_hz},
                tolerance=0.2,
                window_size=5,
            )
            label = f'{spec.component}: {spec.topic} frequency'
            diagnostic = HeaderlessTopicDiagnostic(label, self.updater, param)

            self.topic_diagnostics[spec.name] = (spec, diagnostic)
            self.topic_counts[spec.name] = 0

            qos = (
                qos_profile_sensor_data
                if spec.qos == QOS_SENSOR_DATA
                else qos_profile_system_default
            )
            self.create_subscription(
                msg_class,
                spec.topic,
                self._make_topic_callback(spec.name, diagnostic),
                qos,
            )

    def _build_link_probes(self) -> None:
        """Register publisher-presence probes. 구독하지 않고 그래프만 본다."""
        self.link_specs = []
        names = [n for n in self.get_parameter('link_probe_names').value if n]

        for name in names:
            values = self._read_probe_values(name, _LINK_FIELDS)
            spec, problems = parse_link_probe(name, values)
            if problems:
                self.config_problems.extend(problems)
                continue
            self.link_specs.append(spec)
            self.updater.add(
                f'{spec.component}: {spec.name} link',
                self._make_link_task(spec),
            )

    def _make_link_task(self, spec):
        """Build a diagnostic task reporting whether the topic has a publisher.

        메시지를 구독하지 않는다. count_publishers 는 DDS 그래프 조회라 대역폭을
        쓰지 않으며, **로봇이 서 있어도 판정된다** — 그것이 이 프로브의 존재
        이유다(LinkProbeSpec docstring).
        """

        def task(stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
            count = self.count_publishers(spec.topic)
            stat.add('topic', spec.topic)
            stat.add('publishers', str(count))
            stat.add('required', str(spec.min_publishers))
            if count >= spec.min_publishers:
                stat.summary(DiagnosticStatus.OK, f'발행자 {count}개')
            else:
                stat.summary(
                    DiagnosticStatus.ERROR,
                    f'발행자가 없습니다 (필요 {spec.min_publishers}개)',
                )
            return stat

        return task

    def _build_process_probes(self) -> None:
        """Parse process probe specs. Sampling happens on a timer."""
        self.process_specs = []
        names = [n for n in self.get_parameter('process_probe_names').value if n]

        for name in names:
            values = self._read_probe_values(name, _PROCESS_FIELDS)
            spec, problems = parse_process_probe(name, values)
            if problems:
                self.config_problems.extend(problems)
                continue
            self.process_specs.append(spec)
            self.updater.add(
                f'{spec.component}: {spec.name} cpu',
                self._make_process_task(spec),
            )

        self.cpu_percent: dict = {}
        self.cpu_pid: dict = {}
        self.cpu_previous: dict = {}
        # 패턴에 걸린 노드 후보 전체. 2개 이상이면 어느 것을 쟀는지 알 수 없으므로
        # 진단에 그대로 남긴다(2026-08-01 실기 오보고).
        self.cpu_candidates: dict = {}
        self.clock_ticks = self._clock_ticks()

    def _import_msg_type(self, msg_type: str):
        """Import a message class from its 'pkg/msg/Type' string, or None.

        optional 프로브(nvblox 등)가 없어도 어댑터가 기동해야 하므로 예외를 흡수한다.
        """
        parts = msg_type.split('/')
        if len(parts) != 3:
            return None
        package, _, type_name = parts
        try:
            module = importlib.import_module(f'{package}.msg')
            return getattr(module, type_name)
        except (ImportError, AttributeError):
            return None

    def _clock_ticks(self) -> int:
        """Return SC_CLK_TCK, or 0 when unavailable."""
        try:
            return int(os.sysconf('SC_CLK_TCK'))
        except (ValueError, OSError, AttributeError):
            return 0

    # ------------------------------------------------------------------
    # topic_rate 프로브
    # ------------------------------------------------------------------
    def _make_topic_callback(self, name: str, diagnostic):
        """Build a subscription callback that ticks the frequency diagnostic."""

        def callback(_msg) -> None:
            self.topic_counts[name] = self.topic_counts.get(name, 0) + 1
            diagnostic.tick()

        return callback

    # ------------------------------------------------------------------
    # process_cpu 프로브
    # ------------------------------------------------------------------
    def sample_processes(self) -> None:
        """Refresh CPU percentages for every configured process probe."""
        now_ns = self.steady_clock.now().nanoseconds

        for spec in self.process_specs:
            pid, kept = self._find_pid(spec.cmdline_pattern)
            self.cpu_candidates[spec.name] = kept
            if pid is None:
                self.cpu_percent[spec.name] = None
                self.cpu_pid[spec.name] = None
                self.cpu_previous.pop(spec.name, None)
                continue

            jiffies = self._read_jiffies(pid)
            if jiffies is None:
                self.cpu_percent[spec.name] = None
                continue

            self.cpu_pid[spec.name] = pid
            self.cpu_percent[spec.name] = self._advance_cpu(
                spec.name, jiffies, now_ns
            )

    def _advance_cpu(self, name: str, jiffies: int, now_ns: int):
        """Compute CPU percent from the previous sample of this process.

        CpuTracker와 같은 계산이지만 노드가 pid 교체를 함께 다루므로 상태를 직접 들고 있다.
        이상 입력에서는 값을 내지 않고 기준만 다시 잡는다.
        """
        previous = self.cpu_previous.get(name)
        self.cpu_previous[name] = (jiffies, now_ns)

        if previous is None or self.clock_ticks <= 0:
            return None

        prev_jiffies, prev_ns = previous
        elapsed_ns = now_ns - prev_ns
        if elapsed_ns <= 0:
            return None
        delta = jiffies - prev_jiffies
        if delta < 0:
            return None

        cpu_seconds = delta / self.clock_ticks
        return (cpu_seconds / (elapsed_ns / 1_000_000_000)) * 100.0

    def _find_pid(self, pattern: str):
        """Return (chosen pid or None, every node-looking candidate pid).

        패턴에 걸린 것을 모두 모은 뒤 셸과 `ros2 run|launch` 런처를 걸러낸다. 첫 일치를
        그냥 쓰면 노드 대신 그것을 띄운 셸을 재게 된다(2026-08-01 실기, CPU 0.0 % 오보고).
        """
        try:
            entries = list(PROC_ROOT.iterdir())
        except OSError:
            return None, []

        candidates = []
        for entry in entries:
            if not entry.name.isdigit():
                continue
            try:
                cmdline = (entry / 'cmdline').read_text(encoding='utf-8', errors='ignore')
            except OSError:
                continue
            if match_cmdline(cmdline, pattern):
                candidates.append((entry.name, cmdline))

        return select_probe_pid(candidates)

    def _read_jiffies(self, pid: str):
        """Return utime+stime for a pid, or None when it disappeared."""
        try:
            text = (PROC_ROOT / pid / 'stat').read_text(encoding='utf-8')
        except OSError:
            return None
        return parse_stat_jiffies(text)

    def _make_process_task(self, spec):
        """Build a diagnostic task reporting one process's CPU usage."""

        def task(stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
            percent = self.cpu_percent.get(spec.name)
            pid = self.cpu_pid.get(spec.name)

            if pid is None:
                if spec.required:
                    # 이 프로세스는 있어야 한다. 부재 자체가 결함이다.
                    # 주기 토픽이 없어 다른 관측 수단이 없는 부품에만 쓴다
                    # (2026-09-02, 음성).
                    stat.summary(
                        DiagnosticStatus.ERROR,
                        '노드가 실행되지 않고 있습니다',
                    )
                    stat.add('pattern', spec.cmdline_pattern)
                    return stat
                # 프로세스가 없다. Docker PID namespace 때문일 수도 있다.
                # 결함이 아니라 미구성으로 보고한다.
                stat.summary(
                    DiagnosticStatus.OK,
                    '프로세스를 찾지 못했습니다 (미구성 또는 미실행)',
                )
                stat.add('pattern', spec.cmdline_pattern)
                stat.add('observable', 'false')
                return stat

            if percent is None:
                stat.summary(DiagnosticStatus.OK, '첫 표본 수집 중')
                stat.add('pid', str(pid))
                return stat

            level = (
                DiagnosticStatus.WARN
                if percent >= spec.warn_percent
                else DiagnosticStatus.OK
            )
            summary = f'CPU {percent:.1f}%'

            # 후보가 둘 이상이면 어느 프로세스를 잰 값인지 단정할 수 없다. 숨기지 않는다.
            candidates = self.cpu_candidates.get(spec.name) or []
            if len(candidates) > 1:
                level = max(level, DiagnosticStatus.WARN)
                summary = (
                    f'{summary} — 같은 패턴에 {len(candidates)}개가 걸립니다. '
                    f'측정 대상이 모호하므로 pattern을 좁혀 주세요'
                )

            stat.summary(level, summary)
            stat.add('pid', str(pid))
            stat.add('cpu_percent', f'{percent:.1f}')
            stat.add('warn_percent', f'{spec.warn_percent:.1f}')
            stat.add('pattern', spec.cmdline_pattern)
            stat.add('candidate_pids', ','.join(candidates))
            return stat

        return task

    # ------------------------------------------------------------------
    # 자기 진단
    # ------------------------------------------------------------------
    def diagnose_self(
        self,
        stat: DiagnosticStatusWrapper,
    ) -> DiagnosticStatusWrapper:
        """Report adapter configuration state and zero-message probes.

        **QoS 비호환과 진짜 단절을 사람이 구분할 수 있게 한다.** 구독은 만들어졌는데
        메시지가 0건이면 QoS가 맞지 않을 가능성이 있다. FrequencyStatus만 보면 둘이
        똑같이 "주기 미달"로 보인다.
        """
        if self.config_problems:
            stat.summary(
                DiagnosticStatus.ERROR,
                f'설정 오류 {len(self.config_problems)}건. 해당 프로브가 빠졌습니다',
            )
        else:
            stat.summary(DiagnosticStatus.OK, '어댑터 정상')

        stat.add('topic_probes', str(len(self.topic_diagnostics)))
        stat.add('process_probes', str(len(self.process_specs)))

        if self.skipped_probes:
            stat.add('skipped', ', '.join(self.skipped_probes))

        # 미수신을 발행자 수로 분류한다. FrequencyStatus만 보면 "노드가 안 떴다"와
        # "발행은 되는데 우리가 못 받는다(QoS 비호환)"가 구별되지 않는다.
        no_publisher = []
        qos_suspected = []
        for name, (spec, _) in self.topic_diagnostics.items():
            if self.topic_counts.get(name, 0) > 0:
                continue
            count = self.count_publishers(spec.topic)
            label = f'{name}({spec.topic}, qos={spec.qos}, pub={count})'
            if classify_zero_message(count) == ZERO_QOS_SUSPECTED:
                qos_suspected.append(label)
            else:
                no_publisher.append(label)

        if no_publisher:
            stat.add('zero_message_no_publisher', ', '.join(no_publisher))
        if qos_suspected:
            # 감시 도구 자체의 결함일 가능성이 있으므로 등급을 올린다.
            stat.summary(
                DiagnosticStatus.WARN,
                f'발행자가 있는데 수신 0건인 프로브 {len(qos_suspected)}개. '
                'QoS 비호환 의심',
            )
            stat.add('zero_message_qos_suspected', ', '.join(qos_suspected))
            stat.add(
                'zero_message_hint',
                'ros2 topic info -v <topic>으로 발행 QoS를 확인하고 '
                'probes.yaml의 qos를 맞출 것',
            )

        for problem in self.config_problems:
            stat.add('problem', problem)

        return stat


def main() -> None:
    """Spin the adapter node."""
    rclpy.init()
    node = ExternalDiagnosticsNode()
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
