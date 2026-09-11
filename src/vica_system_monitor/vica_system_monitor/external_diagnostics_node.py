"""Publish /diagnostics on behalf of things we cannot modify."""

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
    parse_process_probe,
    parse_topic_probe,
    QOS_SENSOR_DATA,
    ZERO_QOS_SUSPECTED,
)
from .process_cpu import match_cmdline, parse_stat_jiffies, select_probe_pid


PROC_ROOT = Path('/proc')

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
_PROCESS_FIELDS = (
    ('component', ''),
    ('cmdline_pattern', ''),
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
        self.declare_parameter('process_probe_names', [''])

        self.config_problems: list = []

        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self.updater = Updater(self)
        self.updater.setHardwareID('vica_external')
        self.updater.add('external adapter', self.diagnose_self)

        self.topic_diagnostics: dict = {}
        self.topic_counts: dict = {}
        self.skipped_probes: list = []

        self._build_topic_probes()
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

    def _read_probe_values(self, name: str, fields) -> dict:
        """Declare and read one probe's dotted parameters."""
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
        self.cpu_candidates: dict = {}
        self.clock_ticks = self._clock_ticks()

    def _import_msg_type(self, msg_type: str):
        """Import a message class from its 'pkg/msg/Type' string, or None."""
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

    def _make_topic_callback(self, name: str, diagnostic):
        """Build a subscription callback that ticks the frequency diagnostic."""

        def callback(_msg) -> None:
            self.topic_counts[name] = self.topic_counts.get(name, 0) + 1
            diagnostic.tick()

        return callback

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
        """Compute CPU percent from the previous sample of this process."""
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
        """Return (chosen pid or None, every node-looking candidate pid)."""
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

    def diagnose_self(
        self,
        stat: DiagnosticStatusWrapper,
    ) -> DiagnosticStatusWrapper:
        """Report adapter configuration state and zero-message probes."""
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
