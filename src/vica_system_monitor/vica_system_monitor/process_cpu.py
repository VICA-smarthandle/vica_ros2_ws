"""Per-process CPU sampling from /proc, as pure computation."""

from typing import Dict, List, NamedTuple, Optional, Sequence, Tuple


NS_PER_SEC = 1_000_000_000

_SHELL_NAMES = frozenset({'bash', 'sh', 'dash', 'zsh', 'ksh', 'fish'})

_ROS2_SUBCOMMANDS = frozenset({'run', 'launch'})

_UTIME_INDEX_AFTER_COMM = 11
_STIME_INDEX_AFTER_COMM = 12


class CpuSample(NamedTuple):
    """One resolved CPU measurement."""

    name: str
    percent: float


class _Previous(NamedTuple):
    """Last accepted sample for one tracked process."""

    jiffies: int
    now_ns: int


def parse_stat_jiffies(stat_text: str) -> Optional[int]:
    """Return utime+stime from a /proc/<pid>/stat line, or None when unparsable."""
    if not stat_text:
        return None

    close = stat_text.rfind(')')
    if close < 0:
        return None

    tail = stat_text[close + 1:].split()
    if len(tail) <= _STIME_INDEX_AFTER_COMM:
        return None

    try:
        utime = int(tail[_UTIME_INDEX_AFTER_COMM])
        stime = int(tail[_STIME_INDEX_AFTER_COMM])
    except ValueError:
        return None

    return utime + stime


def match_cmdline(cmdline_text: str, pattern: str) -> bool:
    """Return True when the NUL-separated cmdline contains the pattern."""
    if not cmdline_text or not pattern:
        return False
    joined = cmdline_text.replace('\x00', ' ').strip()
    if not joined:
        return False
    return pattern in joined


def _argv(cmdline_text: str) -> List[str]:
    """Split a NUL-separated /proc/<pid>/cmdline into arguments."""
    return [part for part in cmdline_text.split('\x00') if part]


def is_node_process(cmdline_text: str) -> bool:
    """Return False for shells and `ros2 run|launch` launchers."""
    argv = _argv(cmdline_text)
    if not argv:
        return False

    if argv[0].rsplit('/', 1)[-1] in _SHELL_NAMES:
        return False

    for index, arg in enumerate(argv):
        name = arg.rsplit('/', 1)[-1]
        if name != 'ros2':
            continue
        following = argv[index + 1] if index + 1 < len(argv) else ''
        if following in _ROS2_SUBCOMMANDS:
            return False

    return True


def select_probe_pid(
    candidates: Sequence[Tuple[str, str]],
) -> Tuple[Optional[str], List[str]]:
    """Pick the node process among pattern matches."""
    kept = [pid for pid, cmdline in candidates if is_node_process(cmdline)]
    if not kept:
        return None, []

    chosen = min(kept, key=lambda pid: (int(pid) if pid.isdigit() else 0, pid))
    return chosen, kept


class CpuTracker:
    """Turn successive jiffie counters into CPU percentages."""

    def __init__(self, clock_ticks_per_sec: int) -> None:
        """Store the platform clock tick rate (os.sysconf('SC_CLK_TCK'))."""
        self._ticks = clock_ticks_per_sec
        self._previous: Dict[str, _Previous] = {}

    def update(
        self,
        name: str,
        jiffies: int,
        now_ns: int,
    ) -> Optional[float]:
        """Record a sample and return CPU percent since the previous one."""
        previous = self._previous.get(name)
        self._previous[name] = _Previous(jiffies=jiffies, now_ns=now_ns)

        if previous is None:
            return None
        if self._ticks <= 0:
            return None

        elapsed_ns = now_ns - previous.now_ns
        if elapsed_ns <= 0:
            return None

        delta_jiffies = jiffies - previous.jiffies
        if delta_jiffies < 0:
            return None

        cpu_seconds = delta_jiffies / self._ticks
        elapsed_seconds = elapsed_ns / NS_PER_SEC
        return (cpu_seconds / elapsed_seconds) * 100.0

    def forget(self, name: str) -> None:
        """Drop stored state so a reappearing process starts clean."""
        self._previous.pop(name, None)

    def tracked(self) -> Dict[str, _Previous]:
        """Return the current baseline map. 진단 표시용이다."""
        return dict(self._previous)
