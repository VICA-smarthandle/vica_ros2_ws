"""Per-process CPU sampling from /proc, as pure computation.

이 모듈은 파일을 읽지 않는다. 호출자가 `/proc/<pid>/stat` 내용을 문자열로 넘기고, 여기서는
파싱과 델타 계산만 한다. 그래야 개발용 컴퓨터에서 그대로 검증할 수 있다.

목적은 워크스페이스 리뷰 7.1·7.4절의 요구다. `imu_base_link_adapter`의 CPU 38.6 %와 EKF
30 Hz 미달이 유일한 before이며 재현할 수 없으므로, 최적화 전에 상시 계측 수단을 둔다.
임계값 40 %를 하드코딩하면 개선 후에도 통과해 회귀를 잡지 못하므로 값은 YAML로 뺀다
(리뷰 7.5절).

한계:
- Docker 프로세스(nvblox, D455)는 PID namespace가 분리되면 host `/proc`에서 보이지 않는다.
  Jetson에서 확인하고 안 보이면 그 프로브를 미구성으로 둔다.
- CPU 사용률로 주행을 막지 않는다. 등급은 WARN 상한으로 둔다.
"""

from typing import Dict, NamedTuple, Optional


NS_PER_SEC = 1_000_000_000

# /proc/<pid>/stat에서 utime·stime의 위치. comm 필드 뒤를 0-based로 센다.
# 전체 필드는 1-based로 pid(1) comm(2) state(3) ... utime(14) stime(15)이다.
# 괄호 뒤부터 세면 state가 0번이므로 utime은 11번, stime은 12번이다.
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
    """Return utime+stime from a /proc/<pid>/stat line, or None when unparsable.

    comm 필드(2번째)는 괄호로 감싸이며 **공백을 포함할 수 있다.** 예를 들어
    ``42 (my node name) S ...``에서 단순 split을 쓰면 이후 모든 필드 위치가 밀린다.
    따라서 마지막 ``)`` 뒤부터 센다.

    예외를 던지지 않는다. 감시 노드가 파싱 실패로 죽으면 상태 표시 자체가 사라진다.
    """
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
    """Return True when the NUL-separated cmdline contains the pattern.

    커널 스레드는 cmdline이 비어 있으므로 매칭하지 않는다.
    """
    if not cmdline_text or not pattern:
        return False
    joined = cmdline_text.replace('\x00', ' ').strip()
    if not joined:
        return False
    return pattern in joined


class CpuTracker:
    """Turn successive jiffie counters into CPU percentages.

    첫 표본은 델타를 낼 수 없으므로 값이 없다(None). 이상 입력에서는 값을 내지 않고
    기준만 다시 잡는다 — 잘못된 수치를 보고하는 것보다 "아직 모른다"가 안전하다.
    """

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
        """Record a sample and return CPU percent since the previous one.

        멀티스레드 프로세스는 100 %를 넘을 수 있다. 잘라내지 않는다 — 코어 수 대비
        실제 부하를 보는 것이 목적이다.
        """
        previous = self._previous.get(name)
        self._previous[name] = _Previous(jiffies=jiffies, now_ns=now_ns)

        if previous is None:
            return None
        if self._ticks <= 0:
            return None

        elapsed_ns = now_ns - previous.now_ns
        if elapsed_ns <= 0:
            # 같은 시각 또는 시간 역전. 기준은 위에서 이미 갱신했다.
            return None

        delta_jiffies = jiffies - previous.jiffies
        if delta_jiffies < 0:
            # pid 재사용 등으로 카운터가 줄었다. 기준만 다시 잡는다.
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
