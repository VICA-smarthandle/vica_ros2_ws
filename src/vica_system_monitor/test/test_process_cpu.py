"""Unit tests for /proc based per-process CPU sampling.

파일시스템을 읽지 않는다. /proc 내용을 문자열로 주입해 계산만 검증한다.
이 프로브의 목적은 imu_base_link_adapter의 CPU 38.6 % baseline을 기록하고 최적화 이후
회귀를 잡는 것이다(워크스페이스 리뷰 7.1·7.4절).
"""

from vica_system_monitor.process_cpu import (
    CpuSample,
    CpuTracker,
    match_cmdline,
    parse_stat_jiffies,
)


def stat_line(utime: int, stime: int) -> str:
    """Build a minimal /proc/<pid>/stat line.

    utime은 14번째, stime은 15번째 필드다(1-based). 앞의 13개를 채워 둔다.
    """
    fields = ['1234', '(python3)', 'S'] + ['0'] * 10
    fields += [str(utime), str(stime)]
    fields += ['0'] * 10
    return ' '.join(fields)


# ---------------------------------------------------------------------------
# /proc/<pid>/stat 파싱
# ---------------------------------------------------------------------------


def test_parse_stat_reads_utime_and_stime():
    """14·15번째 필드를 더해 총 jiffies를 만든다."""
    assert parse_stat_jiffies(stat_line(utime=100, stime=50)) == 150


def test_parse_stat_handles_spaces_in_process_name():
    """프로세스 이름에 공백이 있어도 필드 위치가 밀리지 않는다.

    /proc stat의 comm 필드는 괄호로 감싸이며 공백을 포함할 수 있다. 괄호를 기준으로
    잘라야 한다. 단순 split은 여기서 깨진다.
    """
    line = '42 (my node name) S ' + ' '.join(['0'] * 10) + ' 70 30 ' + ' '.join(['0'] * 8)
    assert parse_stat_jiffies(line) == 100


def test_parse_stat_returns_none_for_garbage():
    """깨진 입력에 예외를 던지지 않는다. 감시 노드가 죽으면 안 된다."""
    assert parse_stat_jiffies('') is None
    assert parse_stat_jiffies('not a stat line') is None
    assert parse_stat_jiffies('1 (x) S 0 0') is None


# ---------------------------------------------------------------------------
# CPU % 계산
# ---------------------------------------------------------------------------


def test_first_sample_has_no_percentage():
    """첫 표본은 델타를 낼 수 없으므로 값이 없다."""
    tracker = CpuTracker(clock_ticks_per_sec=100)
    result = tracker.update('imu', jiffies=1000, now_ns=0)

    assert result is None


def test_second_sample_computes_percentage():
    """1초 동안 100 jiffies(=1초 CPU)를 쓰면 100 %다."""
    tracker = CpuTracker(clock_ticks_per_sec=100)
    tracker.update('imu', jiffies=1000, now_ns=0)
    result = tracker.update('imu', jiffies=1100, now_ns=1_000_000_000)

    assert result == 100.0


def test_partial_cpu_usage():
    """10초 동안 386 jiffies(=3.86초 CPU)면 38.6 %다.

    현재 imu_base_link_adapter의 baseline과 같은 수치다(리뷰 7.1절).
    """
    tracker = CpuTracker(clock_ticks_per_sec=100)
    tracker.update('imu', jiffies=0, now_ns=0)
    result = tracker.update('imu', jiffies=386, now_ns=10_000_000_000)

    assert round(result, 1) == 38.6


def test_multi_core_can_exceed_100_percent():
    """멀티스레드 프로세스는 100 %를 넘을 수 있다. 잘라내지 않는다."""
    tracker = CpuTracker(clock_ticks_per_sec=100)
    tracker.update('nav2', jiffies=0, now_ns=0)
    result = tracker.update('nav2', jiffies=250, now_ns=1_000_000_000)

    assert result == 250.0


def test_tracks_processes_independently():
    """프로세스별로 상태를 따로 유지한다."""
    tracker = CpuTracker(clock_ticks_per_sec=100)
    tracker.update('a', jiffies=0, now_ns=0)
    tracker.update('b', jiffies=0, now_ns=0)

    assert tracker.update('a', jiffies=50, now_ns=1_000_000_000) == 50.0
    assert tracker.update('b', jiffies=10, now_ns=1_000_000_000) == 10.0


# ---------------------------------------------------------------------------
# 이상 입력 방어
# ---------------------------------------------------------------------------


def test_zero_elapsed_time_returns_none():
    """같은 시각의 두 표본으로 나눗셈하지 않는다."""
    tracker = CpuTracker(clock_ticks_per_sec=100)
    tracker.update('imu', jiffies=0, now_ns=1000)
    assert tracker.update('imu', jiffies=50, now_ns=1000) is None


def test_time_reversal_resets_baseline():
    """시간이 역전되면 값을 내지 않고 기준을 다시 잡는다."""
    tracker = CpuTracker(clock_ticks_per_sec=100)
    tracker.update('imu', jiffies=0, now_ns=5_000_000_000)
    assert tracker.update('imu', jiffies=50, now_ns=1_000_000_000) is None
    # 기준이 갱신되었으므로 다음 표본은 정상 계산된다.
    assert tracker.update('imu', jiffies=150, now_ns=2_000_000_000) == 100.0


def test_jiffies_going_backwards_resets_baseline():
    """pid가 재사용되어 카운터가 줄면 기준을 다시 잡는다."""
    tracker = CpuTracker(clock_ticks_per_sec=100)
    tracker.update('imu', jiffies=1000, now_ns=0)
    assert tracker.update('imu', jiffies=10, now_ns=1_000_000_000) is None


def test_forget_removes_state():
    """프로세스가 사라지면 상태를 버려 다음 등장 시 오염되지 않게 한다."""
    tracker = CpuTracker(clock_ticks_per_sec=100)
    tracker.update('imu', jiffies=1000, now_ns=0)
    tracker.forget('imu')
    assert tracker.update('imu', jiffies=1100, now_ns=1_000_000_000) is None


def test_zero_clock_ticks_is_rejected():
    """Clock tick이 0이면 계산하지 않는다."""
    tracker = CpuTracker(clock_ticks_per_sec=0)
    tracker.update('imu', jiffies=0, now_ns=0)
    assert tracker.update('imu', jiffies=100, now_ns=1_000_000_000) is None


# ---------------------------------------------------------------------------
# cmdline 매칭
# ---------------------------------------------------------------------------


def test_match_cmdline_finds_pattern():
    """/proc/<pid>/cmdline은 NUL로 구분된다."""
    cmdline = 'python3\x00/opt/ros/imu_base_link_adapter.py\x00--ros-args\x00'
    assert match_cmdline(cmdline, 'imu_base_link_adapter')


def test_match_cmdline_rejects_other_process():
    """다른 프로세스를 잡지 않는다."""
    cmdline = 'python3\x00/opt/ros/vslam_covariance_adapter.py\x00'
    assert not match_cmdline(cmdline, 'imu_base_link_adapter')


def test_match_cmdline_handles_empty():
    """빈 cmdline(커널 스레드)은 매칭하지 않는다."""
    assert not match_cmdline('', 'anything')
    assert not match_cmdline('\x00', 'anything')


def test_cpu_sample_holds_name_and_percent():
    """결과 표본이 이름과 값을 함께 담는다."""
    sample = CpuSample(name='imu', percent=38.6)
    assert sample.name == 'imu'
    assert sample.percent == 38.6
