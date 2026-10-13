"""방향지시 실제 노드 대조(수동 시험) — 파이썬 판과 C++ 판을 함께 띄우고 실주행 녹화를 실시간으로 튼다.

    unshare -rn bash -c 'ip link set lo up; python3 turn_run_both.py <bag> [시작초] [길이초]'

대조 시험(test/parity)은 판정 코어만 본다. 여기서는 ROS 배선을 본다: YAML 파라미터 읽기·타이머 주기·
전용 스레드 TF 리스너·구독 QoS·발행·Ctrl+C 종료. 두 판은 같은 도메인에서 같은 재생(/odom·/tf·/tf_static·
/rail_plan)을 받고, 출력만 /vica/turn_guide_py · /vica/turn_guide_cpp 로 갈라 낸다. 같은 /tf 부하에서의 CPU 도 잰다.
그 뒤 기동 실패(publish_rate_hz:=0.0) 때의 종료 코드·stderr 를 두 판에서 본다.
"""
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
import yaml
from rclpy.node import Node
from vica_interfaces.msg import TurnGuide

SRC = os.environ.get('GUIDANCE_PY_SRC', '/home/ji_w/wt-guidance-cpp/src/vica_user_guidance')
INSTALL = os.environ.get('GUIDANCE_INSTALL', '/home/ji_w/wt-guidance-cpp/install')
EXE = {
    'py': os.path.join(INSTALL, 'vica_user_guidance', 'lib', 'vica_user_guidance', 'turn_guide_node'),
    'cpp': os.path.join(INSTALL, 'vica_user_guidance_cpp', 'lib', 'vica_user_guidance_cpp',
                        'turn_guide_node'),
}
CLK = os.sysconf('SC_CLK_TCK')


def cpu_ticks(pid):
    with open(f'/proc/{pid}/stat') as f:
        parts = f.read().rsplit(')', 1)[1].split()
    return int(parts[11]) + int(parts[12])


def params_file(work, extra=None):
    cfg = yaml.safe_load(open(os.path.join(SRC, 'config', 'user_guidance.yaml'), encoding='utf-8'))
    p = cfg['turn_guide_node']['ros__parameters']
    p.update(extra or {})
    path = os.path.join(work, 'turn.yaml')
    yaml.safe_dump({'turn_guide_node': {'ros__parameters': p}}, open(path, 'w'), allow_unicode=True)
    return path


class Probe(Node):
    def __init__(self):
        super().__init__('turn_probe')
        self.rows = {'py': [], 'cpp': []}
        for k in self.rows:
            self.create_subscription(TurnGuide, f'/vica/turn_guide_{k}',
                                     lambda m, k=k: self.rows[k].append((time.monotonic(), m)), 50)


def onsets(rows):
    out, last = [], None
    for t, m in rows:
        key = (m.sequence_id, m.direction, m.phase)
        if m.phase in (1, 2) and key != last:
            out.append((t, m.sequence_id, m.direction, m.phase))
        last = key
    return out


def compare(a, b):
    used, lags = set(), []
    for t, _, d, p in a:
        best = None
        for j, (t2, _, d2, p2) in enumerate(b):
            if j in used or d2 != d or p2 != p or abs(t2 - t) > 0.3:
                continue
            if best is None or abs(t2 - t) < abs(b[best][0] - t):
                best = j
        if best is not None:
            used.add(best)
            lags.append(b[best][0] - t)
    return len(used), lags


def drive(bag, offset, duration):
    work = tempfile.mkdtemp(prefix='turn_integ_')
    pf = params_file(work)
    procs, logs = {}, {}
    for k, exe in EXE.items():
        logs[k] = open(os.path.join(work, f'{k}.log'), 'w')
        procs[k] = subprocess.Popen(
            [exe, '--ros-args', '--params-file', pf, '-r', f'/vica/turn_guide:=/vica/turn_guide_{k}'],
            stdout=logs[k], stderr=subprocess.STDOUT)
    rclpy.init()
    probe = Probe()
    spin = threading.Thread(target=rclpy.spin, args=(probe,), daemon=True)
    spin.start()
    time.sleep(3.0)
    play = subprocess.Popen(
        ['ros2', 'bag', 'play', bag, '--topics', '/odom', '/tf', '/tf_static', '/rail_plan',
         '--start-offset', str(offset), '--disable-keyboard-controls'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(5.0)
    t0 = time.monotonic()
    c0 = {k: cpu_ticks(p.pid) for k, p in procs.items()}
    time.sleep(duration)
    c1 = {k: cpu_ticks(p.pid) for k, p in procs.items()}
    t1 = time.monotonic()
    play.send_signal(signal.SIGINT)
    play.wait(timeout=20)
    time.sleep(1.5)
    stale = {k: [m.source_stale for _, m in probe.rows[k][-5:]] for k in procs}
    n_before = {k: len(probe.rows[k]) for k in procs}
    for p in procs.values():
        p.send_signal(signal.SIGINT)
    codes = {k: p.wait(timeout=20) for k, p in procs.items()}
    time.sleep(1.0)
    finals = {k: sum(1 for _, m in probe.rows[k][n_before[k]:] if m.valid_until == m.header.stamp)
              for k in procs}
    probe.destroy_node()
    rclpy.shutdown()
    for f in logs.values():
        f.close()

    print(f'== 재생 {bag} +{offset}s, {duration:.0f}s 측정')
    for k in procs:
        cpu = 100.0 * (c1[k] - c0[k]) / CLK / (t1 - t0)
        print(f'  {k:3}: 메시지 {len(probe.rows[k])} · 신호 시작 {len(onsets(probe.rows[k]))} · CPU {cpu:.1f} % '
              f'· 재생 끝 뒤 stale {stale[k]} · Ctrl+C 종료 코드 {codes[k]} · 마지막 IDLE {finals[k]}')
    a, b = onsets(probe.rows['py']), onsets(probe.rows['cpp'])
    matched, lags = compare(a, b)
    lags.sort()
    print(f'  신호 시작 짝: {matched}/{len(a)} (C++ 쪽 {len(b)}), 시각 차(C++ − 파이썬) 중앙 '
          f'{lags[len(lags) // 2] * 1000 if lags else float("nan"):.0f} ms · 범위 '
          f'{min(lags, default=float("nan")) * 1000:.0f} ~ {max(lags, default=float("nan")) * 1000:.0f} ms')
    for k in procs:
        warns = [ln.strip() for ln in open(os.path.join(work, f'{k}.log'), encoding='utf-8', errors='replace')
                 if 'WARN' in ln or 'Error' in ln or 'Traceback' in ln]
        print(f'  {k} 로그 경고·오류 {len(warns)}: {warns[:2]}')
        lines = open(os.path.join(work, f'{k}.log'), encoding='utf-8', errors='replace').read().splitlines()
        if any('Traceback' in ln for ln in lines):
            print(f'    {k} 트레이스백 마지막 줄: {[ln for ln in lines if ln.strip()][-1][:160]}')


def startup_error():
    work = tempfile.mkdtemp(prefix='turn_integ_err_')
    pf = params_file(work, {'publish_rate_hz': 0.0})
    print('== 기동 실패(publish_rate_hz: 0.0)')
    for k, exe in EXE.items():
        p = subprocess.Popen([exe, '--ros-args', '--params-file', pf],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            out, _ = p.communicate(timeout=10)
            code = p.returncode
        except subprocess.TimeoutExpired:
            p.kill()
            out, _ = p.communicate()
            code = '10초 안에 안 끝남(강제 종료)'
        last = [ln for ln in out.splitlines() if ln.strip()][-1:]
        print(f'  {k:3}: 종료 코드 {code} · 마지막 줄 {last}')


if __name__ == '__main__':
    drive(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 60.0,
          float(sys.argv[3]) if len(sys.argv) > 3 else 180.0)
    startup_error()
