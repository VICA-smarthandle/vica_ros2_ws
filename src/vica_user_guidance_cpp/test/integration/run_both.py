"""실제 노드 대조(수동 시험) — 파이썬 판과 C++ 판을 차례로 띄워 같은 각본을 주고 관측을 견준다."""
import json
import os
import pty
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time
import tty

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from vica_interfaces.msg import SmartHandleState, TurnGuide

SRC = os.environ.get('GUIDANCE_PY_SRC', '/home/ji_w/wt-guidance-cpp/src/vica_user_guidance')
INSTALL = os.environ.get('GUIDANCE_INSTALL', '/home/ji_w/wt-guidance-cpp/install')
WORK = tempfile.mkdtemp(prefix='guidance_integ_')
LINK = os.path.join(WORK, 'vica_smart_handle')
END = 26.0
CHANNEL_FRAMES = ['usonic_left_wheel', 'usonic_front_left', 'usonic_front_right', 'usonic_right_wheel',
                  'usonic_right_rear_wheel', 'usonic_rear_right', 'usonic_rear_left',
                  'usonic_left_rear_wheel']


def xor(b):
    x = 0
    for v in b:
        x ^= v
    return x


def frame(h1, body):
    return bytes([0xAA, h1]) + body + bytes([xor(body)])


def us_frame(seq, dists):
    return frame(0x57, bytes([seq]) + b''.join(struct.pack('<H', d) for d in dists))


def touch_frame(seq, touched):
    return frame(0x56, bytes([seq, 1 if touched else 0]))


class Device:
    """손잡이 보드 흉내: pty 마스터 쪽. 아래로 온 바이트를 시각과 함께 적는다."""

    def __init__(self):
        self.lock = threading.Lock()
        self.down = []
        self.fd = None
        self.plug()

    def plug(self):
        master, slave = pty.openpty()
        name = os.ttyname(slave)
        tty.setraw(slave)
        os.close(slave)
        os.set_blocking(master, False)
        if os.path.lexists(LINK):
            os.unlink(LINK)
        os.symlink(name, LINK)
        self.fd = master

    def unplug(self):
        fd, self.fd = self.fd, None
        if os.path.lexists(LINK):
            os.unlink(LINK)
        if fd is not None:
            os.close(fd)

    def poll(self, t):
        if self.fd is None:
            return
        try:
            data = os.read(self.fd, 4096)
        except (BlockingIOError, OSError):
            return
        with self.lock:
            self.down.extend((t, b) for b in data)

    def up(self, data):
        if self.fd is None:
            return
        try:
            os.write(self.fd, data)
        except OSError:
            pass


class Harness(Node):
    def __init__(self):
        super().__init__('integ_harness')
        self.t0 = time.monotonic()
        self.p_estop = self.create_publisher(Bool, '/estop_state', 10)
        self.p_turn = self.create_publisher(TurnGuide, '/vica/turn_guide', 10)
        self.p_goal = self.create_publisher(String, '/vica_goal_event', 10)
        self.p_haptic = self.create_publisher(String, '/vica/haptic_request', 10)
        self.p_tf = self.create_publisher(TFMessage, '/tf', 100)
        self.p_tf_static = self.create_publisher(
            TFMessage, '/tf_static', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.ranges = []
        self.states = []
        self.logs = []
        for name in ['left_wheel', 'front_left', 'front_right', 'right_wheel', 'right_rear_wheel',
                     'rear_right', 'rear_left', 'left_rear_wheel']:
            self.create_subscription(Range, f'/ultrasonic/{name}',
                                     lambda m, n=name: self.ranges.append((self.t(), n, m)), 10)
        self.create_subscription(SmartHandleState, '/vica/smart_handle_state',
                                 lambda m: self.states.append((self.t(), m)), 10)
        self.create_subscription(Log, '/rosout', self._log, 100)
        static = TFMessage()
        for fid in CHANNEL_FRAMES:
            ts = TransformStamped()
            ts.header.frame_id = 'base_footprint'
            ts.child_frame_id = fid
            ts.transform.rotation.w = 1.0
            static.transforms.append(ts)
        self.p_tf_static.publish(static)

    def t(self):
        return time.monotonic() - self.t0

    def _log(self, m):
        if m.name == 'user_guidance_driver_node':
            self.logs.append((self.t(), m.level, m.msg))

    def tf_burst(self, odom_ok):
        msg = TFMessage()
        now = self.get_clock().now().to_msg()
        for k in range(3):
            ts = TransformStamped()
            ts.header.stamp = now
            if k == 0 and odom_ok:
                ts.header.frame_id, ts.child_frame_id = 'odom', 'base_footprint'
            else:
                ts.header.frame_id, ts.child_frame_id = 'map', f'noise_{k}'
            ts.transform.rotation.w = 1.0
            msg.transforms.append(ts)
        self.p_tf.publish(msg)


def proc_cpu_seconds(pid):
    try:
        with open(f'/proc/{pid}/stat') as f:
            parts = f.read().rsplit(')', 1)[1].split()
        return (int(parts[11]) + int(parts[12])) / os.sysconf('SC_CLK_TCK')
    except OSError:
        return None


def run(impl, domain):
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    with open(os.path.join(SRC, 'config', 'user_guidance.yaml'), encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    params = cfg['user_guidance_driver_node']['ros__parameters']
    params['serial_port'] = LINK
    pfile = os.path.join(WORK, f'params_{impl}.yaml')
    with open(pfile, 'w', encoding='utf-8') as f:
        yaml.safe_dump({'user_guidance_driver_node': {'ros__parameters': params}}, f, allow_unicode=True)

    dev = Device()
    rclpy.init()
    h = Harness()
    pkg = 'vica_user_guidance_cpp' if impl == 'cpp' else 'vica_user_guidance'
    exe = os.path.join(INSTALL, pkg, 'lib', pkg, 'user_guidance_driver_node')
    proc = subprocess.Popen(
        [exe, '--ros-args', '--params-file', pfile],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    node_pid = proc.pid
    seq = {'us': 0, 'touch': 0}
    estop_on, estop_val, turn = True, False, 0
    cpu = {}
    last_tf = last_estop = last_us = last_touch = 0.0
    events = [
        (2.0, 'turn', 1), (4.0, 'turn', 0), (4.5, 'turn', 2), (6.0, 'turn', 0),
        (6.5, 'goal', '{"event": "goal_succeeded", "x": NaN, "name": "407호"}'),
        (7.0, 'goal', 'not json'),
        (8.0, 'estop', True), (9.0, 'estop', False),
        (11.0, 'haptic', 'long'), (11.5, 'haptic', 'bogus'),
        (12.0, 'odom', False), (13.5, 'odom', True),
        (14.0, 'touch', True), (16.0, 'touch', False),
        (17.0, 'unplug', None), (18.5, 'plug', None),
        (21.5, 'haptic', 'tick'),
        (22.5, 'estop_off', None),
    ]
    odom_ok, touched = True, False
    try:
        while h.t() < END:
            t = h.t()
            while events and events[0][0] <= t:
                _, kind, arg = events.pop(0)
                if kind == 'turn':
                    turn = arg
                elif kind == 'goal':
                    h.p_goal.publish(String(data=arg))
                elif kind == 'estop':
                    estop_val = arg
                elif kind == 'haptic':
                    h.p_haptic.publish(String(data=arg))
                elif kind == 'odom':
                    odom_ok = arg
                elif kind == 'touch':
                    touched = arg
                elif kind == 'unplug':
                    dev.unplug()
                elif kind == 'plug':
                    dev.plug()
                elif kind == 'estop_off':
                    estop_on = False
            if t - last_tf >= 0.02:
                last_tf = t
                h.tf_burst(odom_ok)
            if estop_on and t - last_estop >= 0.05:
                last_estop = t
                h.p_estop.publish(Bool(data=estop_val))
                msg = TurnGuide()
                msg.direction = turn
                h.p_turn.publish(msg)
            if t - last_us >= 0.21:
                last_us = t
                seq['us'] = (seq['us'] + 1) % 256
                dev.up(us_frame(seq['us'], [0, 250, 3001, 0, 0, 0, 0, 0]))
            if t - last_touch >= 0.05:
                last_touch = t
                seq['touch'] = (seq['touch'] + 1) % 256
                dev.up(touch_frame(seq['touch'], touched))
            dev.poll(t)
            for mark in (3.0, 11.0):
                if mark not in cpu and t >= mark and node_pid:
                    cpu[mark] = proc_cpu_seconds(node_pid)
            rclpy.spin_once(h, timeout_sec=0.005)
        proc.send_signal(signal.SIGINT)
        end = time.monotonic() + 5
        while time.monotonic() < end:
            dev.poll(h.t())
            rclpy.spin_once(h, timeout_sec=0.05)
            if proc.poll() is not None:
                break
        dev.poll(h.t())
        out = proc.communicate(timeout=10)[0] if proc.poll() is not None else ''
    finally:
        if proc.poll() is None:
            proc.kill()
        dev.unplug()
    summary = summarize(impl, h, dev, cpu, proc.returncode, out)
    h.destroy_node()
    rclpy.shutdown()
    return summary


def collapse(seq):
    out = []
    for b in seq:
        if not out or out[-1][0] != b:
            out.append([b, 1])
        else:
            out[-1][1] += 1
    return out


def summarize(impl, h, dev, cpu, rc, out):
    down = [b for _, b in dev.down]
    states = collapse(down)
    level = {10: 'D', 20: 'I', 30: 'W', 40: 'E', 50: 'F'}
    logs = [f'{level.get(lv, lv)} {msg}' for _, lv, msg in h.logs]
    ranges = {}
    for _, n, m in h.ranges:
        key = f'{n} {m.header.frame_id} type={m.radiation_type} fov={m.field_of_view:.6f} ' \
              f'min={m.min_range:.3f} max={m.max_range:.3f} range={m.range:.3f}'
        ranges[key] = ranges.get(key, 0) + 1
    st = []
    for _, m in h.states:
        k = (f'connected={int(m.connected)} uplink_fresh={int(m.uplink_fresh)} '
             f'contact={int(m.user_contact)} fault={m.fault_code} last={m.last_state_code} '
             f'werr={m.write_error_count}')
        if not st or st[-1][0] != k:
            st.append([k, 1])
        else:
            st[-1][1] += 1
    cpu_pct = None
    if cpu.get(3.0) is not None and cpu.get(11.0) is not None:
        cpu_pct = round((cpu[11.0] - cpu[3.0]) / 8.0 * 100, 1)
    return {
        'impl': impl, 'exit': rc, 'cpu_pct_3_11s': cpu_pct,
        'downlink_runs': [f'{b:02x}x{n}' for b, n in states],
        'last_byte': f'{down[-1]:02x}' if down else None,
        'logs': logs, 'ranges': ranges, 'state_runs': [f'{k} ×{n}' for k, n in st],
        'range_count': len(h.ranges), 'state_count': len(h.states),
        'stdout_tail': out.strip().splitlines()[-3:],
    }


def main():
    res = {impl: run(impl, dom) for impl, dom in (('python', 61), ('cpp', 62))}
    print(json.dumps(res, ensure_ascii=False, indent=1))
    py, cpp = res['python'], res['cpp']
    print('\n==== 차이')
    for key in ('exit', 'last_byte', 'ranges'):
        print(f'{key}: {"같음" if py[key] == cpp[key] else f"다름 py={py[key]} cpp={cpp[key]}"}')
    for key in ('downlink_runs', 'logs', 'state_runs'):
        a = [x.split('x')[0] if key == 'downlink_runs' else x.rsplit(' ×', 1)[0] for x in py[key]]
        b = [x.split('x')[0] if key == 'downlink_runs' else x.rsplit(' ×', 1)[0] for x in cpp[key]]
        print(f'{key} 순서: {"같음" if a == b else "다름"}')
    print(f'CPU(3~11 s, /tf 150 건/s): python {py["cpu_pct_3_11s"]} % · cpp {cpp["cpu_pct_3_11s"]} %')


if __name__ == '__main__':
    sys.exit(main())
