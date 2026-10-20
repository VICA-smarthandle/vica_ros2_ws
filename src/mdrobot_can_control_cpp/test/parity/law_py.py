"""대조 시험용: 파이썬 원본 모터 노드의 메서드를 가짜 self·가짜 버스로 돌린다."""
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.environ.get(
    'MOTOR_PY_SRC', os.path.join(HERE, '..', '..', '..', 'mdrobot_can_control')))

import can  # noqa: E402
from mdrobot_can_control import mdrobot_can_keyboard_knob_node as M  # noqa: E402
from mdrobot_can_control.can_link import CanLink  # noqa: E402
from mdrobot_can_control.freshness import sec_to_ns  # noqa: E402
from can.interfaces.socketcan.socketcan import build_can_frame  # noqa: E402

N = M.MdrobotCanKeyboardKnobNode
out = sys.stdout
h = types.SimpleNamespace(now=0, rxq=[], fail_send=False, fail_recv=False, reopen_ok=True)


class FakeBus:
    def recv(self, timeout=None):
        if h.fail_recv:
            h.fail_recv = False
            raise can.CanOperationError('Failed to receive: Network is down', 100)
        return h.rxq.pop(0) if h.rxq else None

    def send(self, msg):
        data = build_can_frame(msg)
        if h.fail_send:
            h.fail_send = False
            raise can.CanOperationError('Failed to transmit: No buffer space available', 105)
        out.write(f'S {data.hex()}\n')

    def shutdown(self):
        pass


def bus_factory(channel=None, interface=None):
    if not h.reopen_ok:
        raise OSError(19, 'No such device')
    return FakeBus()


M.can.interface.Bus = bus_factory


class Log:
    def info(self, s):
        out.write(f'I {s}\n')

    def error(self, s):
        out.write(f'E {s}\n')

    def warn(self, s):
        out.write(f'W {s}\n')


class Pub:
    def publish(self, m):
        out.write(f'P {1 if m.data else 0}\n')


f = types.SimpleNamespace()
f.can_iface = 'can1'
f.driver_id = 0x001
f.wheel_radius_m = 0.065
f.wheel_base_m = 0.37
f.max_linear_mps = 1.0
f.max_angular_radps = 2.0
f.max_rpm = 400
f.send_hz = 30.0
f.resend_interval_sec = 0.05
f.deadzone_pct = 5
f.knob_min_pct = 55
f.knob_max_pct = 98
f.knob_timeout_sec = 0.8
f.cmd_timeout_sec = 0.5
f.invert_mot1 = False
f.invert_mot2 = False
f.min_rpm_when_moving = 0
f.can_reconnect_interval_sec = 1.0
INT_PARAMS = ('min_rpm_when_moving', 'max_rpm', 'deadzone_pct', 'knob_min_pct', 'knob_max_pct',
              'driver_id')
for kv in os.environ.get('PARAMS', '').split():
    key, value = kv.split('=')
    if key in ('invert_mot1', 'invert_mot2'):
        setattr(f, key, value == '1')
    elif key in INT_PARAMS:
        setattr(f, key, int(value, 0))
    else:
        setattr(f, key, float(value))
f.knob_timeout_ns = sec_to_ns(f.knob_timeout_sec)
f.cmd_timeout_ns = sec_to_ns(f.cmd_timeout_sec)
f.resend_interval_ns = sec_to_ns(f.resend_interval_sec)
f.can_reconnect_interval_ns = sec_to_ns(f.can_reconnect_interval_sec)
f.knob1 = 0
f.knob2 = 0
f.last_knob_ns = None
f.cmd_linear_x = 0.0
f.cmd_angular_z = 0.0
f.last_cmd_ns = None
f.last_print_ns = None
f.prev_rpm_mot1 = None
f.prev_rpm_mot2 = None
f.last_send_ns = None
f.last_can_error_log_ns = None
f.bus = FakeBus()
f.can_link = CanLink(retry_interval_ns=f.can_reconnect_interval_ns)
f.pub_can_ok = Pub()
_log = Log()
f.get_logger = lambda: _log
f.now_ns = lambda: h.now
for name in ['drain_can_rx', 'send_vel_cmd', 'try_reconnect_can', 'send_pnt_io_monitor_on',
             'log_can_error_throttled', 'mps_to_rpm', 'apply_min_rpm', 'control_loop',
             'cmd_vel_callback']:
    setattr(f, name, types.MethodType(getattr(N, name), f))

for line in sys.stdin:
    k, *a = line.split()
    try:
        if k == 'T':
            h.now = int(a[0])
            out.write(f'-- {h.now}\n')
            f.control_loop()
        elif k == 'C':
            h.now = int(a[0])
            m = types.SimpleNamespace(
                linear=types.SimpleNamespace(x=float(a[1])),
                angular=types.SimpleNamespace(z=float(a[2])))
            f.cmd_vel_callback(m)
        elif k == 'R':
            h.rxq.append(can.Message(
                arbitration_id=int(a[0], 16), is_extended_id=False,
                data=bytes.fromhex(a[2])[:int(a[1])]))
        elif k == 'FS':
            h.fail_send = True
        elif k == 'FR':
            h.fail_recv = True
        elif k == 'RO':
            h.reopen_ok = (a[0] == 'ok')
    except Exception as e:  # noqa: BLE001 - 원본처럼 그 자리에서 끝낸다
        out.write(f'X {e}\n')
        break
