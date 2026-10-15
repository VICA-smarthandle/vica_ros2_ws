"""정답지: 파이썬 원본 UserGuidanceDriverNode 의 메서드를 시나리오대로 부르고 관측을 줄로 찍는다.

    python3 gen.py 3 2000 | python3 law_py.py      (GUIDANCE_PY_SRC = vica_user_guidance 소스 폴더)

원본 코드는 고치지 않는다. 진짜 rclpy 노드를 만들어(파라미터는 YAML 로) __init__ 을 그대로 돌리고,
바깥과 닿는 곳만 기록용으로 바꾼다: 로거·정상 시계(now_ns)·ROS 시계·발행자·TF 버퍼·pyserial.
같은 시나리오를 law_cpp(C++ 코어)에 넣어 나온 줄과 글자 단위로 비교한다(test_parity.py).

명령(gen.py 가 만든다)
  P <json>        파라미터(없는 키는 선언 기본값)
  start           노드 생성
  T <ns> / R <ns> 정상 시계 / ROS 시계
  turn <d> · estop <0|1> · goal <hex> · haptic <hex>     구독 콜백
  send · diag · uplink · shutdown                          타이머·종료
  rx <hex>        열린 포트의 수신 버퍼에 바이트를 쌓는다(포트가 없으면 버림)
  tf <frame hex> <0|1|2>   can_transform 결과(2 = TransformException)
  openfail <0|1> · writefail <n> · flushfail <n> · inwfail <n> · readfail <n> · closefail <0|1>
출력
  OPEN port baud timeout write_timeout · OPENFAIL · W <hex> · WFAIL · FLUSH · FFAIL · IFAIL · RD <n>
  RFAIL · CLOSE · RXDROP <n> · L <I|W|E> <글> · LT <주기> <글> · Q <target> <source> <sec> <nsec>
  PR <topic> <sec> <nsec> <frame> <type> <fov> <min> <max> <range>(float32 바이트 16진수)
  PS <sec> <nsec> <frame> <connected uc uf servo l r haptic> <fault> <last> <werr>
  X <예외 이름>: <글>      (그 뒤 파이썬 main 의 finally 처럼 shutdown_to_neutral)
"""
import json
import os
import struct
import sys
import tempfile
from types import SimpleNamespace

sys.dont_write_bytecode = True
sys.path.insert(0, os.environ['GUIDANCE_PY_SRC'])

import rclpy  # noqa: E402
import yaml  # noqa: E402
from rclpy.time import Time  # noqa: E402
from tf2_ros import TransformException  # noqa: E402

from vica_user_guidance import serial_link as sl  # noqa: E402
from vica_user_guidance import user_guidance_driver_node as drv  # noqa: E402

OUT = []


def emit(line):
    OUT.append(line)


def esc(text):
    return text.replace('\\', '\\\\').replace('\n', '\\n').replace('\r', '\\r')


class State:
    t = 0
    r = 0
    openfail = False
    writefail = 0
    flushfail = 0
    inwfail = 0
    readfail = 0
    closefail = False
    tf = {}
    port = None


ST = State()


class FakeSerial:
    def __init__(self):
        self.buf = bytearray()

    def write(self, data):
        if ST.writefail > 0:
            ST.writefail -= 1
            emit('WFAIL')
            raise OSError('write failed')
        emit('W ' + bytes(data).hex())
        return len(data)

    def flush(self):
        if ST.flushfail > 0:
            ST.flushfail -= 1
            emit('FFAIL')
            raise OSError('flush failed')
        emit('FLUSH')

    @property
    def in_waiting(self):
        if ST.inwfail > 0:
            ST.inwfail -= 1
            emit('IFAIL')
            raise OSError('in_waiting failed')
        return len(self.buf)

    def read(self, n):
        if ST.readfail > 0:
            ST.readfail -= 1
            emit('RFAIL')
            raise OSError('read failed')
        data = bytes(self.buf[:n])
        del self.buf[:n]
        emit(f'RD {len(data)}')
        return data

    def close(self):
        emit('CLOSE')
        if ST.port is self:
            ST.port = None
        if ST.closefail:
            raise OSError('close failed')


def fake_factory(port, baudrate, timeout, write_timeout):
    emit(f'OPEN {port} {baudrate} {float(timeout)!r} {float(write_timeout)!r}')
    if ST.openfail:
        emit('OPENFAIL')
        raise OSError('could not open port')
    ST.port = FakeSerial()
    return ST.port


class FakeLogger:
    def _log(self, level, msg, kw):
        if 'throttle_duration_sec' in kw:
            emit(f'LT {float(kw["throttle_duration_sec"])!r} {esc(msg)}')
        else:
            emit(f'L {level} {esc(msg)}')

    def info(self, msg, **kw):
        self._log('I', msg, kw)

    def warn(self, msg, **kw):
        self._log('W', msg, kw)

    warning = warn

    def error(self, msg, **kw):
        self._log('E', msg, kw)


LOGGER = FakeLogger()


class FakeClock:
    def now(self):
        return Time(nanoseconds=ST.r)


def f32(v):
    return struct.pack('<f', v).hex()


def split_stamp(stamp):
    return f'{stamp.sec} {stamp.nanosec}'


class FakePub:
    def __init__(self, topic):
        self.topic = topic

    def publish(self, msg):
        if self.topic is None:   # SmartHandleState
            b = [msg.connected, msg.user_contact, msg.uplink_fresh, msg.servo_ok,
                 msg.left_led_ok, msg.right_led_ok, msg.haptic_ok]
            emit(f'PS {split_stamp(msg.header.stamp)} {msg.header.frame_id} '
                 + ' '.join(str(int(x)) for x in b)
                 + f' {msg.fault_code} {msg.last_state_code} {msg.write_error_count}')
        else:
            emit(f'PR {self.topic} {split_stamp(msg.header.stamp)} {msg.header.frame_id} '
                 f'{msg.radiation_type} {f32(msg.field_of_view)} {f32(msg.min_range)} '
                 f'{f32(msg.max_range)} {f32(msg.range)}')


class FakeTf:
    def can_transform(self, target, source, time):
        sec, nsec = divmod(time.nanoseconds, 1_000_000_000)
        emit(f'Q {target} {source} {sec} {nsec}')
        mode = ST.tf.get(source, 1)
        if mode == 2:
            raise TransformException('no transform')
        return bool(mode)


def build_node(params):
    drv.UserGuidanceDriverNode.get_logger = lambda self: LOGGER
    drv.UserGuidanceDriverNode.now_ns = lambda self: ST.t
    sl.SerialLink._default_factory = staticmethod(fake_factory)
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False, encoding='utf-8') as f:
        yaml.safe_dump({'user_guidance_driver_node': {'ros__parameters': params}}, f,
                       allow_unicode=True)
        path = f.name
    try:
        rclpy.init(args=['--ros-args', '--params-file', path])
        node = drv.UserGuidanceDriverNode()
    finally:
        os.unlink(path)
    node.get_clock = lambda: FakeClock()
    node.pub_state = FakePub(None)
    if node.us_enabled:
        topics = [str(t) for t in node.get_parameter('ultrasonic_topics').value]
        node.us_pubs = [FakePub(t) if p is not None else None for p, t in zip(node.us_pubs, topics)]
        if node.us_tf_gate_enabled:
            node.tf_buffer = FakeTf()
    return node


def text(hexstr):
    return bytes.fromhex(hexstr).decode('utf-8')


def run(lines):
    node = None
    params = {}
    for raw in lines:
        parts = raw.rstrip('\n').split(' ', 1)
        cmd = parts[0]
        arg = parts[1] if len(parts) > 1 else ''
        if not cmd:
            continue
        try:
            if cmd == 'P':
                params = json.loads(arg)
            elif cmd == 'T':
                ST.t = int(arg)
            elif cmd == 'R':
                ST.r = int(arg)
            elif cmd == 'openfail':
                ST.openfail = arg == '1'
            elif cmd == 'writefail':
                ST.writefail = int(arg)
            elif cmd == 'flushfail':
                ST.flushfail = int(arg)
            elif cmd == 'inwfail':
                ST.inwfail = int(arg)
            elif cmd == 'readfail':
                ST.readfail = int(arg)
            elif cmd == 'closefail':
                ST.closefail = arg == '1'
            elif cmd == 'tf':
                fid, mode = arg.split(' ')
                ST.tf[text(fid)] = int(mode)
            elif cmd == 'rx':
                data = bytes.fromhex(arg)
                if ST.port is not None:
                    ST.port.buf.extend(data)
                else:
                    emit(f'RXDROP {len(data)}')
            elif cmd == 'start':
                node = build_node(params)
            elif cmd == 'turn':
                node.cb_turn(SimpleNamespace(direction=int(arg)))
            elif cmd == 'estop':
                node.cb_estop(SimpleNamespace(data=arg == '1'))
            elif cmd == 'goal':
                node.cb_goal(SimpleNamespace(data=text(arg)))
            elif cmd == 'haptic':
                node.cb_haptic_request(SimpleNamespace(data=text(arg)))
            elif cmd == 'send':
                node.send_loop()
            elif cmd == 'diag':
                node.diag_loop()
            elif cmd == 'uplink':
                node.uplink_loop()
            elif cmd == 'shutdown':
                node.shutdown_to_neutral()
            else:
                raise SystemExit(f'모르는 명령: {cmd}')
        except SystemExit:
            raise
        except Exception as exc:  # noqa: BLE001 — 파이썬이 멈췄을 자리
            emit(f'X {type(exc).__name__}: {esc(str(exc))}')
            if node is not None:
                try:
                    node.shutdown_to_neutral()
                except Exception:  # noqa: BLE001
                    pass
            break


def main():
    run(sys.stdin.read().split('\n'))
    sys.stdout.write('\n'.join(OUT) + '\n')
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
