"""정답지(방향지시): 파이썬 원본 TurnGuideNode 의 메서드를 시나리오대로 부르고 관측을 줄로 찍는다.

    python3 turn_gen.py 3 2000 | python3 turn_law_py.py      (GUIDANCE_PY_SRC = vica_user_guidance 소스 폴더)

원본 코드는 고치지 않는다. 진짜 rclpy 노드를 만들어(파라미터는 YAML 로) __init__ 을 그대로 돌리고,
바깥과 닿는 곳만 기록용으로 바꾼다: 로거·정상 시계(now_ns)·ROS 시계·발행자. TF 는 노드가 만든 진짜
tf2_ros.Buffer 에 시나리오의 변환을 직접 넣는다(리스너 스레드는 만들지 않는다 — 시나리오가 순서를 쥔다).
같은 시나리오를 turn_law_cpp(C++ 코어 + tf2::BufferCore)에 넣어 나온 줄과 글자 단위로 비교한다.

명령(turn_gen.py 가 만든다)
  P <json>                 파라미터(없는 키는 선언 기본값)
  start                    노드 생성
  T <ns> / R <ns>          정상 시계 / ROS 시계
  odom <qx> <qy> <qz> <qw> <vx>
  rail <frame hex> <n> <x0> <y0> …
  tf <parent hex> <child hex> <sec> <nsec> <x> <y> <z> <qx> <qy> <qz> <qw> <static 0|1>
  tick                     publish_loop
  idle                     publish_idle_once
출력
  L <I|W|E> <글> · RS <arbiter 판정 이유>
  G <sec> <nsec> <frame> <dir> <phase> <dist f32> <angle f32> <seq> <vsec> <vnsec> <stale>
  X <예외 이름>: <글>      (그 뒤 파이썬 main 의 finally 처럼 publish_idle_once)
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
from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.time import Time  # noqa: E402

from vica_user_guidance import turn_guide_node as tg  # noqa: E402

OUT = []


def emit(line):
    OUT.append(line)


def esc(text):
    return text.replace('\\', '\\\\').replace('\n', '\\n').replace('\r', '\\r')


class State:
    t = 0
    r = 0


ST = State()


class FakeLogger:
    def info(self, msg, **kw):
        emit(f'L I {esc(msg)}')

    def warn(self, msg, **kw):
        emit(f'L W {esc(msg)}')

    warning = warn

    def error(self, msg, **kw):
        emit(f'L E {esc(msg)}')


LOGGER = FakeLogger()


class FakeClock:
    def now(self):
        return Time(nanoseconds=ST.r)


def f32(v):
    return struct.pack('<f', v).hex()


class FakePub:
    def publish(self, m):
        emit(f'G {m.header.stamp.sec} {m.header.stamp.nanosec} {m.header.frame_id} '
             f'{m.direction} {m.phase} {f32(m.distance_m)} {f32(m.turn_angle_deg)} '
             f'{m.sequence_id} {m.valid_until.sec} {m.valid_until.nanosec} {int(m.source_stale)}')


def build_node(params):
    tg.TurnGuideNode.get_logger = lambda self: LOGGER
    tg.TurnGuideNode.now_ns = lambda self: ST.t
    tg.TransformListener = lambda *a, **k: None
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False, encoding='utf-8') as f:
        yaml.safe_dump({'turn_guide_node': {'ros__parameters': params}}, f, allow_unicode=True)
        path = f.name
    try:
        rclpy.init(args=['--ros-args', '--params-file', path])
        node = tg.TurnGuideNode()
    finally:
        os.unlink(path)
    node.get_clock = lambda: FakeClock()
    node.pub_guide = FakePub()
    if node.rail_enabled:
        resolve = node.arbiter.resolve

        def traced(*a, **k):
            cue = resolve(*a, **k)
            emit(f'RS {cue.reason}')
            return cue

        node.arbiter.resolve = traced
    return node


def text(hexstr):
    return bytes.fromhex(hexstr).decode('utf-8')


def set_tf(node, parts):
    parent, child = text(parts[0]), text(parts[1])
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.header.stamp.sec = int(parts[2])
    t.header.stamp.nanosec = int(parts[3])
    v = [float(x) for x in parts[4:11]]
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = v[:3]
    r = t.transform.rotation
    r.x, r.y, r.z, r.w = v[3:]
    if not node.rail_enabled:
        return   # 레일 예고가 꺼지면 TF 버퍼 자체가 없다
    if parts[11] == '1':
        node.tf_buffer.set_transform_static(t, 'default_authority')
    else:
        node.tf_buffer.set_transform(t, 'default_authority')


def run(lines):
    node = None
    params = {}
    for raw in lines:
        parts = raw.rstrip('\n').split(' ')
        cmd = parts[0]
        if not cmd:
            continue
        try:
            if cmd == 'P':
                params = json.loads(raw[2:])
            elif cmd == 'T':
                ST.t = int(parts[1])
            elif cmd == 'R':
                ST.r = int(parts[1])
            elif cmd == 'start':
                node = build_node(params)
            elif cmd == 'odom':
                qx, qy, qz, qw, vx = (float(x) for x in parts[1:6])
                node.odom_callback(SimpleNamespace(
                    pose=SimpleNamespace(pose=SimpleNamespace(
                        orientation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw))),
                    twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=vx)))))
            elif cmd == 'rail':
                n = int(parts[2])
                vals = [float(x) for x in parts[3:3 + 2 * n]]
                poses = [SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(
                    x=vals[2 * i], y=vals[2 * i + 1]))) for i in range(n)]
                if node.rail_enabled:   # 꺼지면 구독이 없다
                    node.rail_callback(SimpleNamespace(
                        header=SimpleNamespace(frame_id=text(parts[1])), poses=poses))
            elif cmd == 'tf':
                set_tf(node, parts[1:])
            elif cmd == 'tick':
                node.publish_loop()
            elif cmd == 'idle':
                node.publish_idle_once()
            else:
                raise SystemExit(f'모르는 명령: {cmd}')
        except SystemExit:
            raise
        except Exception as exc:  # noqa: BLE001 — 파이썬이 멈췄을 자리
            emit(f'X {type(exc).__name__}: {esc(str(exc))}')
            if node is not None:
                try:
                    node.publish_idle_once()
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
