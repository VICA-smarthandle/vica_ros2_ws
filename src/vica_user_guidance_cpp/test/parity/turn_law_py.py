"""정답지(방향지시): 파이썬 원본 TurnGuideNode 의 메서드를 시나리오대로 부르고 관측을 줄로 찍는다."""
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
        return
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
                if node.rail_enabled:
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
