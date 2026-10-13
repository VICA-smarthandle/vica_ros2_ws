"""실주행 녹화(rosbag2) → 방향지시 대조 시나리오 — python3 turn_bag_scenario.py <bag 폴더> [params.json]

녹화의 /odom·/tf·/tf_static·/rail_plan 을 녹화 시각 순서대로 시나리오 명령으로 바꾸고, 50 ms 마다 tick 을
끼운다(정상 시계·ROS 시계 = 녹화 시각). 같은 시나리오를 turn_law_py.py·turn_law_cpp 에 넣어 줄 단위로 비교한다.
--ref 를 주면 녹화에 남은 실제 노드의 /vica/turn_guide 를 (시각 seq dir phase) 줄로 따로 찍는다.

**녹화 중인 bag 은 읽지 않는다**(쓰는 중인 sqlite 를 읽으면 녹화가 깨질 수 있다). 끝난 녹화만 넣는다.
"""
import json
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ['/odom', '/tf', '/tf_static', '/rail_plan']
TICK_NS = 50_000_000


def hx(s):
    return s.encode('utf-8').hex()


def fr(x):
    return repr(float(x))


def open_reader(bag, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    return reader, {k: get_message(v) for k, v in types.items() if k in topics}


def scenario(bag, params):
    reader, types = open_reader(bag, TOPICS)
    out = [f'P {json.dumps(params, ensure_ascii=False)}']
    started = False
    next_tick = None
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if not started:
            out += [f'T {t_ns}', f'R {t_ns}', 'start']
            started = True
            next_tick = t_ns + TICK_NS
        while next_tick <= t_ns:
            out += [f'T {next_tick}', f'R {next_tick}', 'tick']
            next_tick += TICK_NS
        msg = deserialize_message(data, types[topic])
        out += [f'T {t_ns}', f'R {t_ns}']
        if topic == '/odom':
            q = msg.pose.pose.orientation
            out.append(f'odom {fr(q.x)} {fr(q.y)} {fr(q.z)} {fr(q.w)} {fr(msg.twist.twist.linear.x)}')
        elif topic == '/rail_plan':
            body = ' '.join(f'{fr(p.pose.position.x)} {fr(p.pose.position.y)}' for p in msg.poses)
            out.append(f'rail {hx(msg.header.frame_id)} {len(msg.poses)} {body}'.rstrip())
        else:
            static = '1' if topic == '/tf_static' else '0'
            for t in msg.transforms:
                tr, r = t.transform.translation, t.transform.rotation
                out.append(
                    f'tf {hx(t.header.frame_id)} {hx(t.child_frame_id)} {t.header.stamp.sec} '
                    f'{t.header.stamp.nanosec} {fr(tr.x)} {fr(tr.y)} {fr(tr.z)} {fr(r.x)} {fr(r.y)} '
                    f'{fr(r.z)} {fr(r.w)} {static}')
    out.append('idle')
    return '\n'.join(out) + '\n'


def reference(bag):
    reader, types = open_reader(bag, ['/vica/turn_guide'])
    out = []
    while reader.has_next():
        _, data, t_ns = reader.read_next()
        m = deserialize_message(data, types['/vica/turn_guide'])
        out.append(f'{t_ns} {m.sequence_id} {m.direction} {m.phase} {int(m.source_stale)}')
    return '\n'.join(out) + '\n'


def main():
    if sys.argv[1] == '--ref':
        sys.stdout.write(reference(sys.argv[2]))
        return
    params = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
    sys.stdout.write(scenario(sys.argv[1], params))


if __name__ == '__main__':
    main()
