"""대조 시험 시나리오 생성기 — python3 gen.py <seed> <cycles> [params.json]

한 줄에 명령 하나. 첫 줄은 P <파라미터 JSON>, 그다음 기동 전 명령, start, 그 뒤 50 ms 칸마다의 사건.
글자 인자(도착 이벤트·햅틱·프레임 이름)는 UTF-8 바이트의 16진수로 적는다. 명령 목록은 law_py.py 머리말.
"""
import json
import random
import struct
import sys

T0 = 1_000_000_000_000          # 정상 시계 시작(1000 s)
R0 = 1_791_000_000_000_000_000  # ROS 시계 시작(2026-10)

GOALS = [
    '{"event": "goal_succeeded", "map_id": "map_1002_150946", "x": 5.58, "y": -1.96}',
    '{"event": "goal_sent", "name": "홈"}',
    '{"event": "goal_accepted"}',
    '{"event": "goal_canceled", "reason": "estop"}',
    '{"event": "goal_failed"}',
    '{"event": "return_home_succeeded"}',
    '{"event":"goal_succeeded","x":NaN}',
    '{"event":"goal_succeeded","y":-Infinity,"z":Infinity}',
    'not json',
    '',
    '[1, 2]',
    '{"event": 5}',
    '{"name": "x"}',
    '﻿{"event":"goal_succeeded"}',
    '{"event":"goal_succeeded"} x',
    '{"event":"a","event":"goal_succeeded"}',
    '{"event":"goal_succeeded","event":1}',
    '{"event":"goal_\\u0073ucceeded"}',
    '{"event": "goal_succeeded", "name": "화장실 \' \\" 앞 ' + "가" * 130 + '"}',
    '{"event": "goal_sent", "name": "\'따옴표\' 하나"}',
    '{"event":"goal_succeeded",}',
    '{"a":[1,2,{"b":null,"c":[true,false]}],"event":"goal_succeeded"}',
    '{"event":"goal_succeeded","n":1e400}',
    '{"event":"goal_succeeded","n":-0}',
    '{"event":"goal_succeeded","n":01}',
    '{"event":"\\ud800"}',
    '{"event":"\\ud83d\\ude00"}',
    '{"event":"\\ud800\\u0041"}',
    '{"event":"\\ud800\\uZZZZ"}',
    ' {"event":"goal_succeeded"} \n',
    '{"event":"goal_succeeded\t"}',
    'NaN',
    '"goal_succeeded"',
    'null',
    '{"event":"goal_succeeded","x":tru}',
    "{'event':'goal_succeeded'}",
    '{"event":"goal_succeeded","x":.5}',
    '{"event":"goal_succeeded","x":5.}',
    '{"event":"goal_succeeded","x":1E+5,"y":2e-3}',
    '{"event":"goal_succeeded","x":-}',
    '{"event":"goal_succeeded","x":infinity}',
    '{"event":"goal_succeeded","x":-Infinity}',
    '{"event" : "goal_succeeded" , "x" : [ ] , "y" : { } }',
    '{"event":"goal_succeeded","k":"\\b\\f\\n\\r\\t\\/\\\\"}',
    '{"event":"goal_succeeded","k":"\\x"}',
    '\x7f\x85\xa0​ 　\U0001F600 깨진 글',
    '{"event":"goal_succeeded"}\x00',
    '[' * 40 + ']' * 40,
    '{"e":' * 30 + '1' + '}' * 30,
    '{"event":"goal_succeeded","deep":' + '[' * 300 + ']' * 300 + '}',
    '{"event": "goal_succeeded", "name": "a\\"b\'c"}',
    '\t\r\n {"event":"goal_succeeded"}',
    '{"event":"goal_succeeded"',
    '{"event":"goal_succeeded" "x":1}',
    '{"event":"goal_succeeded", 1:2}',
    '{"event":"goal_succeeded","x":1.5e}',
    '{"event":"goal_succeeded","x":123456789012345678901234567890}',
]

HAPTICS = [
    'short', 'long', 'tick', 'locate', ' Long ', 'TICK', '\tlocate\n', 'locate　',
    'ticK', 'LOCATE\x1c', 'sh ort', '', 'unknown', '진동', 'x' * 50, "it's",
    'short\x85', ' long', 'lоng', 'Short ', '​long', 'LoCaTe',
]

DIRECTIONS = [0, 1, 2, 1, 2, 0, 3, 255]


def xor(body):
    x = 0
    for b in body:
        x ^= b
    return x


def frame(header, body):
    return header + body + bytes([xor(body)])


def us_frame(seq, dists):
    return frame(b'\xaa\x57', bytes([seq]) + b''.join(struct.pack('<H', d) for d in dists))


def touch_frame(seq, flags):
    return frame(b'\xaa\x56', bytes([seq, flags]))


def stat_frame(seq, counts):
    return frame(b'\xaa\x58', bytes([seq]) + bytes(counts))


def cfg_frame(seq, levels):
    return frame(b'\xaa\x59', bytes([seq]) + bytes(levels))


def legacy_frame(seq, d0, d1):
    return frame(b'\xaa\x55', bytes([seq]) + struct.pack('<HH', d0, d1))


def rand_dist(rng):
    return rng.choice([
        0, 3001, 3001, 3001, rng.randint(1, 3000), rng.randint(20, 400), 400, 401, 1500, 1501,
        300, 301, 65535, 3002, 1, 2999])


class Gen:
    def __init__(self, seed, params):
        self.rng = random.Random(seed)
        self.params = params
        self.t = T0
        self.r = R0
        self.out = []
        self.seq = {'us': 0, 'touch': 0, 'stat': 0, 'cfg': 0}
        self.touched = False
        self.pending = b''          # 다음 수신으로 넘길 쪼개진 꼬리
        self.estop_on = True        # /estop_state 를 보내는 중인가(끊기면 stale)
        self.estop_val = False
        frame_ids = params.get('ultrasonic_frame_ids', ['usonic_front_left', 'usonic_front_right'])
        self.frame_ids = sorted(set(frame_ids))

    def emit(self, line):
        self.out.append(line)

    def uplink_bytes(self):
        rng = self.rng
        chunk = b''
        for _ in range(rng.choice([0, 1, 1, 1, 2, 3])):
            kind = rng.random()
            if kind < 0.35:
                self.seq['us'] = (self.seq['us'] + 1) % 256
                f = us_frame(self.seq['us'], [rand_dist(rng) for _ in range(8)])
            elif kind < 0.75:
                self.seq['touch'] = (self.seq['touch'] + 1) % 256
                if rng.random() < 0.15:
                    self.touched = not self.touched
                f = touch_frame(self.seq['touch'], (1 if self.touched else 0) |
                                (rng.choice([0, 0, 0, 2, 0x80]) if rng.random() < 0.1 else 0))
            elif kind < 0.80:
                self.seq['stat'] = (self.seq['stat'] + 1) % 256
                f = stat_frame(self.seq['stat'], [rng.choice([0, 1, 12, 255, rng.randint(0, 255)])
                                                  for _ in range(48)])
            elif kind < 0.83:
                self.seq['cfg'] = (self.seq['cfg'] + 1) % 256
                f = cfg_frame(self.seq['cfg'], [rng.choice([1, 2, 3, 0xFF]) for _ in range(16)])
            elif kind < 0.90:
                f = legacy_frame(rng.randint(0, 255), rand_dist(rng), rand_dist(rng))
            elif kind < 0.96:
                f = bytes(rng.randint(0, 255) for _ in range(rng.randint(1, 12)))
                if rng.random() < 0.5:
                    f = b'\xaa' + rng.choice([b'\x57', b'\x56', b'\x58', b'\x59']) + f
            else:
                # 체크섬이 깨진 진짜 모양 프레임
                good = rng.choice([us_frame(1, [100] * 8), touch_frame(1, 1), stat_frame(1, [0] * 48)])
                pos = rng.randint(2, len(good) - 1)
                f = good[:pos] + bytes([good[pos] ^ (1 << rng.randint(0, 7))]) + good[pos + 1:]
            chunk += f
        data = self.pending + chunk
        self.pending = b''
        if data and rng.random() < 0.25:
            cut = rng.randint(0, len(data))
            data, self.pending = data[:cut], data[cut:]
        return data

    def build(self, cycles):
        rng = self.rng
        self.emit('P ' + json.dumps(self.params, ensure_ascii=False))
        self.emit(f'T {self.t}')
        self.emit(f'R {self.r}')
        self.openfail = rng.random() < 0.1
        if self.openfail:
            self.emit('openfail 1')
        self.emit('start')
        for cycle in range(cycles):
            # 시계: 보통 50 ms, 가끔 크게 건너뛰거나 조금 거꾸로
            step = 50_000_000
            x = rng.random()
            if x < 0.004:
                step = rng.choice([3_000_000_000, 10_000_000_000, 1_600_000_000])
            elif x < 0.006:
                step = -rng.choice([5_000_000, 60_000_000])
            elif x < 0.05:
                step = rng.randint(1_000_000, 120_000_000)
            self.t += step
            self.r += step if rng.random() > 0.003 else rng.choice([-2_000_000_000, 5_000_000_000])
            self.emit(f'T {self.t}')
            self.emit(f'R {self.r}')

            # 고장 주입 — 열기 실패는 켜지면 곧 풀어 준다(포트가 열린 구간이 대부분이어야 한다)
            if not self.openfail and rng.random() < 0.003:
                self.openfail = True
                self.emit('openfail 1')
            elif self.openfail and rng.random() < 0.02:
                self.openfail = False
                self.emit('openfail 0')
            if rng.random() < 0.003:
                self.emit(f'writefail {rng.choice([1, 2, 5])}')
            if rng.random() < 0.002:
                self.emit(f'flushfail {rng.choice([1, 3])}')
            if rng.random() < 0.002:
                self.emit(f'inwfail {rng.choice([1, 2])}')
            if rng.random() < 0.002:
                self.emit(f'readfail {rng.choice([1, 2])}')
            if rng.random() < 0.003:
                self.emit(f'closefail {rng.choice([0, 1])}')
            for fid in self.frame_ids:
                if rng.random() < 0.01:
                    self.emit(f'tf {fid.encode().hex()} {rng.choice([0, 1, 1, 2])}')

            # 입력
            if rng.random() < 0.01:
                self.estop_on = not self.estop_on
            if self.estop_on and rng.random() < 0.7:
                if rng.random() < 0.02:
                    self.estop_val = not self.estop_val
                self.emit(f'estop {int(self.estop_val)}')
            if rng.random() < 0.08:
                self.emit(f'turn {rng.choice(DIRECTIONS)}')
            if rng.random() < 0.012:
                self.emit('goal ' + rng.choice(GOALS).encode().hex())
            if rng.random() < 0.01:
                self.emit('haptic ' + rng.choice(HAPTICS).encode().hex())

            # 상향 수신 → 타이머들(파이썬 판 주기: 읽기 20 Hz, 전송 10 Hz, 진단 2 Hz)
            data = self.uplink_bytes()
            if data:
                self.emit('rx ' + data.hex())
            self.emit('uplink')
            if cycle % 2 == 0:
                self.emit('send')
            if cycle % 10 == 3:
                self.emit('diag')
        self.emit('shutdown')
        return '\n'.join(self.out) + '\n'


def load_params(path=None):
    if path:
        with open(path, encoding='utf-8') as f:
            return json.load(f)
    return {}


def main():
    seed = int(sys.argv[1])
    cycles = int(sys.argv[2])
    params = json.loads(sys.argv[3]) if len(sys.argv) > 3 else {}
    sys.stdout.write(Gen(seed, params).build(cycles))


if __name__ == '__main__':
    main()
