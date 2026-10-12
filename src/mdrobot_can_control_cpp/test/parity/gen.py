"""대조 시험용 무작위 시나리오. 인자: seed, 사이클 수, [nan].

실제 주기(30 Hz)와 흔들림·끊김·짧은 시간 역전, 경계값 명령(0, -0, 반올림 동점, inf),
F1·C5·잡프레임, 송신·수신 실패와 재개방 실패를 섞는다. nan 이면 가운데에 NaN 명령을 넣는다.
"""
import math
import random
import sys

seed = int(sys.argv[1])
n = int(sys.argv[2])
nan = len(sys.argv) > 3
r = random.Random(seed)
t = 10 ** 12
L = []
circ = 2 * math.pi * 0.065
specials = [0.0, -0.0, 1e-9, -1e-9, 0.5, -0.5, 1.0, -1.0, 1.5, float('inf'), float('-inf'),
            0.123456789, (12.5 / 60) * circ, (-12.5 / 60) * circ, (0.5 / 60) * circ]
for i in range(n):
    t += r.choice([33_333_333] * 20 + [r.randint(20_000_000, 50_000_000),
                                       r.randint(400_000_000, 1_200_000_000),
                                       -r.randint(0, 5_000_000)])
    if r.random() < 0.9:
        lx = r.choice(specials) if r.random() < 0.1 else r.uniform(-1.5, 1.5)
        az = r.choice(specials) if r.random() < 0.1 else r.uniform(-3, 3)
        L.append(f'C {t - r.randint(0, 30_000_000)} {lx!r} {az!r}')
    for _ in range(r.choice([0, 1, 1, 2, 3])):
        kind = r.random()
        if kind < 0.55:
            d = [0xF1, 0 if r.random() < 0.9 else 1, r.randint(0, 255), r.randint(0, 255), 1, 4,
                 r.choice([r.randint(0, 130), 98, 55, 54, 56, 0, 100, 255]), r.randint(0, 130)]
            dlc = 8
        elif kind < 0.9:
            d = [0xC5, 0, r.randint(0, 255), r.randint(0, 255), 0, 0, 0, 0]
            dlc = 6
        else:
            d = [r.randint(0, 255) for _ in range(8)]
            dlc = r.choice([8, 8, 3])
        L.append(f"R {r.choice(['701', '701', '1', '7FF'])} {dlc} {bytes(d).hex()}")
    if r.random() < 0.01:
        L.append('FS')
    if r.random() < 0.005:
        L.append('FR')
    if r.random() < 0.01:
        L.append('RO ' + r.choice(['ok', 'fail']))
    L.append(f'T {t}')
if nan:
    ticks = [i for i, x in enumerate(L) if x.startswith('T ')]
    idx = ticks[len(ticks) // 2]
    tt = int(L[idx].split()[1])
    L.insert(idx, f'C {tt} nan 0.1')
print('\n'.join(L))
