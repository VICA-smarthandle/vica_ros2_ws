#!/usr/bin/env python3
"""Measure how long an initial-pose check takes, on whatever machine you run it.

**왜 필요한가.** 초기 위치 확인은 앱이 버튼을 누르고 사람이 기다리는 동작이다.
개발 PC 에서 빠르다고 젯슨에서도 빠른 것이 아니다. 젯슨은 같은 시간에 Nav2 ·
Cartographer · 카메라 · rosbridge 를 함께 돌린다. 2026-08-01 실측에서 전체 CPU
사용률이 346 % / 800 %, load average 최대 13.8 이었다. load 13.8 은 코어 8 개를
넘는 값이라 실행 대기 큐가 쌓였다는 뜻이다.

그래서 이 스크립트는 **양쪽에서 같은 것을 재도록** 만들었다. 젯슨에서 돌릴 때는
평소 주행하듯 스택을 다 띄운 상태에서 돌려야 의미가 있다. load average 도 같이
찍으므로 "그때 얼마나 바빴는지"가 결과에 남는다.

    python3 scripts/vica_pose_score_bench.py
    python3 scripts/vica_pose_score_bench.py --map maps/vica_map_0630.yaml --repeat 10

판정: 확인 1 회가 **1 초를 넘으면** 앱에 진행 표시가 필요하고, 3 초를 넘으면
탐색 격자를 줄여야 한다(거친 각도 5도 -> 10도 로 후보 절반).
"""

import argparse
import math
import multiprocessing
import os
import platform
import statistics
import sys
import time

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, 'src', 'vica_nav2'))

from vica_nav2.pose_score import (  # noqa: E402,I202
    build_likelihood_field,
    filter_beams,
    MapGrid,
    search_pose,
)


def load_map(yaml_path):
    """Read a map_server pgm/yaml pair into a MapGrid. 앱이 쓰는 실제 지도로 잰다."""
    meta = {}
    with open(yaml_path) as handle:
        for line in handle:
            if ':' not in line:
                continue
            key, _, value = line.partition(':')
            meta[key.strip()] = value.strip()

    image_path = os.path.join(os.path.dirname(yaml_path), meta['image'])
    with open(image_path, 'rb') as handle:
        assert handle.readline().strip() == b'P5', 'P5 (binary) pgm 만 읽는다'
        fields = []
        while len(fields) < 3:
            line = handle.readline()
            if line.startswith(b'#'):
                continue
            fields += line.split()
        width, height, _ = (int(v) for v in fields[:3])
        pixels = np.frombuffer(handle.read(width * height), dtype=np.uint8)

    # map_server trinary 규칙. pgm 은 위에서 아래로 저장되므로 뒤집어야 ROS 격자다.
    image = pixels.reshape(height, width)[::-1]
    occupancy = np.full(image.shape, -1, dtype=np.int8)
    occupied_ratio = (255.0 - image.astype(np.float64)) / 255.0
    occupancy[occupied_ratio > float(meta.get('occupied_thresh', 0.65))] = 100
    occupancy[occupied_ratio < float(meta.get('free_thresh', 0.25))] = 0

    origin = [float(v) for v in meta['origin'].strip('[]').split(',')]
    return MapGrid(occupancy, float(meta['resolution']), origin[0], origin[1])


def pick_open_spot(grid, count, tries=12):
    """Pick the free cell that yields the most usable beams -- the slowest case.

    계산 시간은 살아남은 빔 수에 비례한다. 빔이 적게 잡히는 구석에서 재면 실제보다
    빨라 보인다. 그래서 여러 자리를 찔러 보고 **가장 많이 잡히는 자리**로 잰다.
    """
    free = np.argwhere(grid.data == 0)
    best = None
    for i in range(tries):
        row, col = free[len(free) * (i + 1) // (tries + 1)]
        x = grid.origin_x + (col + 0.5) * grid.resolution
        y = grid.origin_y + (row + 0.5) * grid.resolution
        ranges, angle_min, increment = simulate_scan(grid, x, y, 0.0, count)
        beams = filter_beams(ranges, angle_min, increment)
        if best is None or beams.used > best[0].used:
            best = (beams, x, y)
    return best


def simulate_scan(grid, x, y, yaw, count, max_range=11.0):
    """Ray cast a scan the way the real lidar would see it from (x, y, yaw)."""
    step = grid.resolution
    ranges = np.full(count, np.inf)
    angle_min = -math.pi
    increment = 2.0 * math.pi / count
    for i in range(count):
        theta = yaw + angle_min + i * increment
        dx, dy = math.cos(theta) * step, math.sin(theta) * step
        cx, cy, travelled = x, y, 0.0
        while travelled <= max_range:
            col = int((cx - grid.origin_x) / grid.resolution)
            row = int((cy - grid.origin_y) / grid.resolution)
            if not (0 <= col < grid.width and 0 <= row < grid.height):
                break
            if grid.data[row, col] >= 65:
                ranges[i] = travelled
                break
            cx += dx
            cy += dy
            travelled += step
    return ranges, angle_min, increment


def _spin(stop):
    """Burn one core until told to stop. --busy 가 쓰는 부하 발생기다."""
    while not stop.is_set():
        pass


class Contention:
    """Hold N cores busy while the benchmark runs.

    젯슨은 이 확인을 혼자 돌리지 않는다. Nav2·Cartographer·카메라·rosbridge 가
    같은 CPU 를 쓴다. 2026-08-01 실측에서 load average 최대 13.8 이었다.
    개발 PC 에서 그 상황을 흉내 내야 "젯슨에서 얼마나 느려질까"를 미리 가늠할 수 있다.
    """

    def __init__(self, workers):
        """Remember how many spinners to start."""
        self.workers = workers
        self.stop = multiprocessing.Event()
        self.procs = []

    def __enter__(self):
        """Start the spinners and let the load average catch up."""
        for _ in range(self.workers):
            proc = multiprocessing.Process(target=_spin, args=(self.stop,), daemon=True)
            proc.start()
            self.procs.append(proc)
        if self.workers:
            time.sleep(1.0)   # load average 가 올라올 시간을 준다
        return self

    def __exit__(self, *exc):
        """Stop the spinners."""
        self.stop.set()
        for proc in self.procs:
            proc.join(timeout=2.0)


def timed(label, repeat, call):
    """Run call() repeat times and print min/median/max in milliseconds."""
    samples = []
    for _ in range(repeat):
        started = time.perf_counter()
        result = call()
        samples.append((time.perf_counter() - started) * 1000.0)
    print('  %-34s %7.1f %7.1f %7.1f ms' % (
        label, min(samples), statistics.median(samples), max(samples)))
    return result


def main():
    """Print machine facts, then time the field build and the two search modes."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', default=os.path.join(REPO_ROOT, 'maps', 'vica_map_0630.yaml'))
    parser.add_argument('--repeat', type=int, default=5)
    parser.add_argument('--beams', type=int, default=721, help='라이다 원본 빔 수')
    parser.add_argument('--busy', type=int, default=0,
                        help='이만큼의 코어를 일부러 바쁘게 만들고 잰다. 젯슨 경합 흉내')
    args = parser.parse_args()

    load = os.getloadavg()
    print('=' * 62)
    print('machine     %s %s' % (platform.node(), platform.machine()))
    print('cores       %d' % os.cpu_count())
    print('load avg    %.2f / %.2f / %.2f   <- 재는 동안 이 기계가 얼마나 바빴는가'
          % load)
    print('numpy       %s' % np.__version__)
    try:
        import scipy
        print('scipy       %s' % scipy.__version__)
    except ImportError:
        print('scipy       없음 -> chamfer 대비책으로 돈다')
    print('=' * 62)

    grid = load_map(args.map)
    print('map         %s  %dx%d = %d cells  res %.3f m'
          % (os.path.basename(args.map), grid.width, grid.height,
             grid.data.size, grid.resolution))

    beams, x, y = pick_open_spot(grid, args.beams)
    print('scan        원본 %d 빔 -> 필터·솎기 뒤 %d 빔 (상한 180)'
          % (beams.total, beams.used))
    print('pose        (%.2f, %.2f)  <- 빔이 가장 많이 잡히는 자리로 골랐다' % (x, y))
    print('-' * 62)
    print('  %-34s %7s %7s %7s' % ('', 'min', 'med', 'max'))

    with Contention(args.busy):
        if args.busy:
            print('  (코어 %d 개를 바쁘게 만든 상태. load avg %.2f)'
                  % (args.busy, os.getloadavg()[0]))
        field = timed('거리표 만들기 (노드 시작 시 1회)', args.repeat,
                      lambda: build_likelihood_field(grid))
        timed('확인 · 방향 지정 (후보 2299)', args.repeat,
              lambda: search_pose(field, grid, beams, x, y, yaw_hint=0.0))
        timed('확인 · 방향 모름 (후보 8712)', args.repeat,
              lambda: search_pose(field, grid, beams, x, y, yaw_hint=None))
    print('-' * 62)
    print('판정: 확인 1회가 1초를 넘으면 앱에 진행 표시가 필요하고,')
    print('      3초를 넘으면 COARSE_YAW_STEP 을 5도 -> 10도 로 올린다.')


if __name__ == '__main__':
    main()
