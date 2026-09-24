#!/usr/bin/env python3
"""초음파 레지스터 정지 시험 도구 (2026-09-24).

핸들 드라이버(user_guidance_driver_node)를 **끈 상태에서** 시리얼을 직접 연다.
하향 시험 명령으로 노이즈 저감 레벨(0x06)·바퀴 옆 지향각(0x07)을 바꾸고, 센서가
되읽어 준 설정(AA 59)을 확인한 뒤, 정해진 시간 동안 거리(AA 57)와 결과 통계(AA 58)를
모아 채널별 요약을 출력·저장한다. 시험 값은 보드 재부팅(드라이버 재기동 포함)하면
부팅 기본값(지향각 3·노이즈 1)으로 돌아간다.

    # 설정만 바꾸고 되읽기 확인
    python3 usonic_register_bench.py --noise 3
    python3 usonic_register_bench.py --side-angle 4
    python3 usonic_register_bench.py --reset
    # 60초 측정(장면 이름을 붙여 저장)
    python3 usonic_register_bench.py --seconds 60 --label side_wall_30cm
    # 설정 + 측정 한 번에
    python3 usonic_register_bench.py --side-angle 4 --seconds 60 --label floor_empty_a4

결과: ~/vica_data/usonic_bench/<시각>_<label>.json
"""
import argparse
import json
import statistics
import sys
import time
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vica_user_guidance import protocol  # noqa: E402
from vica_user_guidance.ultrasonic_frame import FrameAccumulator  # noqa: E402
from vica_user_guidance.ultrasonic_stats import (  # noqa: E402
    ConfigFrameAccumulator,
    StatFrameAccumulator,
    format_config_line,
)

NAMES = ["left_wheel", "front_left", "front_right", "right_wheel",
         "right_rear_wheel", "rear_right", "rear_left", "left_rear_wheel"]
PORT = "/dev/vica_smart_handle"


def wait_config(ser, cfg_acc, timeout=3.0):
    t = time.time()
    while time.time() - t < timeout:
        for f in cfg_acc.feed(ser.read(256)):
            return f
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--noise", type=int, choices=range(1, 6))
    ap.add_argument("--side-angle", type=int, choices=range(1, 5))
    ap.add_argument("--reset", action="store_true")
    ap.add_argument("--seconds", type=float, default=0.0)
    ap.add_argument("--label", default="bench")
    ap.add_argument("--port", default=PORT)
    a = ap.parse_args()

    ser = serial.Serial(a.port, protocol.FIRMWARE_BAUDRATE, timeout=0.05)
    time.sleep(0.3)
    dist_acc, stat_acc, cfg_acc = FrameAccumulator(), StatFrameAccumulator(), ConfigFrameAccumulator()

    cmds = []
    if a.reset:
        cmds.append(protocol.US_CMD_RESET)
    if a.noise:
        cmds.append(protocol.US_CMD_NOISE_BASE + a.noise)
    if a.side_angle:
        cmds.append(protocol.US_CMD_SIDE_ANGLE_BASE + a.side_angle)
    cfg_seen = None
    for c in cmds:
        ser.write(bytes([c]))
        ser.flush()
        cfg_seen = wait_config(ser, cfg_acc)
        print(f"명령 0x{c:02X} →", format_config_line(cfg_seen, NAMES) if cfg_seen else "되읽기 프레임 없음(3 s)")

    if a.seconds <= 0:
        ser.close()
        return

    per = {n: [] for n in NAMES}
    stat_sum = {n: [0] * protocol.US_STAT_KINDS for n in NAMES}
    t0 = time.time()
    last_print = 0.0
    while time.time() - t0 < a.seconds:
        data = ser.read(512)
        for f in dist_acc.feed(data):
            for ch, mm in enumerate(f.distances_mm):
                per[NAMES[ch]].append(mm)
            if time.time() - last_print > 0.5:
                last_print = time.time()
                row = " ".join(
                    f"{n[:7]:>7s}:{'--' if mm == 0 else ('max' if mm == protocol.US_CLEAR_MM else f'{mm/10:.0f}cm'):>5s}"
                    for n, mm in zip(NAMES, f.distances_mm))
                print(f"{time.time()-t0:5.1f}s {row}", flush=True)
        for s in stat_acc.feed(data):
            for ch, row in enumerate(s.counts):
                for k, v in enumerate(row):
                    stat_sum[NAMES[ch]][k] += v
        for c in cfg_acc.feed(data):
            cfg_seen = c
    ser.close()

    out = {"label": a.label, "seconds": a.seconds, "time": time.strftime("%F %T"),
           "config": cfg_seen.levels if cfg_seen else None, "channels": {}}
    print(f"\n== {a.label} ({a.seconds:.0f} s) ==  거리 cm: 에코 수·평균·표준편차·최소 / 에코없음 % / 무효 %"
          " / 통계 ok·clr·ffff·fffe·oth·i2c")
    for n in NAMES:
        v = per[n]
        echo = [mm / 10 for mm in v if 1 <= mm <= protocol.US_DIST_MAX_MM]
        clr = sum(1 for mm in v if mm == protocol.US_CLEAR_MM)
        inv = sum(1 for mm in v if mm == 0)
        tot = max(len(v), 1)
        st = stat_sum[n]
        summ = {
            "n": len(v), "echo": len(echo),
            "mean_cm": round(statistics.mean(echo), 1) if echo else None,
            "std_cm": round(statistics.pstdev(echo), 1) if len(echo) > 1 else None,
            "min_cm": round(min(echo), 1) if echo else None,
            "clear_pct": round(100 * clr / tot, 1), "invalid_pct": round(100 * inv / tot, 1),
            "stats": dict(zip(protocol.US_STAT_KIND_NAMES, st)),
        }
        out["channels"][n] = summ
        print(f"{n:17s} {summ['echo']:4d}·{summ['mean_cm']}·{summ['std_cm']}·{summ['min_cm']}"
              f" / {summ['clear_pct']:5.1f}% / {summ['invalid_pct']:4.1f}% / " + "·".join(map(str, st)))
    d = Path.home() / "vica_data" / "usonic_bench"
    d.mkdir(parents=True, exist_ok=True)
    p = d / f"{time.strftime('%m%d_%H%M%S')}_{a.label}.json"
    p.write_text(json.dumps(out, ensure_ascii=False, indent=1))
    print("저장", p)


if __name__ == "__main__":
    main()
