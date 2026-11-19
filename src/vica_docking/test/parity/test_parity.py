"""C++ V자 인식(dock_replay vdet)이 실기 도구 v2.1 의 파이썬 판(reference_vdetector.py)과 같은 답을 내는지.

표본: 11-18 실기 접근 run02~09 와 해제 undock05·06 의 녹화 스캔 89개(fixtures/field_1118_sample.npz).
녹화 전체(37개 파일·6,931 스캔)로도 같은 대조를 해서 최대 차이 5e-9 m 였다(11-18, 저장소 밖 도구).
"""
import os
import struct
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import reference_vdetector as ref  # noqa: E402


def _blob(t, R, amin, inc, rmin):
    b = bytearray(b'VDR1') + struct.pack('<ii3d', len(R), R.shape[1], amin, inc, rmin)
    for ti, r in zip(t, R):
        b += struct.pack('<d', float(ti)) + np.asarray(r, dtype='<f4').tobytes()
    return bytes(b)


def test_vdetector_matches_reference_on_field_scans():
    d = np.load(HERE / 'fixtures' / 'field_1118_sample.npz')
    R, t = d['ranges'], d['t']
    amin, inc, rmin = float(d['amin']), float(d['inc']), float(d['rmin'])
    out = subprocess.run(
        [os.environ['DOCK_REPLAY'], 'vdet'], input=_blob(t, R, amin, inc, rmin),
        capture_output=True, check=True).stdout.decode().split('\n')
    det = ref.VDetector()
    found = 0
    for i, r in enumerate(R):
        p = det(np.asarray(r, dtype=np.float32), amin, inc, rmin)
        c = out[i].split()
        assert (p is None) == (c[1] == '0'), (i, str(d['src'][i]))
        if p is None:
            continue
        found += 1
        x, y, th = ref.pose_in_dock(p)
        assert abs(x - float(c[2])) < 1e-6
        assert abs(y - float(c[3])) < 1e-6
        assert abs(th - float(c[4])) < 1e-6
        assert abs(p['inner'] - float(c[5])) < 1e-4
        assert (p['na'], p['nb']) == (int(c[8]), int(c[9]))
    assert found >= 60
