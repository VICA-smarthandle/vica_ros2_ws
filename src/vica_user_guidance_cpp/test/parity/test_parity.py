"""C++ 판이 파이썬 원본과 글자까지 같은지 대조한다."""
import json
import math
import os
import pathlib
import subprocess
import sys

import pytest
import yaml

HERE = pathlib.Path(__file__).resolve().parent
PY_SRC = pathlib.Path(os.environ.get('GUIDANCE_PY_SRC', HERE.parents[2] / 'vica_user_guidance'))
LAW_CPP = os.environ.get('LAW_CPP', '')

SEEDS = range(30)
CYCLES = 2000


def _env():
    env = dict(os.environ, GUIDANCE_PY_SRC=str(PY_SRC), PYTHONDONTWRITEBYTECODE='1')
    env['ROS_DOMAIN_ID'] = '231'
    env['ROS_LOCALHOST_ONLY'] = '1'
    return env


def _run(cmd, stdin_text=None):
    return subprocess.run(cmd, input=stdin_text, capture_output=True, text=True, check=True,
                          env=_env(), timeout=300).stdout


def base_params():
    with open(PY_SRC / 'config' / 'user_guidance.yaml', encoding='utf-8') as f:
        return yaml.safe_load(f)['user_guidance_driver_node']['ros__parameters']


def scenario(seed, cycles, overrides=None):
    params = base_params()
    params.update(overrides or {})
    return _run([sys.executable, str(HERE / 'gen.py'), str(seed), str(cycles),
                 json.dumps(params, ensure_ascii=False)])


def both(text):
    py = _run([sys.executable, str(HERE / 'law_py.py')], text)
    cpp = _run([LAW_CPP], text)
    return py, cpp


def first_difference(a, b):
    la, lb = a.splitlines(), b.splitlines()
    for i, (x, y) in enumerate(zip(la, lb)):
        if x != y:
            return f'line {i + 1}:\n  py : {x[:300]}\n  cpp: {y[:300]}'
    return f'length differs: py {len(la)} lines, cpp {len(lb)} lines'


@pytest.fixture(scope='module', autouse=True)
def _need_binaries():
    assert LAW_CPP and os.access(LAW_CPP, os.X_OK), 'LAW_CPP 가 없습니다'
    assert (PY_SRC / 'vica_user_guidance' / 'user_guidance_driver_node.py').is_file(), \
        f'파이썬 원본이 없습니다: {PY_SRC}'


def count(out, prefix):
    return sum(1 for line in out.splitlines() if line.startswith(prefix))


def test_default_config_matches_python_line_by_line():
    total = {}
    for seed in SEEDS:
        py, cpp = both(scenario(seed, CYCLES))
        assert py == cpp, f'seed {seed}: {first_difference(py, cpp)}'
        for key in ('W 0', 'W 1', 'PR ', 'PS ', 'Q ', 'OPEN ', 'OPENFAIL', 'CLOSE', 'WFAIL',
                    'FFAIL', 'IFAIL', 'RFAIL', 'LT ', 'L I [US 통계', 'L I [US 설정',
                    'L W [초음파]', 'L I [초음파]', 'L I [HAPTIC]', 'L W [HAPTIC] 모르는',
                    'L W [HAPTIC] ', 'L I [HANDLE] ARRIVED', 'L I [HANDLE] LEFT',
                    'L I [HANDLE] RIGHT', 'L W [HANDLE] ESTOP(3) reason=estop_stale',
                    'L W 초음파 프레임 수신 끊김', 'X '):
            total[key] = total.get(key, 0) + count(py, key)
    for key, n in total.items():
        if key == 'X ':
            continue
        assert n > 0, (key, total)
    assert total['X '] == 0, total


NAN = float('nan')
INF = float('inf')
PARAM_CASES = [
    {'ultrasonic_channel_enabled': [True] * 8},
    {'ultrasonic_channel_enabled': [False] * 8},
    {'ultrasonic_channel_enabled': [True, False] * 4},
    {'ultrasonic_tf_gate': False},
    {'ultrasonic_tf_target_frame': 'map'},
    {'touch_enabled': False},
    {'ultrasonic_enabled': False},
    {'ultrasonic_enabled': False, 'touch_enabled': False},
    {'estop_required': False},
    {'enable_serial': False},
    {'arrival_hold_sec': 3.0},
    {'arrival_hold_sec': 0.0},
    {'arrival_hold_sec': -1.0},
    {'arrival_hold_sec': 1e10},
    {'arrival_hold_sec': NAN},
    {'cue_timeout_sec': 0.0},
    {'cue_timeout_sec': 1e10, 'estop_timeout_sec': 1e10},
    {'cue_timeout_sec': -1.0},
    {'estop_timeout_sec': 0.0},
    {'reconnect_interval_sec': 0.0},
    {'reconnect_interval_sec': -1.0},
    {'reconnect_interval_sec': 1e12},
    {'reconnect_interval_sec': INF},
    {'ultrasonic_max_range_m': 0.3, 'ultrasonic_max_range_m_per_channel': [-1.0] * 8},
    {'ultrasonic_max_range_m_per_channel': [0.4, 3.5, 0.0, -0.0, NAN, 1e-9, 2.0, 0.301]},
    {'ultrasonic_fov_rad': 0.0},
    {'ultrasonic_fov_rad': 1.0, 'ultrasonic_fov_rad_per_channel': [-1.0] * 8},
    {'ultrasonic_fov_rad': 1e300, 'ultrasonic_fov_rad_per_channel': [-1.0] * 8},
    {'ultrasonic_fov_rad_per_channel': [0.0078539816, 0.02181661564, 2.5 * math.pi / 180] + [-1.0] * 5},
    {'ultrasonic_fov_rad_per_channel': [NAN] * 8},
    {'ultrasonic_fov_rad': NAN, 'ultrasonic_fov_rad_per_channel': [-1.0] * 8},
    {'ultrasonic_fov_rad': INF, 'ultrasonic_fov_rad_per_channel': [-1.0] * 8},
    {'ultrasonic_fov_rad_per_channel': [-1.0] * 3},
    {'ultrasonic_max_range_m_per_channel': [-1.0] * 9},
    {'ultrasonic_measurement_delay_ms': [0] * 8},
    {'ultrasonic_measurement_delay_ms': [-100] * 8},
    {'ultrasonic_measurement_delay_ms': [10 ** 10] * 8},
    {'ultrasonic_measurement_delay_ms': [2 ** 62] * 8},
    {'ultrasonic_measurement_delay_ms': [-(2 ** 62)] * 8},
    {'ultrasonic_measurement_delay_ms': [210] * 7},
    {'ultrasonic_topics': ['/ultrasonic/a'] * 7},
    {'ultrasonic_frame_ids': ['f'] * 9},
    {'ultrasonic_channel_enabled': [True, True]},
    {'ultrasonic_stale_warn_sec': 0.0},
    {'ultrasonic_stale_warn_sec': 0.2},
    {'ultrasonic_stale_warn_sec': NAN},
    {'touch_stale_sec': 0.0},
    {'touch_stale_sec': 5.0},
    {'touch_publish_min_interval_sec': 0.0},
    {'touch_publish_min_interval_sec': 10.0},
    {'touch_publish_min_interval_sec': -INF},
    {'uplink_rate_hz': 0.0},
    {'uplink_rate_hz': -1.0},
    {'uplink_rate_hz': INF},
    {'uplink_rate_hz': 0.0, 'ultrasonic_enabled': False, 'touch_enabled': False},
    {'send_rate_hz': 0.0},
    {'send_rate_hz': NAN},
    {'diag_rate_hz': -2.0},
    {'diag_rate_hz': -INF},
    {'ultrasonic_min_range_m': 5e38},
    {'ultrasonic_min_range_m': 3.4028234e38},
    {'ultrasonic_min_range_m': -INF},
    {'ultrasonic_max_range_m': 1e39, 'ultrasonic_max_range_m_per_channel': [-1.0] * 8},
    {'baudrate': 9600, 'write_timeout_sec': 0.0},
    {'serial_port': '/dev/ttyUSB7'},
    {'ultrasonic_frame_ids': ["it's", 'b"q', 'x y', 'f', 'f', 'g', 'h', '가']},
    {'ultrasonic_topics': ['/u/a', '/u/b', '/u/c', '/u/d', '/u/e', '/u/f', '/u/g', 'u_h']},
]


@pytest.mark.parametrize('overrides', PARAM_CASES, ids=lambda o: json.dumps(o)[:80])
def test_other_configs_match_python(overrides):
    for seed in (7, 8):
        py, cpp = both(scenario(seed, 600, overrides))
        assert py == cpp, f'[{overrides}] seed {seed}: {first_difference(py, cpp)}'


def test_startup_errors_are_the_python_errors():
    """설정 실수로 기동을 막는 경우 — 파이썬 판과 같은 예외·같은 글자인지 본다(위 표의 일부)."""
    expect = {
        'ValueError: ultrasonic_topics/frame_ids/measurement_delay_ms 는 모두 채널 수 8개여야 합니다: 7/8/8':
            {'ultrasonic_topics': ['/ultrasonic/a'] * 7},
        'ValueError: ultrasonic_fov_rad_per_channel 은 8칸이어야 합니다: 3':
            {'ultrasonic_fov_rad_per_channel': [-1.0] * 3},
        'ValueError: ultrasonic_channel_enabled 는 8칸이어야 합니다: 2':
            {'ultrasonic_channel_enabled': [True, True]},
        'ZeroDivisionError: float division by zero': {'send_rate_hz': 0.0},
        'ValueError: cannot convert float NaN to integer': {'ultrasonic_stale_warn_sec': NAN},
        'OverflowError: cannot convert float infinity to integer': {'reconnect_interval_sec': INF},
    }
    for line, overrides in expect.items():
        py, cpp = both(scenario(3, 20, overrides))
        assert py == cpp, first_difference(py, cpp)
        assert ('X ' + line) in py.splitlines(), (line, py[-500:])
