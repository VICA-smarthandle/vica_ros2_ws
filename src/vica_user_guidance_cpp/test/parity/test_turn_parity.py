"""방향지시 C++ 판(turn_guide_node)이 파이썬 원본과 글자까지 같은지 대조한다."""
import json
import os
import pathlib
import subprocess
import sys

import pytest
import yaml

HERE = pathlib.Path(__file__).resolve().parent
PY_SRC = pathlib.Path(os.environ.get('GUIDANCE_PY_SRC', HERE.parents[2] / 'vica_user_guidance'))
LAW_CPP = os.environ.get('TURN_LAW_CPP', '')

SEEDS = range(30)
TICKS = 2000


def _env():
    env = dict(os.environ, GUIDANCE_PY_SRC=str(PY_SRC), PYTHONDONTWRITEBYTECODE='1')
    env['ROS_DOMAIN_ID'] = '231'
    env['ROS_LOCALHOST_ONLY'] = '1'
    return env


def _run(cmd, stdin_text=None):
    return subprocess.run(cmd, input=stdin_text, capture_output=True, text=True, check=True,
                          env=_env(), timeout=600).stdout


def base_params():
    with open(PY_SRC / 'config' / 'user_guidance.yaml', encoding='utf-8') as f:
        return yaml.safe_load(f)['turn_guide_node']['ros__parameters']


def scenario(seed, ticks, overrides=None, options=None):
    params = base_params()
    params.update(overrides or {})
    return _run([sys.executable, str(HERE / 'turn_gen.py'), str(seed), str(ticks),
                 json.dumps(params, ensure_ascii=False), json.dumps(options or {})])


def both(text):
    py = _run([sys.executable, str(HERE / 'turn_law_py.py')], text)
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
    assert LAW_CPP and os.access(LAW_CPP, os.X_OK), 'TURN_LAW_CPP 가 없습니다'
    assert (PY_SRC / 'vica_user_guidance' / 'turn_guide_node.py').is_file(), \
        f'파이썬 원본이 없습니다: {PY_SRC}'


def count(out, prefix):
    return sum(1 for line in out.splitlines() if line.startswith(prefix))


REASONS = ['idle', 'stale', 'prepare', 'prepare_hold', 'prepare_chain', 'prepare_passed',
           'prepare_off_rail', 'now', 'now_new', 'now_inherit', 'now_hold', 'complete']


def test_default_config_matches_python_line_by_line():
    total = {}
    for seed in SEEDS:
        py, cpp = both(scenario(seed, TICKS))
        assert py == cpp, f'seed {seed}: {first_difference(py, cpp)}'
        for r in REASONS:
            total[r] = total.get(r, 0) + sum(1 for ln in py.splitlines() if ln == f'RS {r}')
        for key in ('G ', 'L W 레일 예고: TF 조회 실패 — 예고를 쉰다 (Lookup would require extrapolation',
                    'L W 레일 예고: TF 조회 실패 — 예고를 쉰다 ("map" passed to lookupTransform', 'X '):
            total[key] = total.get(key, 0) + count(py, key)
    for key, n in total.items():
        if key == 'X ':
            continue
        assert n > 0, (key, total)
    assert total['X '] == 0, total


NAN = float('nan')
INF = float('inf')
PARAM_CASES = [
    {'enable_rail_forecast': False},
    {'enable_rail_forecast': False, 'window_sec': 1.5, 'enter_threshold_deg': 20.0},
    {'rail_behind_start_m': 0.0},
    {'rail_min_prepare_dist_m': 0.0},
    {'rail_rehold_end_m': 0.0},
    {'rail_min_prepare_dist_m': 0.0, 'rail_rehold_end_m': 0.0, 'rail_behind_start_m': 0.0},
    {'controller_lookahead_time_sec': 2.5, 'controller_lookahead_min_m': 0.6,
     'controller_lookahead_max_m': 1.2},
    {'minor_turn_deg': 0.0},
    {'minor_turn_deg': 35.0, 'window_sec': 1.5, 'enter_threshold_deg': 20.0},
    {'enter_threshold_deg': 10.0, 'exit_threshold_deg': 15.0},
    {'min_duration_sec': 0.0},
    {'window_sec': 0.0},
    {'window_sec': 1e10},
    {'window_sec': -1.0},
    {'odom_timeout_sec': 0.0},
    {'odom_timeout_sec': -1.0},
    {'odom_timeout_sec': 1e12},
    {'rail_timeout_sec': 0.0},
    {'rail_timeout_sec': 1e10},
    {'rail_corner_threshold_deg': 10.0},
    {'rail_corner_threshold_deg': 170.0},
    {'rail_corner_threshold_deg': NAN},
    {'rail_lead_time_sec': 5.0},
    {'rail_lead_time_sec': 0.0},
    {'rail_lead_time_sec': INF},
    {'rail_max_offtrack_m': 0.3},
    {'rail_max_offtrack_m': NAN},
    {'rail_handoff_m': 0.0},
    {'rail_handoff_m': -INF},
    {'rail_merge_gap_m': 0.0},
    {'rail_merge_gap_m': 5.0},
    {'rail_goal_change_m': 0.0},
    {'rail_goal_change_m': 1e9},
    {'rail_behind_start_m': INF},
    {'enter_threshold_deg': NAN},
    {'exit_threshold_deg': NAN},
    {'minor_turn_min_speed_mps': NAN},
    {'controller_lookahead_time_sec': NAN},
    {'map_frame': 'odom'},
    {'base_frame': 'base_link'},
    {'odom_topic': '/wheel/odom', 'rail_topic': '/rail_plan2'},
    {'publish_rate_hz': INF},
    {'cue_valid_sec': 0.0},
    {'cue_valid_sec': -1.0},
    {'cue_valid_sec': 1e10},
    {'cue_valid_sec': 5e9},
    {'cue_valid_sec': -2e9},
    {'cue_valid_sec': NAN},
    {'cue_valid_sec': INF},
    {'publish_rate_hz': 0.0},
    {'publish_rate_hz': -1.0},
    {'publish_rate_hz': NAN},
    {'window_sec': NAN},
    {'min_duration_sec': INF},
    {'odom_timeout_sec': NAN},
    {'rail_timeout_sec': -INF},
]


@pytest.mark.parametrize('overrides', PARAM_CASES, ids=lambda o: json.dumps(o)[:80])
def test_other_configs_match_python(overrides):
    for seed in (7, 8):
        py, cpp = both(scenario(seed, 600, overrides))
        assert py == cpp, f'[{overrides}] seed {seed}: {first_difference(py, cpp)}'


CLOCK_CASES = [
    ({}, {'r0': 0}),
    ({'cue_valid_sec': -1.0}, {'r0': 0}),
    ({}, {'r0': (2 ** 31 - 1) * 1_000_000_000 - 5_000_000_000}),
    ({}, {'r0': 2 ** 63 - 3_000_000_000}),
]


@pytest.mark.parametrize('case', CLOCK_CASES, ids=lambda c: json.dumps(c)[:80])
def test_ros_clock_edges_match_python(case):
    overrides, options = case
    py, cpp = both(scenario(5, 400, overrides, options))
    assert py == cpp, f'[{case}] {first_difference(py, cpp)}'


def test_startup_and_publish_errors_are_the_python_errors():
    """설정 실수로 멈추는 경우 — 파이썬 판과 같은 예외·같은 글자인지 본다(위 표의 일부)."""
    expect = {
        'ZeroDivisionError: float division by zero': {'publish_rate_hz': 0.0},
        'ValueError: cannot convert float NaN to integer': {'window_sec': NAN},
        'OverflowError: cannot convert float infinity to integer': {'min_duration_sec': INF},
        'RCLError: failed to create timer: timer period must be non-negative, at ./src/rcl/timer.c:139':
            {'publish_rate_hz': -1.0},
        'OverflowError: Total nanoseconds value is too large to store in C duration.':
            {'cue_valid_sec': 1e10},
        "AssertionError: The 'sec' field must be an integer in [-2147483648, 2147483647]":
            {'cue_valid_sec': 5e9},
        'ValueError: Nanoseconds value must not be negative': {'cue_valid_sec': -2e9},
    }
    for line, overrides in expect.items():
        py, cpp = both(scenario(3, 40, overrides))
        assert py == cpp, first_difference(py, cpp)
        assert ('X ' + line) in py.splitlines(), (line, py[-500:])


BAGS = [b for b in os.environ.get('TURN_BAGS', '').split(':') if b]


@pytest.mark.skipif(not BAGS, reason='TURN_BAGS 가 없으면 녹화 재생은 건너뛴다')
@pytest.mark.parametrize('bag', BAGS)
def test_recorded_drive_matches_python(bag):
    params = base_params()
    text = _run([sys.executable, str(HERE / 'turn_bag_scenario.py'), bag,
                 json.dumps(params, ensure_ascii=False)])
    py, cpp = both(text)
    assert py == cpp, f'{bag}: {first_difference(py, cpp)}'
    assert count(py, 'G ') > 1000
