"""C++ 판이 파이썬 원본과 글자까지 같은지 대조한다."""
import os
import pathlib
import subprocess
import sys

import pytest

HERE = pathlib.Path(__file__).resolve().parent
PY_SRC = pathlib.Path(os.environ.get(
    'MOTOR_PY_SRC', HERE.parents[2] / 'mdrobot_can_control'))
LAW_CPP = os.environ.get('LAW_CPP', '')
PREFLIGHT_CPP = os.environ.get('PREFLIGHT_CPP', '')

SEEDS = range(30)
CYCLES = 2000
NAN_SEEDS = range(100, 105)


def _run(cmd, stdin_text=None, extra_env=None):
    env = dict(os.environ, MOTOR_PY_SRC=str(PY_SRC), PYTHONDONTWRITEBYTECODE='1')
    env.update(extra_env or {})
    return subprocess.run(
        cmd, input=stdin_text, capture_output=True, text=True, check=True, env=env,
        timeout=120).stdout


def _scenario(seed, cycles, nan=False):
    args = [sys.executable, str(HERE / 'gen.py'), str(seed), str(cycles)]
    if nan:
        args.append('nan')
    return _run(args)


def _both(scenario, params=''):
    extra = {'PARAMS': params}
    py = _run([sys.executable, str(HERE / 'law_py.py')], scenario, extra)
    cpp = _run([LAW_CPP], scenario, extra)
    return py, cpp


def _first_difference(a, b):
    la, lb = a.splitlines(), b.splitlines()
    for i, (x, y) in enumerate(zip(la, lb)):
        if x != y:
            return f'line {i + 1}:\n  py : {x}\n  cpp: {y}'
    return f'length differs: py {len(la)} lines, cpp {len(lb)} lines'


@pytest.fixture(scope='module', autouse=True)
def _need_binaries():
    assert LAW_CPP and os.access(LAW_CPP, os.X_OK), 'LAW_CPP 가 없습니다'
    assert PREFLIGHT_CPP and os.access(PREFLIGHT_CPP, os.X_OK), 'PREFLIGHT_CPP 가 없습니다'
    assert (PY_SRC / 'mdrobot_can_control').is_dir(), f'파이썬 원본이 없습니다: {PY_SRC}'


def test_control_loop_matches_python_line_by_line():
    sent = nonzero = faults = recovered = 0
    for seed in SEEDS:
        py, cpp = _both(_scenario(seed, CYCLES))
        assert py == cpp, f'seed {seed}: {_first_difference(py, cpp)}'
        for line in py.splitlines():
            data = line[2 + 16:] if line.startswith('S ') else ''
            if data.startswith('cf'):
                sent += 1
                nonzero += data[4:8] != '0000' or data[10:14] != '0000'
            faults += '[CAN FAULT]' in line
            recovered += '[CAN RECOVERED]' in line
    assert sent > 0 and nonzero / sent > 0.2, (sent, nonzero)
    assert faults > 0 and recovered > 0, (faults, recovered)


PARAM_CASES = [
    'invert_mot1=1', 'invert_mot2=1 invert_mot1=1', 'min_rpm_when_moving=30',
    'min_rpm_when_moving=500', 'max_rpm=40000 max_linear_mps=500 max_angular_radps=900',
    'max_rpm=50', 'max_rpm=0', 'max_rpm=-400', 'deadzone_pct=0', 'deadzone_pct=-1',
    'deadzone_pct=100', 'knob_min_pct=0 knob_max_pct=100', 'knob_min_pct=98 knob_max_pct=55',
    'knob_min_pct=50 knob_max_pct=50', 'knob_min_pct=-10 knob_max_pct=300',
    'knob_min_pct=33 knob_max_pct=36', 'wheel_base_m=0.5 wheel_radius_m=0.1',
    'wheel_radius_m=-0.065', 'wheel_radius_m=0.0', 'wheel_radius_m=-0.0', 'wheel_base_m=0.0',
    'max_linear_mps=inf', 'max_angular_radps=inf', 'max_linear_mps=nan', 'max_linear_mps=-1.0',
    'resend_interval_sec=0', 'resend_interval_sec=-1', 'resend_interval_sec=1e12',
    'cmd_timeout_sec=0', 'cmd_timeout_sec=1e10 knob_timeout_sec=1e10', 'cmd_timeout_sec=-1',
    'can_reconnect_interval_sec=0', 'can_reconnect_interval_sec=-1',
    'can_reconnect_interval_sec=1e10', 'driver_id=0x7FF', 'driver_id=0x123', 'driver_id=-1',
    'driver_id=0x100000000', 'driver_id=0xFFFFFFFF',
]


@pytest.mark.parametrize('params', PARAM_CASES)
def test_control_loop_matches_python_with_other_params(params):
    py, cpp = _both(_scenario(7, 1500), params)
    assert py == cpp, f'[{params}]: {_first_difference(py, cpp)}'


def test_zero_wheel_radius_stops_like_python():
    """바퀴 반지름 0 은 파이썬처럼 그 사이클에 죽어야 한다(C++ 기본 동작은 inf → 최고 rpm)."""
    py, cpp = _both(_scenario(7, 300), 'wheel_radius_m=0.0')
    assert py == cpp
    assert py.rstrip().splitlines()[-1] == 'X float division by zero'
    assert not any(line.startswith('S ') for line in cpp.splitlines())


def test_nan_command_kills_both_at_the_same_cycle():
    for seed in NAN_SEEDS:
        py, cpp = _both(_scenario(seed, 300, nan=True))
        assert py == cpp, f'seed {seed}: {_first_difference(py, cpp)}'
        assert py.rstrip().splitlines()[-1] == 'X cannot convert float NaN to integer'


FLAG_CASES = [
    '0x1\n', '0x0\n', '0x40c1\n', '0x1003\n', '4099\n', '0\n', '00\n', '010\n', '0x_1\n',
    '0x__1\n', '1_0\n', '_1\n', '1_\n', 'zz\n', '  0x3  \n', '-0x1\n', '+0x3\n', '0b11\n',
    '0o7\n', '0X1F\n', '', '\n', '0x1 0x2\n', "0x'1\n",
]
NAME_CASES = ['', '.', '..', 'a/b', 'can1/', "it's", 'nosuch0']


def _python_preflight(root, iface):
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(PY_SRC))
    from mdrobot_can_control.can_preflight import require_can_interface_up
    try:
        return f'OK 0x{require_can_interface_up(iface, sys_class_net=root):X}'
    except RuntimeError as exc:
        return f'ERR {exc}'


def _cpp_preflight(root, iface):
    return _run([PREFLIGHT_CPP, str(root), iface]).rstrip('\n')


@pytest.mark.parametrize('flags', FLAG_CASES)
def test_preflight_flags_match_python(tmp_path, flags):
    (tmp_path / 'can1').mkdir()
    (tmp_path / 'can1' / 'flags').write_text(flags, encoding='utf-8')
    assert _cpp_preflight(tmp_path, 'can1') == _python_preflight(tmp_path, 'can1')


@pytest.mark.parametrize('iface', NAME_CASES)
def test_preflight_names_match_python(tmp_path, iface):
    assert _cpp_preflight(tmp_path, iface) == _python_preflight(tmp_path, iface)


def test_preflight_unreadable_and_directory_match_python(tmp_path):
    (tmp_path / 'can1').mkdir()
    (tmp_path / 'can1' / 'flags').mkdir()
    assert _cpp_preflight(tmp_path, 'can1') == _python_preflight(tmp_path, 'can1')
    if os.geteuid() != 0:
        (tmp_path / 'can2').mkdir()
        p = tmp_path / 'can2' / 'flags'
        p.write_text('0x1\n', encoding='utf-8')
        p.chmod(0)
        try:
            assert _cpp_preflight(tmp_path, 'can2') == _python_preflight(tmp_path, 'can2')
        finally:
            p.chmod(0o644)
