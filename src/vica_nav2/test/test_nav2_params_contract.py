from pathlib import Path

import pytest
import yaml


SAFETY_SUPERVISOR_CMD_TIMEOUT_SEC = 0.5


def _load_params():
    config_path = (
        Path(__file__).parents[1] / 'config' / 'nav2_params.yaml'
    )
    return yaml.safe_load(config_path.read_text(encoding='utf-8'))


def _controller_limits(params):
    """컨트롤러 종류와 무관하게 속도·가속 한계를 같은 이름으로 돌려준다."""
    controller = params['controller_server']['ros__parameters']
    fp = controller['FollowPath']
    if 'vcc' in fp['plugin'].lower():
        gc = controller['general_goal_checker']
        return {
            'plugin_family': 'vcc',
            'max_vel_x': fp['desired_linear_vel'],
            'min_vel_x': 0.0,
            'max_vel_theta': fp['max_angular_vel'],
            'acc_lim_x': fp['start_ramp_accel'], 'decel_lim_x': -fp['max_linear_decel'],
            'acc_lim_theta': fp['max_angular_accel'], 'decel_lim_theta': -fp['max_angular_accel'],
            'xy_goal_tolerance': gc['xy_goal_tolerance'],
            'trans_stopped_velocity': gc['trans_stopped_velocity'],
        }
    if 'purepursuit' in fp['plugin'].lower().replace('_', ''):
        gc = controller['general_goal_checker']
        sm = params['velocity_smoother']['ros__parameters']
        return {
            'plugin_family': 'rpp',
            'max_vel_x': fp['desired_linear_vel'],
            'min_vel_x': 0.0 if not fp.get('allow_reversing', False) else -fp['desired_linear_vel'],
            'max_vel_theta': sm['max_velocity'][2],
            'acc_lim_x': sm['max_accel'][0], 'decel_lim_x': sm['max_decel'][0],
            'acc_lim_theta': sm['max_accel'][2], 'decel_lim_theta': sm['max_decel'][2],
            'xy_goal_tolerance': gc['xy_goal_tolerance'],
            'trans_stopped_velocity': gc['trans_stopped_velocity'],
        }
    if 'mppi' in fp['plugin'].lower():
        gc = controller['general_goal_checker']
        return {
            'plugin_family': 'mppi',
            'max_vel_x': fp['vx_max'], 'min_vel_x': fp['vx_min'],
            'max_vel_theta': fp['wz_max'],
            'acc_lim_x': fp['ax_max'], 'decel_lim_x': fp['ax_min'],
            'acc_lim_theta': fp['az_max'], 'decel_lim_theta': -fp['az_max'],
            'xy_goal_tolerance': gc['xy_goal_tolerance'],
            'trans_stopped_velocity': gc['trans_stopped_velocity'],
        }
    return {
        'plugin_family': 'dwb',
        'max_vel_x': fp['max_vel_x'], 'min_vel_x': fp['min_vel_x'],
        'max_vel_theta': fp['max_vel_theta'],
        'acc_lim_x': fp['acc_lim_x'], 'decel_lim_x': fp['decel_lim_x'],
        'acc_lim_theta': fp['acc_lim_theta'], 'decel_lim_theta': fp['decel_lim_theta'],
        'xy_goal_tolerance': fp['xy_goal_tolerance'],
        'trans_stopped_velocity': fp['trans_stopped_velocity'],
    }


def test_goal_is_reached_only_after_the_robot_slows_to_a_stop():
    params = _load_params()
    controller = params['controller_server']['ros__parameters']
    goal_checker = controller['general_goal_checker']
    lim = _controller_limits(params)

    assert goal_checker['plugin'] in (
        'nav2_controller::StoppedGoalChecker',
        'vica_vcc_controller::LatchedGoalChecker',
    )
    if goal_checker['plugin'] == 'nav2_controller::StoppedGoalChecker':
        assert goal_checker['stateful'] is False
    else:
        fp = controller['FollowPath']
        assert goal_checker['unlatch_distance'] == fp['align_exit_dist']
        assert goal_checker['xy_goal_tolerance'] < goal_checker['unlatch_distance']
        assert 0.0 < fp['arrive_margin'] < goal_checker['xy_goal_tolerance'] / 2
        assert fp['align_rearm_time'] < controller['failure_tolerance']
    assert goal_checker['trans_stopped_velocity'] == 0.03
    assert goal_checker['rot_stopped_velocity'] == 0.05
    assert lim['trans_stopped_velocity'] == goal_checker['trans_stopped_velocity']
    assert lim['xy_goal_tolerance'] == goal_checker['xy_goal_tolerance']


def test_dwb_deceleration_limit_keeps_full_emergency_braking_power():
    """DWB decel_lim은 장애물 회피 trajectory rollout과 controller 정지 시"""
    params = _load_params()
    lim = _controller_limits(params)

    assert abs(lim['decel_lim_x']) >= 1.0
    assert abs(lim['decel_lim_theta']) >= 1.0


def test_velocity_smoother_arrival_softening_stays_within_goal_tolerance():
    """velocity_smoother.max_decel만 도착 시 완만함을 담당한다(decel_lim_x는"""
    params = _load_params()
    controller = params['controller_server']['ros__parameters']
    lim = _controller_limits(params)
    progress_checker = controller['progress_checker']
    smoother = params['velocity_smoother']['ros__parameters']

    max_vel_x = smoother['max_velocity'][0]
    max_decel_x = abs(smoother['max_decel'][0])

    stop_time = max_vel_x / max_decel_x
    assert stop_time < progress_checker['movement_time_allowance']

    if lim['plugin_family'] != 'vcc':
        stop_distance = max_vel_x ** 2 / (2.0 * max_decel_x)
        assert stop_distance < lim['xy_goal_tolerance']
        return

    fp = controller['FollowPath']
    gc = controller['general_goal_checker']
    stop_r = gc['xy_goal_tolerance'] - fp['arrive_margin']
    v_at = max(
        fp['desired_linear_vel'] * stop_r / fp['approach_velocity_scaling_dist'],
        fp['min_approach_linear_velocity'])
    decel = min(max_decel_x, fp['max_linear_decel'], fp.get('planned_linear_decel', fp['max_linear_decel']))
    latency = 0.3
    vcc_stop = v_at ** 2 / (2.0 * decel) + v_at * latency
    assert vcc_stop < lim['xy_goal_tolerance'], (
        f'멈춤 반경 {stop_r:.2f} m 에서 {v_at:.3f} m/s 로 들어오면 {vcc_stop:.3f} m 더 간다 — '
        f'허용오차 {lim["xy_goal_tolerance"]} 를 넘는다')


def test_velocity_smoother_timeout_does_not_widen_safety_detection_gap():
    """safety_supervisor_node.cmd_is_alive()는 마지막 명령 수신 이후 경과"""
    params = _load_params()
    smoother = params['velocity_smoother']['ros__parameters']

    assert (
        smoother['velocity_timeout']
        <= SAFETY_SUPERVISOR_CMD_TIMEOUT_SEC
    )


def test_smoother_lets_dwb_stop_rotating_as_fast_as_it_plans_to():
    """DWB의 회전 감속 가정과 velocity_smoother의 실제 허용치가 어긋나면 안 된다."""
    params = _load_params()
    lim = _controller_limits(params)
    smoother = params['velocity_smoother']['ros__parameters']

    dwb_yaw_decel = abs(lim['decel_lim_theta'])
    smoother_yaw_decel = abs(smoother['max_decel'][2])

    assert smoother_yaw_decel >= dwb_yaw_decel, (
        f'smoother 회전 감속 {smoother_yaw_decel}이 '
        f'DWB decel_lim_theta {dwb_yaw_decel}보다 약하다 — 조향 지연 발생'
    )


def test_smoother_does_not_throttle_dwb_rotational_acceleration():
    """회전 가속도 방향도 마찬가지로 smoother가 DWB보다 약하면 안 된다."""
    params = _load_params()
    lim = _controller_limits(params)
    smoother = params['velocity_smoother']['ros__parameters']

    assert smoother['max_accel'][2] >= lim['acc_lim_theta'], (
        f"smoother 회전 가속 {smoother['max_accel'][2]}이 "
        f"controller 회전 가속 {lim['acc_lim_theta']}보다 약하다"
    )


def test_smoother_lets_dwb_stop_moving_as_fast_as_it_plans_to():
    """직선 감속도 DWB 가정과 정합해야 한다."""
    params = _load_params()
    lim = _controller_limits(params)
    smoother = params['velocity_smoother']['ros__parameters']

    dwb_x_decel = abs(lim['decel_lim_x'])
    smoother_x_decel = abs(smoother['max_decel'][0])
    max_vel_x = lim['max_vel_x']
    padding = params['local_costmap']['local_costmap']['ros__parameters'][
        'footprint_padding'
    ]

    planned = max_vel_x ** 2 / (2 * dwb_x_decel)
    actual = max_vel_x ** 2 / (2 * smoother_x_decel)
    extra = actual - planned

    assert extra <= padding, (
        f'smoother 직선 감속 {smoother_x_decel}이 DWB {dwb_x_decel}보다 약해'
        f' 정지거리가 {extra * 100:.1f} cm 더 길어지는데'
        f' footprint_padding {padding * 100:.0f} cm를 넘는다.'
        ' DWB가 계획한 정지선을 실제로 넘어서므로 감속을 되돌리거나'
        ' padding을 늘린다'
    )


def test_stopping_distance_is_documented_against_padding():
    """정지거리가 padding을 넘는다는 사실을 수치로 고정한다."""
    params = _load_params()
    lim = _controller_limits(params)
    smoother = params['velocity_smoother']['ros__parameters']
    local = params['local_costmap']['local_costmap']['ros__parameters']

    v = lim['max_vel_x']
    decel = abs(smoother['max_decel'][0])
    driver_delay_sec = 0.3

    stopping_distance = v * driver_delay_sec + v ** 2 / (2 * decel)
    padding = local['footprint_padding']

    assert stopping_distance > padding, (
        f'정지거리 {stopping_distance:.3f} m가 padding {padding} m 이하로'
        f' 계산됐다. 실측 지연이 줄었다면 driver_delay_sec를 갱신하라'
    )


def test_vcc_limits_match_the_smoother():
    """VCC 출력단 한계가 velocity_smoother 와 어긋나면 한쪽이 다른 쪽 계획을 깎는다."""
    params = _load_params()
    fp = params['controller_server']['ros__parameters']['FollowPath']
    if 'vcc' not in fp['plugin'].lower():
        pytest.skip('VCC 가 활성 컨트롤러가 아니다')
    sm = params['velocity_smoother']['ros__parameters']
    assert fp['max_angular_vel'] <= sm['max_velocity'][2]
    assert sm['min_velocity'][2] == -sm['max_velocity'][2]
    assert fp['max_linear_decel'] == abs(sm['max_decel'][0])
    planned = fp.get('planned_linear_decel', fp['max_linear_decel'])
    assert fp['curve_decel'] <= planned <= fp['max_linear_decel']
    assert fp['desired_linear_vel'] <= sm['max_velocity'][0]
    assert fp['lane_rate'] < 0.115


def test_rpp_is_preserved_for_one_line_rollback():
    params = _load_params()
    controller = params['controller_server']['ros__parameters']
    assert 'FollowPathRPP' in controller
    assert 'purepursuit' in controller['FollowPathRPP']['plugin'].lower().replace('_', '')
    assert controller['controller_plugins'] == ['FollowPath']
