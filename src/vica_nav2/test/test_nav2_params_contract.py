from pathlib import Path

import yaml


SAFETY_SUPERVISOR_CMD_TIMEOUT_SEC = 0.5


def _load_params():
    config_path = (
        Path(__file__).parents[1] / 'config' / 'nav2_params.yaml'
    )
    return yaml.safe_load(config_path.read_text(encoding='utf-8'))


def test_goal_is_reached_only_after_the_robot_slows_to_a_stop():
    params = _load_params()
    controller = params['controller_server']['ros__parameters']
    goal_checker = controller['general_goal_checker']
    follow_path = controller['FollowPath']

    assert goal_checker['plugin'] == (
        'nav2_controller::StoppedGoalChecker'
    )
    assert goal_checker['stateful'] is False
    assert goal_checker['trans_stopped_velocity'] == 0.03
    assert goal_checker['rot_stopped_velocity'] == 0.05
    assert (
        follow_path['trans_stopped_velocity']
        == goal_checker['trans_stopped_velocity']
    )
    assert (
        follow_path['xy_goal_tolerance']
        == goal_checker['xy_goal_tolerance']
    )


def test_dwb_deceleration_limit_keeps_full_emergency_braking_power():
    """DWB decel_lim은 장애물 회피 trajectory rollout과 controller 정지 시"""
    params = _load_params()
    follow_path = params['controller_server']['ros__parameters']['FollowPath']

    assert abs(follow_path['decel_lim_x']) >= 1.0
    assert abs(follow_path['decel_lim_theta']) >= 1.0


def test_velocity_smoother_arrival_softening_stays_within_goal_tolerance():
    """velocity_smoother.max_decel만 도착 시 완만함을 담당한다(decel_lim_x는"""
    params = _load_params()
    controller = params['controller_server']['ros__parameters']
    follow_path = controller['FollowPath']
    progress_checker = controller['progress_checker']
    smoother = params['velocity_smoother']['ros__parameters']

    max_vel_x = smoother['max_velocity'][0]
    max_decel_x = abs(smoother['max_decel'][0])

    stop_time = max_vel_x / max_decel_x
    stop_distance = max_vel_x ** 2 / (2.0 * max_decel_x)

    assert stop_distance < follow_path['xy_goal_tolerance']
    assert stop_time < progress_checker['movement_time_allowance']


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
    follow_path = params['controller_server']['ros__parameters']['FollowPath']
    smoother = params['velocity_smoother']['ros__parameters']

    dwb_yaw_decel = abs(follow_path['decel_lim_theta'])
    smoother_yaw_decel = abs(smoother['max_decel'][2])

    assert smoother_yaw_decel >= dwb_yaw_decel, (
        f'smoother 회전 감속 {smoother_yaw_decel}이 '
        f'DWB decel_lim_theta {dwb_yaw_decel}보다 약하다 — 조향 지연 발생'
    )


def test_smoother_does_not_throttle_dwb_rotational_acceleration():
    """회전 가속도 방향도 마찬가지로 smoother가 DWB보다 약하면 안 된다."""
    params = _load_params()
    follow_path = params['controller_server']['ros__parameters']['FollowPath']
    smoother = params['velocity_smoother']['ros__parameters']

    assert smoother['max_accel'][2] >= follow_path['acc_lim_theta'], (
        f"smoother 회전 가속 {smoother['max_accel'][2]}이 "
        f"DWB acc_lim_theta {follow_path['acc_lim_theta']}보다 약하다"
    )


def test_smoother_lets_dwb_stop_moving_as_fast_as_it_plans_to():
    """직선 감속도 DWB 가정과 정합해야 한다."""
    params = _load_params()
    follow_path = params['controller_server']['ros__parameters']['FollowPath']
    smoother = params['velocity_smoother']['ros__parameters']

    dwb_x_decel = abs(follow_path['decel_lim_x'])
    smoother_x_decel = abs(smoother['max_decel'][0])
    max_vel_x = follow_path['max_vel_x']
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
    follow_path = params['controller_server']['ros__parameters']['FollowPath']
    smoother = params['velocity_smoother']['ros__parameters']
    local = params['local_costmap']['local_costmap']['ros__parameters']

    v = follow_path['max_vel_x']
    decel = abs(smoother['max_decel'][0])
    driver_delay_sec = 0.3

    stopping_distance = v * driver_delay_sec + v ** 2 / (2 * decel)
    padding = local['footprint_padding']

    assert stopping_distance > padding, (
        f'정지거리 {stopping_distance:.3f} m가 padding {padding} m 이하로'
        f' 계산됐다. 실측 지연이 줄었다면 driver_delay_sec를 갱신하라'
    )
