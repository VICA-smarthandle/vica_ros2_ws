from pathlib import Path

import yaml


# safety_supervisor_node.py의 cmd_timeout_sec 기본값 (declare_parameter default).
# launch에서 override되지 않으므로 실제 런타임 값과 동일하다.
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
    # stateful=False여야 감속 중 tolerance를 재이탈해도 속도 조건만으로
    # 성공 처리되지 않고 매 주기 XY/yaw를 재검사한다.
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
    """DWB decel_lim은 장애물 회피 trajectory rollout과 controller 정지 시
    제동에도 쓰이는 전역 값이다. 목적지 도착을 부드럽게 하려고 이 값을
    약화시키면 안 된다 — 완화는 velocity_smoother.max_decel에서만 한다."""
    params = _load_params()
    follow_path = params['controller_server']['ros__parameters']['FollowPath']

    assert abs(follow_path['decel_lim_x']) >= 1.0
    assert abs(follow_path['decel_lim_theta']) >= 1.0


def test_velocity_smoother_arrival_softening_stays_within_goal_tolerance():
    """velocity_smoother.max_decel만 도착 시 완만함을 담당한다(decel_lim_x는
    항상 비상 제동 강도를 유지). 완화된 정지가 xy_goal_tolerance 안에서
    끝나고 progress_checker가 실패로 판단하기 훨씬 전에 끝나야 한다."""
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
    """safety_supervisor_node.cmd_is_alive()는 마지막 명령 수신 이후 경과
    시간(cmd_timeout_sec)만 보고 내용은 보지 않는다. controller_server가
    멈춰도 velocity_smoother가 자체 velocity_timeout까지는 계속 감속 램프
    명령을 내보내므로, velocity_timeout이 cmd_timeout_sec보다 크면 그만큼
    Safety의 freshness 기반 탐지가 지연된다. velocity_timeout을
    cmd_timeout_sec 이하로 유지해 이 탐지 공백을 Safety 자체 timeout
    이내로 수렴시킨다."""
    params = _load_params()
    smoother = params['velocity_smoother']['ros__parameters']

    assert (
        smoother['velocity_timeout']
        <= SAFETY_SUPERVISOR_CMD_TIMEOUT_SEC
    )


def test_smoother_lets_dwb_stop_rotating_as_fast_as_it_plans_to():
    """DWB의 회전 감속 가정과 velocity_smoother의 실제 허용치가 어긋나면 안 된다.

    DWB는 decel_lim_theta만큼 회전을 줄일 수 있다고 보고 궤적을 평가한 뒤
    명령을 낸다. velocity_smoother가 그보다 약하게 감속시키면 실제 거동이
    계획을 지나치고, 반대로 꺾었다가 또 지나치는 한계 진동이 생긴다
    (2026-07-27 주행 중 비틀거림).

    직선 감속도 같은 이유로 정합해야 한다(아래 별도 테스트).
    """
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
    """직선 감속도 DWB 가정과 정합해야 한다.

    smoother가 약하면 DWB가 계산한 정지거리보다 실제로 더 멀리 간다.
    2026-07-28 주행에서 -1.0(DWB -2.5의 1/2.5)이었을 때 명령이 vx +0.010
    m/s인데 실제는 +0.227 m/s였고, 전방 우측 범퍼가 충돌했다.

    이 정합만으로 충돌이 막히지는 않는다. 정지거리의 대부분은 CAN·드라이버
    응답 지연 300 ms 구간의 이동(0.227 x 0.3 = 6.8 cm)이고, 감속 구간은
    -1.0에서 2.6 cm, -2.5에서 1.0 cm다. 그래도 줄일 수 있는 쪽은 줄인다.
    """
    params = _load_params()
    follow_path = params['controller_server']['ros__parameters']['FollowPath']
    smoother = params['velocity_smoother']['ros__parameters']

    dwb_x_decel = abs(follow_path['decel_lim_x'])
    smoother_x_decel = abs(smoother['max_decel'][0])

    assert smoother_x_decel >= dwb_x_decel, (
        f'smoother 직선 감속 {smoother_x_decel}이 '
        f'DWB decel_lim_x {dwb_x_decel}보다 약하다 — 정지거리가 계획보다 길어진다'
    )


def test_stopping_distance_is_documented_against_padding():
    """정지거리가 padding을 넘는다는 사실을 수치로 고정한다.

    지연 300 ms는 실측값이다(2026-07-28: cmd_vel_safe -> wheel/odom).
    이 테스트는 padding이 정지거리를 덮는다고 착각하는 것을 막는다.
    padding을 늘려 덮으려면 inscribed radius가 최협 통로 반폭 0.35 m를
    넘어 통과 자체가 불가능해지므로, 해결은 드라이버 지연 쪽에 있다.
    """
    params = _load_params()
    follow_path = params['controller_server']['ros__parameters']['FollowPath']
    smoother = params['velocity_smoother']['ros__parameters']
    local = params['local_costmap']['local_costmap']['ros__parameters']

    v = follow_path['max_vel_x']
    decel = abs(smoother['max_decel'][0])
    driver_delay_sec = 0.3  # 실측: CAN·드라이버 구간

    stopping_distance = v * driver_delay_sec + v ** 2 / (2 * decel)
    padding = local['footprint_padding']

    assert stopping_distance > padding, (
        f'정지거리 {stopping_distance:.3f} m가 padding {padding} m 이하로'
        f' 계산됐다. 실측 지연이 줄었다면 driver_delay_sec를 갱신하라'
    )
