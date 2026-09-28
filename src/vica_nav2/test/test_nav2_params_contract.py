from pathlib import Path

import pytest
import yaml


# safety_supervisor_node.py의 cmd_timeout_sec 기본값 (declare_parameter default).
# launch에서 override되지 않으므로 실제 런타임 값과 동일하다.
SAFETY_SUPERVISOR_CMD_TIMEOUT_SEC = 0.5


def _load_params():
    config_path = (
        Path(__file__).parents[1] / 'config' / 'nav2_params.yaml'
    )
    return yaml.safe_load(config_path.read_text(encoding='utf-8'))


def _controller_limits(params):
    """컨트롤러 종류와 무관하게 속도·가속 한계를 같은 이름으로 돌려준다.

    2026-09-18 MPPI 전환. 아래 계약들이 지키는 것은 "컨트롤러가 계획한 가·감속을
    velocity_smoother 가 막지 않는가", "완만한 정지가 goal tolerance 안에서
    끝나는가" 같은 **관계**이지 키 이름이 아니다. 그래서 이름만 옮겨 준다.

        DWB                MPPI            뜻
        max_vel_x          vx_max          직진 상한
        min_vel_x          vx_min          후진 허용 여부
        max_vel_theta      wz_max          회전 상한
        acc_lim_x          ax_max          직진 가속
        decel_lim_x        ax_min          직진 감속(음수)
        acc_lim_theta      az_max          회전 가속
        decel_lim_theta    -az_max         회전 감속 — MPPI 는 가·감속이 대칭이다
        xy_goal_tolerance  (goal_checker)  MPPI 는 자체 도착 판정이 없다

    2026-09-21 RPP 전환. RPP 는 더 적다 — **회전 상한도 가·감속도 없다.**
    그 셋은 velocity_smoother 가 유일한 방어라 거기서 읽는다.

        RPP                             뜻
        desired_linear_vel              직진 상한
        (없음 · allow_reversing false)  후진 허용 여부
        smoother.max_velocity[2]        회전 상한      ← 컨트롤러에 없다
        smoother.max_accel[0]           직진 가속      ← 컨트롤러에 없다
        smoother.max_decel[0]           직진 감속      ← 컨트롤러에 없다
        smoother.max_accel[2]           회전 가속      ← 컨트롤러에 없다
    """
    controller = params['controller_server']['ros__parameters']
    fp = controller['FollowPath']
    if 'vcc' in fp['plugin'].lower():
        # 2026-09-24 VCC 전환. VCC 는 자기 출력단에 한계를 갖는다(설계서 7절).
        # 직진 가속은 출발 램프의 앞 구간(start_ramp_accel)이 가장 세다.
        gc = controller['general_goal_checker']
        return {
            'plugin_family': 'vcc',
            'max_vel_x': fp['desired_linear_vel'],
            'min_vel_x': 0.0,   # 출력단이 v 를 [0, max] 로 자른다 — 후진 없음
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
            # allow_reversing false 면 후진을 안 쓴다 = min_vel_x 0 과 같은 뜻이다.
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

    assert goal_checker['plugin'] == (
        'nav2_controller::StoppedGoalChecker'
    )
    # stateful=False여야 감속 중 tolerance를 재이탈해도 속도 조건만으로
    # 성공 처리되지 않고 매 주기 XY/yaw를 재검사한다.
    assert goal_checker['stateful'] is False
    assert goal_checker['trans_stopped_velocity'] == 0.03
    assert goal_checker['rot_stopped_velocity'] == 0.05
    # DWB 는 자체 도착 판정도 하므로 goal_checker 와 같은 값이어야 한다.
    # MPPI 는 자체 판정이 없어 goal_checker 하나가 정본이다(_controller_limits).
    assert lim['trans_stopped_velocity'] == goal_checker['trans_stopped_velocity']
    assert lim['xy_goal_tolerance'] == goal_checker['xy_goal_tolerance']


def test_dwb_deceleration_limit_keeps_full_emergency_braking_power():
    """DWB decel_lim은 장애물 회피 trajectory rollout과 controller 정지 시
    제동에도 쓰이는 전역 값이다. 목적지 도착을 부드럽게 하려고 이 값을
    약화시키면 안 된다 — 완화는 velocity_smoother.max_decel에서만 한다."""
    params = _load_params()
    lim = _controller_limits(params)

    assert abs(lim['decel_lim_x']) >= 1.0
    assert abs(lim['decel_lim_theta']) >= 1.0


def test_velocity_smoother_arrival_softening_stays_within_goal_tolerance():
    """velocity_smoother.max_decel만 도착 시 완만함을 담당한다(decel_lim_x는
    항상 비상 제동 강도를 유지). 완화된 정지가 xy_goal_tolerance 안에서
    끝나고 progress_checker가 실패로 판단하기 훨씬 전에 끝나야 한다."""
    params = _load_params()
    controller = params['controller_server']['ros__parameters']
    lim = _controller_limits(params)
    progress_checker = controller['progress_checker']
    smoother = params['velocity_smoother']['ros__parameters']

    max_vel_x = smoother['max_velocity'][0]
    max_decel_x = abs(smoother['max_decel'][0])

    stop_time = max_vel_x / max_decel_x
    stop_distance = max_vel_x ** 2 / (2.0 * max_decel_x)

    assert stop_distance < lim['xy_goal_tolerance']
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
    """직선 감속도 DWB 가정과 정합해야 한다.

    smoother가 약하면 DWB가 계산한 정지거리보다 실제로 더 멀리 간다.
    2026-07-28 주행에서 -1.0(DWB -2.5의 1/2.5)이었을 때 명령이 vx +0.010
    m/s인데 실제는 +0.227 m/s였고, 전방 우측 범퍼가 충돌했다.

    이 정합만으로 충돌이 막히지는 않는다. 정지거리의 대부분은 CAN·드라이버
    응답 지연 300 ms 구간의 이동(0.227 x 0.3 = 6.8 cm)이고, 감속 구간은
    -1.0에서 2.6 cm, -2.5에서 1.0 cm다. 그래도 줄일 수 있는 쪽은 줄인다.

    2026-08-01 정정. 그전까지 이 시험은 `smoother >= DWB`를 요구했다. 그런데
    사용자가 승차감을 위해 직선 감속만 -1.0으로 두기로 했다. 시각장애인이 핸들을
    잡고 걷는 로봇이라 정지 순간의 충격이 곧 안전이기 때문이다.

    그래서 요구를 '같아야 한다'에서 **'초과분이 footprint_padding 안에 있어야
    한다'**로 바꾼다. 위 주석의 계산이 그대로 근거다 — 최고속도 0.26 m/s에서
    감속 구간 이동은 -2.5에서 1.35 cm, -1.0에서 3.38 cm로 차이가 2.0 cm다.
    padding 0.05 m가 이를 덮는다. 덮지 못하면 DWB가 계획한 정지선을 실제로
    넘어서므로 그때는 감속을 되돌리거나 padding을 늘려야 한다.

    회전 감속(-3.2)은 DWB와 그대로 맞춘다. 조향 지연은 승차감이 아니라
    경로 이탈로 나타나기 때문이다.
    """
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
    """정지거리가 padding을 넘는다는 사실을 수치로 고정한다.

    지연 300 ms는 실측값이다(2026-07-28: cmd_vel_safe -> wheel/odom).
    이 테스트는 padding이 정지거리를 덮는다고 착각하는 것을 막는다.
    padding을 늘려 덮으려면 inscribed radius가 최협 통로 반폭 0.35 m를
    넘어 통과 자체가 불가능해지므로, 해결은 드라이버 지연 쪽에 있다.
    """
    params = _load_params()
    lim = _controller_limits(params)
    smoother = params['velocity_smoother']['ros__parameters']
    local = params['local_costmap']['local_costmap']['ros__parameters']

    v = lim['max_vel_x']
    decel = abs(smoother['max_decel'][0])
    driver_delay_sec = 0.3  # 실측: CAN·드라이버 구간

    stopping_distance = v * driver_delay_sec + v ** 2 / (2 * decel)
    padding = local['footprint_padding']

    assert stopping_distance > padding, (
        f'정지거리 {stopping_distance:.3f} m가 padding {padding} m 이하로'
        f' 계산됐다. 실측 지연이 줄었다면 driver_delay_sec를 갱신하라'
    )


def test_vcc_limits_match_the_smoother():
    """VCC 출력단 한계가 velocity_smoother 와 어긋나면 한쪽이 다른 쪽 계획을 깎는다.

    회전 상한은 smoother 와 같고 좌우 대칭이어야 한다(손잡이 끝 0.62 m/s 문제, yaml 2567-2577).
    직선 제동은 smoother max_decel 과 같아야 한다 — 제동은 완화하지 않는다(yaml 2606-2612).
    """
    params = _load_params()
    fp = params['controller_server']['ros__parameters']['FollowPath']
    if 'vcc' not in fp['plugin'].lower():
        pytest.skip('VCC 가 활성 컨트롤러가 아니다')
    sm = params['velocity_smoother']['ros__parameters']
    assert fp['max_angular_vel'] <= sm['max_velocity'][2]
    assert sm['min_velocity'][2] == -sm['max_velocity'][2]
    assert fp['max_linear_decel'] == abs(sm['max_decel'][0])
    assert fp['desired_linear_vel'] <= sm['max_velocity'][0]
    # 차선 이동 속도는 손잡이 0.115 m/s 탈락선(backlog §11) 아래
    assert fp['lane_rate'] < 0.115


def test_rpp_is_preserved_for_one_line_rollback():
    params = _load_params()
    controller = params['controller_server']['ros__parameters']
    assert 'FollowPathRPP' in controller
    assert 'purepursuit' in controller['FollowPathRPP']['plugin'].lower().replace('_', '')
    assert controller['controller_plugins'] == ['FollowPath']
