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
