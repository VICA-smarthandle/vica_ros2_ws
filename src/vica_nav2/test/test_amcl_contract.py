"""위치추정(AMCL) 설정이 자기잠금으로 되돌아가지 않는지 감시한다. NAV2-B1.

이 로봇에는 **절대 방위를 주는 센서가 없다.** EKF에 들어가는 두 입력이 모두 각'속도'만
준다(`vica_localization/config/ekf.yaml`의 odom0 = vx·vy·vyaw, imu0 = vyaw).
따라서 yaw는 순수 적분이고 **그 드리프트를 되돌리는 장치는 AMCL 하나뿐**이다.

그래서 "AMCL이 측정을 얼마나 자주 반영하는가"가 이 로봇에서는 편의 문제가 아니라
위치추정 정확도 그 자체다. 아래 시험은 2026-07-31 실주행에서 실제로 난 사고의
수치를 상수로 박아 두고, 설정이 그 사고를 다시 허용하는 값으로 돌아가면 깨진다.

사고 기록 (`devlog/2026-07-31.md:14-64`):

    실제 로봇이 AMCL이 믿은 곳보다 26~30 cm 멀리 있었다.
    AMCL 추정이 t+77.6 ~ 85.6 동안 (1.837, 3.848, 94.4도)로 완전히 고정됐다.
    20초를 갇혔다.

되먹임의 모양이 이렇다.

    자세가 틀림 -> 못 움직임 -> 갱신 문턱 미달 -> 자세가 계속 틀림

**갇힐수록 갱신이 안 되는 구조**라 스스로 빠져나올 수 없다. 문턱을 낮추는 것이
유일한 탈출구다.
"""
import math
from pathlib import Path

import yaml


# --- 2026-07-31 실주행 실측. 이 값들이 이 파일의 모든 상한의 근거다 ---

# AMCL 추정과 실제 위치의 어긋남 [m]. 세 시점에서 30.5 / 31.4 / 43.0 cm였고
# 가장 작은 것이 30.5 cm다. 갱신 문턱이 이보다 크면 "고쳐야 할 오차"보다
# "고치기 위해 움직여야 하는 거리"가 더 커서 영영 못 고친다.
OBSERVED_POSE_ERROR_M = 0.305

# 같은 주행에서 관측된 AMCL yaw 점프 [deg]. 이 크기의 각도 오차가 실재하므로
# 회전 갱신 문턱은 이보다 작아야 그 오차를 회전만으로도 잡을 수 있다.
OBSERVED_YAW_JUMP_DEG = 10.8

# 한 스캔의 유효 반사 개수. 정합점 측정에서 414 / 414 / 417개였다.
SCAN_VALID_RETURNS = 414


def _load_params():
    config_path = (
        Path(__file__).parents[1] / 'config' / 'nav2_params.yaml'
    )
    return yaml.safe_load(config_path.read_text(encoding='utf-8'))


def _amcl():
    return _load_params()['amcl']['ros__parameters']


def test_translation_update_threshold_is_smaller_than_the_error_it_must_fix():
    """갱신에 필요한 이동 거리가 고쳐야 할 위치 오차보다 작아야 한다.

    이것이 자기잠금의 정체다. `update_min_d`가 0.25일 때 로봇은 30 cm 어긋난 자세를
    고치려고 25 cm를 먼저 움직여야 했는데, 자세가 틀려서 좁은 곳에 갇혀 있었으므로
    그만큼 움직일 수가 없었다. 8초 동안 추정값이 소수점까지 동일했다.

    좁은 곳에서 조금씩 더듬는 동작 — 정확히 갇혔을 때 하는 동작 — 이 갱신을
    트리거하지 못하면 AMCL은 눈을 감은 채로 남는다.
    """
    amcl = _amcl()

    assert amcl['update_min_d'] < OBSERVED_POSE_ERROR_M, (
        '갱신 문턱이 실측 오차 30.5 cm 이상이면 그 오차를 고칠 수 없다'
    )
    # 실측 오차와 같기만 해서는 부족하다. 그 오차를 '여러 번에 걸쳐' 줄여야 하므로
    # 한 자릿수 cm 대의 이동에서도 갱신이 일어나야 한다.
    assert amcl['update_min_d'] <= 0.10


def test_rotation_update_threshold_is_smaller_than_the_observed_yaw_jump():
    """갱신에 필요한 회전각이 실제로 난 yaw 오차보다 작아야 한다.

    절대 방위 센서가 없어 yaw는 순수 적분이다(모듈 docstring). AMCL이 이 드리프트를
    되돌리는 유일한 장치인데, `update_min_a` 0.2 rad = 11.46도는 실측된 yaw 점프
    10.8도보다 커서 **그만한 오차가 나 있어도 회전만으로는 갱신이 걸리지 않는다.**

    DWB의 미세 자세 보정은 대부분 이 문턱을 넘지 못한다. 복구 Spin(spin_dist 0.30
    rad = 17.2도)만 넘기는데, 복구까지 가기 전에 잡는 것이 목적이다.
    """
    amcl = _amcl()

    observed_rad = math.radians(OBSERVED_YAW_JUMP_DEG)
    assert amcl['update_min_a'] < observed_rad, (
        f'update_min_a {amcl["update_min_a"]} rad '
        f'= {math.degrees(amcl["update_min_a"]):.1f}도는 '
        f'실측 yaw 점프 {OBSERVED_YAW_JUMP_DEG}도보다 크다'
    )
    assert amcl['update_min_a'] <= 0.10


def test_amcl_reads_enough_of_the_scan_to_tell_poses_apart():
    """스캔의 일부만 보면 틀린 자세와 맞는 자세를 구분하지 못한다.

    2026-07-31 정합점 측정에서 AMCL이 믿은 자세의 정합률은 13~22 %였고 최적 보정
    후에는 47~50 %였다. **그 둘을 가르는 신호가 스캔 안에 분명히 있었다.** 그런데
    `max_beams` 60은 유효 반사 414개 중 14.5 %만 표본으로 쓴다.

    대가는 CPU 3배다. 되돌릴 때 1순위가 이 값이며, 상한은 제어주기 놓침으로
    판정한다: `grep -c "missed its desired rate" ~/.ros/log/controller_server_*.log`
    """
    amcl = _amcl()

    fraction = amcl['max_beams'] / SCAN_VALID_RETURNS
    assert fraction >= 0.25, (
        f'max_beams {amcl["max_beams"]}는 유효 반사 {SCAN_VALID_RETURNS}개의 '
        f'{fraction:.1%}뿐이다'
    )
    # 상한도 둔다. Jetson은 nvblox·STT와 CPU를 다투므로(2026-07-30 GPU 경합 조사)
    # 전수에 가깝게 올리면 제어주기를 놓친다.
    assert amcl['max_beams'] <= 240


def test_rotation_noise_is_larger_than_translation_noise():
    """회전 잡음을 병진 잡음보다 크게 두어야 회전 뒤 자세를 되돌릴 수 있다.

    Humble `differential_motion_model.cpp` 기준:

        alpha1  회전 -> 회전 오차     alpha2  병진 -> 회전 오차
        alpha3  병진 -> 병진 오차     alpha4  회전 -> 병진 오차

    이 로봇은 **제자리 회전 직후에 오차가 컸다.** alpha1을 올리면 회전 시 입자
    분산이 커져 스캔 매칭이 자세를 되돌릴 여지가 생긴다. 전부 같은 값(Nav2 샘플
    기본 0.2)으로 두면 "회전이 더 못 미덥다"는 이 로봇의 사실이 모델에 들어가지
    않는다.

    t+86의 AMCL 점프 -10.8도가 오히려 자세를 더 틀리게 만든 관찰과 맞물린다
    (`devlog/2026-07-31.md:48-50`).
    """
    amcl = _amcl()

    assert amcl['alpha1'] > amcl['alpha3'], (
        '회전->회전 잡음(alpha1)이 병진->병진 잡음(alpha3) 이하면 '
        '회전 직후 오차를 모델이 표현하지 못한다'
    )
    assert amcl['alpha1'] > amcl['alpha2']
    assert amcl['alpha1'] > amcl['alpha4']


def test_global_relocalization_stays_disabled():
    """전역 재추정(recovery_alpha)을 켜지 않는다. 이건 '하지 말 것'이다.

    30 cm 어긋남에서 빠져나올 수단이 없다는 이유로 이 값을 켜고 싶어지는데,
    **D4가 미해결이다** — 정지 상태에서 AMCL이 15 cm / 10.8도씩 재정렬하는 현상의
    빈도와 크기를 아직 측정하지 못했다(`docs/nav2_backlog.md` §8 D4).

    빈도를 모르는 채로 전역 재추정을 켜면 오작동 시 입자가 **더 나쁜 곳으로 점프**한다.
    핸들을 잡은 시각장애인이 뒤에 있는 로봇에서 그건 회복이 아니라 사고다.

    같은 문제의 올바른 답은 NAV2-B2 `pose_bootstrap` — 기동 시 한 번 제대로 잡고
    검증에 실패하면 그 자리에 서는 것이다.
    """
    amcl = _amcl()

    assert amcl['recovery_alpha_slow'] == 0.0
    assert amcl['recovery_alpha_fast'] == 0.0
