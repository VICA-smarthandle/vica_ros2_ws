"""costmap footprint가 실제 차체(vica_description/meshes/base_link.stl)를 덮는지 감시한다.

2026-07-27 화장실 주행에서 커브 중 전방 좌측 범퍼가 실제로 충돌했다. 원인은
footprint 전방이 +0.15 m로 선언되어 있었으나 실제 차체는 +0.305 m까지 뻗어 있어,
Nav2가 15.5 cm를 free 공간으로 오인한 것이었다. 좌우도 각 4 cm 짧았다.

이 테스트는 같은 종류의 회귀(차체보다 작은 footprint)를 막는다.
"""
import struct
from pathlib import Path

import pytest
import yaml


# base_link.stl은 mm 단위이고 URDF에서 scale 0.001로 쓰인다.
STL_SCALE = 0.001
# URDF의 collision origin이 xyz="0 0 -body_center_z"로 z만 오프셋이므로
# STL의 x/y는 base_link 좌표와 직접 대응한다. z 값이 바뀌어도 이 대응은 유지된다.
#
# 후방과 좌우는 STL을 신뢰하지 않는다. CAD 후방부가 실제 차체를 반영하지 못한다는
# 것이 2026-07-29에 확인됐다 -- 줄자로 중심 -> 핸들 끝이 56.5 cm인데 STL은 -0.505로
# 6 cm 짧았고, 핸들 기둥 위치도 CAD(-0.265~-0.305) 대비 실측(-0.205)이 7~10 cm
# 어긋났다. 그래서 이 저장소는 CAD보다 줄자를 우선한다.
#
# 2026-08-13 하드웨어 최종 실측(사용자 확정):
#   차체 61 cm (앞뒤 대칭이라 +-0.305) · 손잡이 19 cm 돌출 · 총 세로 80 cm
#   반폭 0.225 · 손잡이 최대폭 7 cm
# 후방은 -0.305 - 0.19 = -0.495 다. 2026-08-02 값(-0.595, 손잡이 29 cm)에서
# 손잡이가 10 cm 짧아졌다.
MEASURED_REAR = -0.495
MEASURED_HALF_WIDTH = 0.225
# STL과 실측이 이보다 더 벌어지면 둘 중 하나가 낡은 것이므로 사람이 봐야 한다.
# 현재 차이는 반폭에서 1.5 mm(STL 0.2265 vs 실측 0.225)로, padding 5 cm 안에 묻힌다.
STL_TOLERANCE = 0.01
LASER_X = 0.185  # VICA.xacro laser_x
CAMERA_X = 0.28683  # VICA.xacro camera_x


def _load_params():
    config_path = Path(__file__).parents[1] / 'config' / 'nav2_params.yaml'
    return yaml.safe_load(config_path.read_text(encoding='utf-8'))


def _footprint(costmap_name):
    params = _load_params()
    raw = params[costmap_name][costmap_name]['ros__parameters']['footprint']
    return yaml.safe_load(raw)


def _stl_path():
    return (
        Path(__file__).parents[2]
        / 'vica_description' / 'meshes' / 'base_link.stl'
    )


def _stl_xy_bounds():
    """base_link.stl을 XY로 투영한 (min_x, max_x, max_abs_y)를 m 단위로 돌려준다."""
    data = _stl_path().read_bytes()
    triangle_count = struct.unpack('<I', data[80:84])[0]
    min_x, max_x, max_abs_y = 1e9, -1e9, 0.0
    for i in range(triangle_count):
        base = 84 + i * 50
        for vertex in range(3):
            off = base + 12 + vertex * 12
            x, y, _z = struct.unpack('<fff', data[off:off + 12])
            min_x = min(min_x, x * STL_SCALE)
            max_x = max(max_x, x * STL_SCALE)
            max_abs_y = max(max_abs_y, abs(y * STL_SCALE))
    return min_x, max_x, max_abs_y


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_footprint_covers_the_real_chassis(costmap):
    if not _stl_path().exists():
        pytest.skip('vica_description/meshes/base_link.stl 없음')

    _min_x, max_x, max_abs_y = _stl_xy_bounds()
    points = _footprint(costmap)

    front = max(p[0] for p in points)
    rear = min(p[0] for p in points)
    half_width = max(abs(p[1]) for p in points)

    # 전방은 STL과 줄자가 일치한다(+0.305). 여기서 짧으면 범퍼가 costmap상
    # free 공간을 쓸고 지나가 충돌한다 -- 2026-07-27에 실제로 그랬다.
    assert front >= max_x, (
        f'{costmap} footprint 전방 {front}이 실제 차체 {max_x:.3f}보다 짧다'
    )
    # 좌우·후방은 줄자 실측이 기준이다. CAD 후방부가 실물과 다르다는 것이
    # 확인됐기 때문이다(위 상수 주석).
    assert half_width >= MEASURED_HALF_WIDTH, (
        f'{costmap} footprint 반폭 {half_width}이 실측 {MEASURED_HALF_WIDTH}보다 좁다'
    )
    assert rear <= MEASURED_REAR, (
        f'{costmap} footprint 후방 {rear}이 실측 차체 {MEASURED_REAR}보다 짧다'
    )
    # 다만 STL을 아주 버리지는 않는다. 실측과 CAD가 크게 벌어지면 둘 중 하나가
    # 낡은 것이고, 그걸 모르고 지나가면 2026-07-27 같은 충돌로 돌아온다.
    assert max_abs_y - half_width <= STL_TOLERANCE, (
        f'{costmap} 반폭 실측 {half_width}이 STL {max_abs_y:.4f}보다 '
        f'{max_abs_y - half_width:.4f} m 작다. 허용 {STL_TOLERANCE} m를 넘었으니 '
        'CAD와 실물 중 어느 쪽이 낡았는지 사람이 확인해야 한다'
    )


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_footprint_contains_forward_mounted_sensors(costmap):
    # LiDAR·카메라가 footprint 밖에 있으면 그 자체로 footprint가 틀렸다는 신호다.
    # 구 footprint(전방 +0.15)는 둘 다 바깥이었고, 실제로 충돌로 이어졌다.
    front = max(p[0] for p in _footprint(costmap))
    assert front >= LASER_X
    assert front >= CAMERA_X


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_padding_keeps_a_hard_clearance_margin(costmap):
    """padding은 유일한 '하드 여유'다.

    planner는 벽에서 inscribed radius(반폭 + padding)만큼 떨어진 곳까지 통과
    가능으로 본다. padding이 0이면 차체 가장자리가 벽에 정확히 닿는 경로가
    합법이 되어, AMCL 오차가 그대로 충돌이 된다(2026-07-27 실제 충돌).

    2026-08-02: 하한을 0.05 -> 0.03으로 내린다. 반대 방향 실패가 실측됐다.

    Smac의 GridCollisionChecker는 로봇 중심 셀이 253(INSCRIBED) 이상이면
    footprint 모양을 보지도 않고 충돌로 끝낸다. 그 253 밴드를 칠하는 범위가
    내접반경(반폭 + padding)이므로, padding이 클수록 "여기 서 있으면 안 된다"는
    영역이 넓어진다. 2026-08-02 주행에서 planner "Starting point in lethal
    space"가 회차마다 14회씩 나며 1 m 폭 구간을 통과하지 못했다.

      padding 0.05 -> 내접 0.2775 m -> 1.0 m 통로에서 중심 허용폭 0.445 m
      padding 0.03 -> 내접 0.2575 m ->                        0.485 m

    같은 날 footprint를 육각형으로 바꿔 외접반경을 줄여봤으나 이 판정에는
    영향이 없었다(lethal space 14회 -> 14회). 내접반경은 가장 가까운 변까지의
    거리이고 그 변은 차체 좌우 변이라 뒤를 뾰족하게 해도 안 바뀐다.

    0.03은 costmap 해상도 0.05 m보다 작다. 격자 반올림에 묻힐 수 있어 효과가
    없을 가능성이 있고, 그 경우 남는 축은 차체 좌우 폭 축소(하드웨어)다.
    벽·의자에 스치기 시작하면 0.05로 되돌린다.
    """
    params = _load_params()
    padding = params[costmap][costmap]['ros__parameters']['footprint_padding']
    assert padding >= 0.03, (
        f'{costmap} footprint_padding {padding}은 하드 여유로 부족하다'
    )


# 맵 실측 통로 반폭. 2026-08-13에 scripts/vica_corridor_measure.py로 새 지도
# vica_map_0812_1을 다시 쟀다(free 셀마다 EDT로 가장 가까운 벽까지의 거리를 구하고,
# 미탐색은 벽으로 친다). 로봇이 설 수 있는 곳의 10%tile 폭이 0.70 m라 반폭 0.35 m다.
# 옛 건물 값(scratchpad/corridor_width.py)과 우연히 같아 숫자는 그대로 둔다.
NARROWEST_CORRIDOR_HALF_WIDTH = 0.35
# inflation_radius 상한. 이 값을 넘으면 협착부가 아니라 '보통 통로'에서도
# 비용 0인 중앙선이 사라져, 우회가 아니라 전면 정체가 된다.
#
# 2026-08-13: 0.70 -> 0.650. 0.70은 옛 건물의 지도에서 나온 값인데 로봇은 이미
# 다른 건물에 있다. 새 값은 vica_map_0812_1(990 x 398 px, 0.05 m/px, free
# 168.2 m2)에서 '로봇이 실제로 설 수 있는 곳'(여유 >= 내접 0.275 m)만 골라 낸
# 여유의 중앙값이다. 전체 free로 세지 않는 이유는 넓은 방 한가운데가 분포를
# 끌어올려 통로가 실제보다 넓게 나오기 때문이다.
#
# 이 갱신으로 판정은 바뀌지 않는다. inflation_radius는 이번에 0.55 -> 0.56이고
# 0.56 < 0.650이라 옛 값에서도 새 값에서도 통과한다. 그런데도 고치는 이유는,
# 통과하는 시험이 낡은 상수를 가장 잘 숨기기 때문이다. 다음 사람이 없는 건물의
# 숫자를 근거로 삼지 않게 하려고 갱신한다.
CORRIDOR_HALF_WIDTH_MEDIAN = 0.650

# [미측정 -- 옛 장소 값] 로봇이 실제로 통과하는 경로의 최협 지점 여유
# (analysis/bottleneck_path.py, 2026-07-30 Hybrid 주행). 옛 건물 방2 -> 화장실
# 우회로의 (1.66, 3.04) 지점이다. 이 값보다 inflation_radius가 크면 그 통로에는
# 비용 0인 중앙선이 없다.
#
# 2026-08-13: 새 장소에서는 아직 그 코스를 주행한 적이 없어 다시 잴 수 없다.
# 지도 통계로 대신할 수도 없다 -- 이 값은 '좁은 곳이 어딘가 있다'가 아니라
# '로봇이 실제로 지나간 경로의 최협 지점'이라 주행 bag이 있어야 나온다.
# 그래서 옛 값을 그대로 둔다. 그럴 수 있는 것은 이 상수가 assert가 아니라 아래
# print 경고에만 쓰여 시험을 깨뜨리지 않기 때문이다.
# 새 장소 첫 주행 bag에서 같은 방식으로 다시 재고 이 표식을 지운다.
DRIVEN_CORRIDOR_CLEARANCE = 0.412

# 경로 추종 오차 실측(analysis/why_lethal_under_footprint.py, 2026-07-30
# hybrid_infl035 주행 안내소 구간): 중앙값 0.045, p95 0.120, 최대 0.164 m.
# inflation_radius는 이 오차를 흡수하는 완충이다. 근거는 아래 테스트 주석 참고.
#
# 이 값은 현재 하한으로 강제되지 않는다. '지도에는 있고 실제로는 없는' 장애물의
# inflation이 로봇을 가두는 반대 방향 실패가 나와, 지도 수정이 끝날 때까지
# 하한을 COSTMAP_RESOLUTION으로 낮춰 두었다. 지도가 고쳐지면 이 기준
# (inscribed + 0.120 = 0.397 이상)으로 되돌린다. 상세는 아래 테스트 주석.
PATH_TRACKING_ERROR_P95 = 0.120

# costmap 해상도. 완충이 1셀도 안 되면 벽에서 멀어지려는 경사가 아예 없다.
COSTMAP_RESOLUTION = 0.05

# ── 2026-08-13 외접반경 축소. 이번 회차를 가능하게 만든 사실이다 ───────────────
# 줄자 실측에서 손잡이 돌출이 29 -> 19 cm로 줄어 footprint 후방이 -0.595 ->
# -0.495가 됐고, padding 포함 외접반경이 0.6506 -> 0.5516 m로 줄었다. 내접반경은
# 좌우 폭이 정하므로 0.275 m로 거의 그대로다(반폭 0.225 + padding 0.05).
#
# 이 10 cm가 막혀 있던 축을 열었다.
#
#   컨트롤러 교체 전제 :  inflation_radius >= 외접반경
#      이전  0.651 필요  vs  통로 반폭 중앙값 0.65  ->  창이 없다. 불가능
#      지금  0.552 필요  vs  통로 반폭 중앙값 0.65  ->  0.56 으로 충족. 여유 9 cm
#
# 전제인 이유: Smac의 GridCollisionChecker는 inflation_radius 바깥에서 footprint
# 검사를 건너뛴다. 그래서 inflation_radius가 외접반경보다 짧으면 그 사이 폭이
# planner의 사각지대가 된다 -- 외접이 0.651이던 때 inflation 0.55는 벽에서
# 0.55~0.651 m(폭 10 cm)를 검사 없이 통과시켰다. 지금 그 구간을 막는 것은
# DWB의 ObstacleFootprint critic 하나뿐이라, MPPI처럼
# potential-field 최적화를 쓰는 controller로 바꾸면 방벽이 사라진다.
# 그래서 이번에 inflation_radius를 0.55 -> 0.56으로 올려 외접반경 0.5516을 덮는다.
# 상세: devlog/2026-08-05-inflation-외접반경-검증.md (5절 · 8절)
#
# 상한(CORRIDOR_HALF_WIDTH_MEDIAN 0.650)과 하한(내접 0.275 + 추종 오차 0.120 =
# 0.395) 사이가 이제 [0.395, 0.650]이고, 0.56은 그 안이다. 외접반경이 0.651이던
# 동안에는 상한 0.65 위에 있어 두 조건을 함께 만족시킬 수 없었다.


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_inflation_radius_keeps_the_path_off_the_wall(costmap):
    """inflation_radius는 '벽에서 이만큼은 떨어져라'는 목표 이격거리다.

    너무 작으면(0.35) 경사 구간이 몇 cm뿐이라 통로가 평탄해지고 NavFn이 벽에
    붙은 최단경로를 그린다 — 2026-07-27 전방 범퍼 충돌의 배경이다.

    너무 크면(0.65) 통로 '전체'가 비용 지대가 된다. NavFn은 경로 비용 합을
    최소화하므로 비용 0인 먼 길로 우회하고, 좁은 길에서는 최소비용 경로가
    거의 동점이 되어 경로가 계속 뒤바뀌며 주춤거린다 — 같은 날 실주행 확인.

    그래서 하한(실효 여유 확보)과 상한(통로 중앙부는 평탄 유지)을 함께 건다.
    """
    params = _load_params()
    cm = params[costmap][costmap]['ros__parameters']
    padding = cm['footprint_padding']
    half_width = max(abs(p[1]) for p in _footprint(costmap))
    inscribed = half_width + padding
    inflation_radius = cm['inflation_layer']['inflation_radius']

    # 하한: 경로 추종 오차를 흡수할 완충이 있어야 한다.
    #
    # 이 하한은 두 번 근거가 바뀌었다.
    #
    # 2026-07-28까지는 0.15였다. 근거는 inflation_radius 0.38을 시험했다가
    # 61초 갇혀 되돌린 경험이었고, 결론은 "inflation을 줄이면 좁은 곳 진입 자체를
    # 막지 못한다"였다. 그때 planner는 SmacPlanner2D로 로봇을 점으로 봤다
    # (Node2D::isNodeValid -> inCollision(index, bool), 셀 하나). 진입 억제력이
    # 없었던 것은 inflation이 작아서가 아니라 planner가 footprint를 안 봤기
    # 때문이라, 이 근거는 Hybrid 전환으로 수명을 다했다.
    #
    # 2026-07-30에 잠시 'costmap 1셀 경사'로 낮췄고(0.05), 그 값이 0.35를
    # 통과시켰다. 그 주행에서 안내소 구간이 ABORTED로 실패했다. 원인을 셋으로
    # 분리한 결과(analysis/why_lethal_under_footprint.py):
    #   planner가 낸 /plan 39개는 전부 여유 >= 0.283 m로 lethal을 지나지 않았고,
    #   AMCL 점프도 없었다(물리 최대 속도의 1.5배 초과 0회).
    #   실제 원인은 경로 추종 오차였다 -- 중앙값 0.045, p95 0.120, 최대 0.164 m.
    #   경로 최소 여유 0.283 m는 내접 0.277 위로 0.6 cm뿐이라, 오차 16 cm가
    #   그것을 먹고 footprint 안에 lethal이 들어왔다(0.283 - 0.164 = 0.119).
    #   그러면 planner가 "Starting point in lethal space"로 아무 경로도 못 내고,
    #   Spin은 외접반경을 쓸어야 해서 여유 0.180 m에서는 원리적으로 불가하다.
    #   (당시 계산의 외접 0.675 m는 낡았다. 2026-08-13 실측은 0.5516 m다.
    #    그래도 0.180 m 여유에서는 여전히 회전할 수 없어 결론은 그대로다.)
    #
    # 그래서 하한의 정체는 '경사의 존재'가 아니라 '추종 오차 완충'이다.
    # 비용 0 지대에서 planner는 경로 길이만 최소화하므로 경로 여유는
    # inflation_radius 근처에 수렴한다. 따라서 inflation_radius가 내접 + 추종
    # 오차보다 작으면 오차가 그대로 lethal 진입이 된다.
    #
    # footprint_padding으로 대신할 수 없다. padding은 하드 판정(253)을 키워
    # 통과 가능성 자체를 줄인다. 완충은 소프트 비용인 inflation의 일이다.
    #
    # ── 2026-07-30 저녁: 이 하한을 다시 1셀로 낮춰 0.35를 허용한다 ──
    # 반대 방향의 실패 사례가 나왔기 때문이다. 두 근거가 서로 당긴다.
    #
    #   큰 inflation  추종 오차를 흡수한다 (위 문단)
    #                 그러나 '지도에는 있고 실제로는 없는' 장애물의 inflation이
    #                 통로를 막아 로봇을 가둔다.
    #   작은 inflation  유령 inflation에 덜 걸린다
    #                   그러나 추종 오차가 내접반경을 먹는다.
    #
    # 사용자 실측 관찰(2026-07-30, 방2 -> 화장실): 지도에는 있고 실주행에는 없던
    # 장애물 때문에 global은 그것을 피해 경로를 냈는데 local은 비어 있어 DWB가
    # 벽에 붙어 갔고, 결국 지도에 고정된 그 장애물의 inflation에 걸려 lethal로
    # 판정되어 탈출하지 못했다.
    #
    # 이 긴장은 지도를 고치면 사라진다 -- 지도가 실제와 맞으면 유령 inflation이
    # 없어지고 추종 오차 논거만 남는다. 사용자가 다음 주행 전에 지도에 실제
    # 장애물을 채워 넣기로 했다. 따라서 0.35는 '지도 수정 중'에만 유효한 값이며,
    # 지도가 고쳐지면 PATH_TRACKING_ERROR_P95 기준(0.397 이상)으로 되돌려야 한다.
    #
    # 기록해 둘 실측: 0.35에서 안내소 구간이 ABORTED 됐고 내접 미달 지점이 3점
    # 있었다(2026-07-30 hybrid_infl035). 이 하한을 낮추는 것은 그 위험을 안는 것이다.
    # 2026-07-30 밤: 하한을 추종 오차 기준으로 되돌린다. 위 문단이 예고한 조건이
    # 충족됐다 -- 사용자가 실물 장애물을 원위치로 복원해 지도-실제 불일치를 없앴고,
    # 그래서 '유령 inflation이 통로를 막는' 쪽 근거가 사라졌다. 남은 것은 추종 오차
    # 논거뿐이다.
    #
    # 그리고 0.35의 위험이 실제 사고로 나타났다(2026-07-30 lattice_infl035_mapfixed):
    # 화장실 구간에서 lethal space가 14회 났고, 그 상태에서 Spin이 253 밴드에서
    # 회전을 시작해 핸들이 의자 등받이에, 좌측 후방 모서리가 의자 다리에 부딪혔다.
    # 회전 중 footprint 외곽선 최대값은 99(INSCRIBED)로 254에 닿지 않아
    # isCollisionFree를 통과했다. 253 밴드는 벽에서 내접반경 안쪽이고 회전하면
    # 외접반경이 쓸리므로, 그 밴드 진입 자체를 줄이는 것이 유일한 방어다.
    # (이 계산에 쓴 외접 0.675 m는 낡았다. 2026-08-13 손잡이 축소 실측으로
    #  0.5516 m다. 내접 0.275 m 밴드 안에서 0.552 m를 쓰는 것은 마찬가지라
    #  결론은 그대로고, 다만 회전 시 쓸리는 반경이 12 cm 줄었다.)
    assert inflation_radius - inscribed >= PATH_TRACKING_ERROR_P95, (
        f'{costmap} 완충 {inflation_radius - inscribed:.3f} m가 경로 추종 오차'
        f' p95 {PATH_TRACKING_ERROR_P95} m보다 작다. 오차가 내접반경을 먹어'
        ' footprint가 253 밴드에 들어가고, 그 자리에서 Spin이 회전하면 부딪힌다'
        ' (2026-07-30 의자 충돌)'
    )
    # 상한: 로봇이 실제로 지나는 통로에 비용 0인 중앙선이 남아야 한다.
    #
    # 2026-07-30 Hybrid 주행에서 inflation_radius 0.45가 이 조건을 깼다.
    # 우회로 최협 지점 (1.66, 3.04)의 여유가 0.412 m라 통로 전체가 비용 지대가
    # 되었고, DWB가 어느 궤적을 골라도 비용을 물어 vx=0 / wz=±0.02로 30초간
    # 진동했다.
    #
    # ── 2026-07-31: 상한 기준을 최협 여유에서 통로 중앙값으로 올린다 ──
    # 정책이 바뀌었다. 종전 상한 0.412의 전제는 "그 협착부를 지나야 한다"였는데,
    # 이제는 복구가 필요할 만한 좁은 길은 굳이 지나지 않고 돌아가게 하는 것이
    # 목표다. 협착부가 비용 지대가 되어 planner가 우회하는 것은 결함이 아니라
    # 의도다. 따라서 0.412를 넘는 것 자체는 허용한다.
    #
    # 다만 상한이 없으면 안 된다. 지도 전체가 비용 지대가 되면 우회할 곳도
    # 사라져 아무 데도 못 간다. 기준은 '보통 통로'다 -- 맵 실측 통로 반폭의
    # 중앙값 0.70 m를 넘으면 협착부가 아니라 일반 통로에서도 비용 0인 중앙선이
    # 사라진다. 그때는 우회가 아니라 전면 정체가 된다.
    #
    # DRIVEN_CORRIDOR_CLEARANCE(0.412)는 지우지 않고 남긴다. 이 값을 넘긴
    # 실험에서 화장실 우회로를 못 쓰게 되는 것이 예상 결과이고, 그 통로를 다시
    # 쓰기로 하면 상한을 이 값으로 되돌려야 한다.
    assert inflation_radius <= CORRIDOR_HALF_WIDTH_MEDIAN, (
        f'{costmap} inflation_radius {inflation_radius}가 실측 통로 반폭 중앙값'
        f' {CORRIDOR_HALF_WIDTH_MEDIAN} m를 넘어, 협착부가 아니라 일반 통로에도'
        ' 비용 0인 중앙선이 없다. 우회가 아니라 전면 정체가 된다'
    )
    if inflation_radius > DRIVEN_CORRIDOR_CLEARANCE:
        # 실패가 아니라 기록이다. 이 조건에서는 화장실 우회로 최협 지점
        # (1.66, 3.04)에 비용 0인 셀이 없다.
        print(
            f'[주의] {costmap} inflation_radius {inflation_radius}는 실주행'
            f' 통로 최협 여유 {DRIVEN_CORRIDOR_CLEARANCE} m를 넘는다.'
            ' 그 통로는 전 구간이 비용 지대이며 우회로가 없으면 진동한다.'
        )


def test_local_and_global_costmap_use_the_same_footprint():
    # 두 costmap이 다르면 planner가 통과 가능하다고 만든 경로를
    # controller가 거부해 주행이 멈춘다.
    assert _footprint('local_costmap') == _footprint('global_costmap')

    params = _load_params()
    local = params['local_costmap']['local_costmap']['ros__parameters']
    global_ = params['global_costmap']['global_costmap']['ros__parameters']
    assert local['footprint_padding'] == global_['footprint_padding']
