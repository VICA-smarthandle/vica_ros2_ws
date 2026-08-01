"""목적지 접근 다단계 감속 사다리 (순수 로직, rclpy 비의존).

왜 필요한가:

    이 로봇은 시각장애인이 핸들을 잡고 따라 걷는 안내 로봇이다. 급정지는 사용자가
    앞으로 쏠려 넘어질 위험을 만든다. 그런데 **급정지의 실체는 감속률이 아니라
    속도 낙차(Δv)** 다. 최고속도가 0.26 m/s뿐이라 `decel_lim_x` -2.5 m/s²로도
    0.104초·1.35 cm면 멈춘다. 사용자 손에 전달되는 것은 "몇 m/s²로 줄였는가"가
    아니라 "얼마에서 0으로 떨어졌는가"다.

    그래서 설계는 "천천히 세운다"가 아니라 **"세우기 전에 이미 느리게 만든다"** 다.
    목적지가 가까워질수록 `/speed_limit`으로 최대속도 상한을 단계적으로 낮춰
    도착 순간의 Δv 자체를 줄인다.

무엇을 건드리지 않는가:

    `/speed_limit` → `DWBLocalPlanner::setSpeedLimit` → `KinematicsHandler` 경로는
    `max_vel_x`·`max_vel_theta`만 바꾼다. `KinematicParameters`는 최대속도에만
    `base_` 원본 짝을 두고 `acc_lim_`/`decel_lim_`에는 두지 않는다. 즉 **제동력은
    줄어들지 않는다** — 비상 제동 성능을 잃지 않는다.

기본 단계 (정지거리 = 드라이버 지연 0.3 s 이동 + v²/(2×2.5)):

    잔여거리    제한     max_vel_x    정지 시 Δv    정지거리
    ~1.5 m     100 %    0.260        0.260        9.2 cm
    ≤1.5 m      70 %    0.182        0.182        6.1 cm
    ≤1.0 m      55 %    0.143        0.143        4.7 cm
    ≤0.5 m      40 %    0.104        0.104        3.3 cm

    3.3 cm는 `xy_goal_tolerance` 0.25 m 안이고, 0.104 m/s는 progress_checker
    요구치(0.10 m / 20 s = 0.005 m/s)를 20배 이상 넘어 정체 판정에 걸리지 않는다.
    `trans_stopped_velocity` 0.03 m/s는 도착 판정 하한이라 충돌하지 않는다.

첫 감속 지점을 3.0 m → 1.5 m로 **늦춘** 이유 (2026-08-01 사용자 판단):

    이 변경 전 코드는 잔여거리 3.0 m에서 70 %를 한 번 거는 단발 래치였다. 즉 이
    작업은 단계를 "추가"한 것이 아니라 **첫 감속 지점을 3.0 m에서 1.5 m로 늦추고,
    그 뒤를 세 단계로 쪼갠 것**이다. 1.5 m 전까지는 100 %로 달린다.

    근거: 이미 저속인 로봇이라 미리 줄이면 손해만 크다. 최고속도 0.26 m/s에서
    3 m는 그 자체로 11.5초다. 여기에 70 %를 걸면 16.5초가 되어 5초를 더 쓰는데,
    그 구간은 아직 도착 준비가 필요한 거리가 아니다. 감속은 Δv를 줄이려는 것이고
    Δv는 마지막 1.5 m 안에서만 의미가 있다.

    **"감속이 늦다"고 되돌리려는 판단이 나오면 이 문단을 먼저 볼 것.** 되돌릴
    근거는 실주행에서 마지막 1.5 m 안에 정지가 여전히 거칠다는 관측이어야 하고,
    그때도 첫 지점을 올리기 전에 단계 개수를 늘리는 쪽을 먼저 시험한다.

    음성 안내(`DISTANCE_MILESTONES_M`)의 3 m 지점은 그대로다. 사용자는 여전히
    3 m에서 "약 3미터 남았습니다"를 듣고, 감속은 그 절반 지점부터 시작한다.

위험 — 실기에서 확인할 것:

    제한율은 `max_vel_x`뿐 아니라 `max_vel_theta`에도 같은 비율로 걸린다. 40 %면
    0.4 → 0.16 rad/s다. **마지막 0.5 m에 코너가 있으면 회전이 느려져 도착이
    지연된다.** 최악의 경우 `yaw_goal_tolerance` 0.25 rad을 맞추는 데 1.6초가 더
    걸린다. 기본값은 목적지 직전 구간이 직선인 목적지를 전제한 값이다. 마지막
    구간에 회전이 남는 목적지가 있으면 그 목적지 기준으로 마지막 단계 거리를
    줄이거나 비율을 올려야 한다.

되돌아가지 않는다 (latch):

    Nav2 잔여거리는 재계획마다 출렁이고, 경로가 바뀌면 늘어나기도 한다. 그때마다
    제한이 내려갔다 올라갔다 하면 사용자 손에 울컥거림으로 전달된다. 그래서 이
    사다리는 **한 방향으로만 내려간다.** 올라가는 유일한 경로는 새 Goal이며,
    목적지 변경·재개·취소·도착·실패·E-stop에서 `reset()`이 불린다.
"""
from __future__ import annotations

from typing import Optional, Sequence, Tuple

# (잔여거리 m, 최대속도 상한 %) — 먼 거리부터. 위 표의 기본값이다.
# 실기 조정은 launch parameter (approach_slowdown_distances_m /
# approach_speed_limit_percents) 로 하고 이 상수는 건드리지 않는다.
DEFAULT_APPROACH_STAGES: Tuple[Tuple[float, float], ...] = (
    (1.5, 70.0),
    (1.0, 55.0),
    (0.5, 40.0),
)

# nav2_msgs/SpeedLimit 은 percentage=True 에서 0.0 을 "제한 없음"으로 읽는다.
NO_SPEED_LIMIT = 0.0

StageList = Tuple[Tuple[float, float], ...]


def normalize_stages(stages) -> StageList:
    """단계 목록을 검증하고 먼 거리부터 정렬해 돌려준다.

    실기에서 파라미터로 바꾸는 값이므로 잘못된 조합은 조용히 굴러가는 것보다
    기동 시점에 죽는 편이 안전하다. 감속이 어긋난 채로 사용자가 핸들을 잡는 것이
    가장 나쁜 결과다.

    빈 목록은 허용한다 — 접근 감속을 끄는 스위치다.
    """
    parsed = []
    for stage in stages:
        try:
            distance, percent = stage
        except (TypeError, ValueError):
            raise ValueError(
                f"단계는 (거리 m, 비율 %) 쌍이어야 한다: {stage!r}"
            ) from None
        distance = float(distance)
        percent = float(percent)
        if not distance > 0.0:
            raise ValueError(f"단계 거리는 0보다 커야 한다: {distance}")
        if not 0.0 < percent <= 100.0:
            raise ValueError(f"단계 비율은 (0, 100] 범위여야 한다: {percent}")
        parsed.append((distance, percent))

    # 먼 거리 -> 가까운 거리. update() 가 "마지막으로 걸리는 단계"를 고를 수 있는
    # 근거이며, 한 tick 에 여러 단계를 건너뛰어도 가장 깊은 단계로 바로 간다.
    parsed.sort(key=lambda s: s[0], reverse=True)

    for (far_d, far_p), (near_d, near_p) in zip(parsed, parsed[1:]):
        if near_d == far_d:
            raise ValueError(f"단계 거리가 중복된다: {far_d}")
        if near_p >= far_p:
            # 가까워질수록 비율이 올라가면 도착 직전에 오히려 빨라진다.
            # 오타를 실주행 전에 잡으려는 검증이다.
            raise ValueError(
                "가까운 단계일수록 비율이 낮아야 한다: "
                f"{far_d}m={far_p}% 다음에 {near_d}m={near_p}%"
            )

    return tuple(parsed)


def stages_from_lists(distances, percents) -> StageList:
    """ROS parameter 두 배열(거리·비율)을 단계 목록으로 합친다.

    ROS 2 파라미터에는 쌍의 배열 타입이 없어 double 배열 둘로 나눠 받는다.
    길이가 어긋나면 어느 거리에 어느 비율인지 알 수 없으므로 거부한다.
    """
    distance_list = [float(d) for d in distances]
    percent_list = [float(p) for p in percents]
    if len(distance_list) != len(percent_list):
        raise ValueError(
            "거리와 비율의 개수가 다르다: "
            f"거리 {len(distance_list)}개, 비율 {len(percent_list)}개"
        )
    return normalize_stages(zip(distance_list, percent_list))


class ApproachSpeedLadder:
    """잔여거리에 따라 한 방향으로만 내려가는 최대속도 제한 사다리.

    시간에 의존하지 않는다 — 판정 입력은 잔여거리 하나뿐이다.
    """

    def __init__(self, stages: Optional[Sequence] = None) -> None:
        """단계 목록을 받는다. None 이면 기본 단계, 빈 목록이면 기능을 끈다."""
        self._stages = normalize_stages(
            DEFAULT_APPROACH_STAGES if stages is None else stages
        )
        # -1 은 "아직 어느 단계에도 들어가지 않음" = 제한 없음.
        self._index = -1

    @property
    def stages(self) -> StageList:
        """검증·정렬을 마친 단계 목록 (먼 거리부터)."""
        return self._stages

    @property
    def index(self) -> int:
        """지금 들어가 있는 단계 번호. 제한 전이면 -1."""
        return self._index

    @property
    def percent(self) -> float:
        """지금 걸려 있는 제한율. 아직 제한 전이면 0.0(해제)."""
        return NO_SPEED_LIMIT if self._index < 0 else self._stages[self._index][1]

    def reset(self) -> None:
        """새 Goal 용 초기화. 다음 접근에서 다시 처음 단계부터 내려간다."""
        self._index = -1

    def update(self, distance_remaining: Optional[float]) -> Optional[float]:
        """이번 tick 에 새로 진입한 단계의 제한율. 바뀐 것이 없으면 None.

        None 을 돌려주면 노드는 아무것도 발행하지 않는다. 같은 제한을 매 tick
        다시 발행할 이유가 없고, 발행하지 않는 것이 곧 latch 유지다.
        """
        target = self._stage_index_for(distance_remaining)
        if target <= self._index:
            # 거리가 늘었거나(재계획·역주행) 같은 단계면 그대로 유지한다.
            return None
        self._index = target
        return self._stages[target][1]

    def _stage_index_for(self, distance_remaining: Optional[float]) -> int:
        """이 거리에 해당하는 가장 깊은 단계 번호. 판단 불가면 현재 단계."""
        # Nav2 는 경로를 계산하기 전에 None 이나 0.0 을 준다. 이를 "다 왔다"로
        # 읽어 마지막 단계로 뛰면 출발도 하기 전에 40 % 로 기어간다.
        # 양수만 신뢰하고, 그 외에는 현재 단계를 유지한다.
        if distance_remaining is None or distance_remaining <= 0.0:
            return self._index

        index = -1
        for i, (threshold, _) in enumerate(self._stages):
            # 경계값은 포함이다. 잔여거리가 정확히 1.5 m 면 첫 단계에 들어간다.
            if distance_remaining <= threshold:
                index = i
        return index
