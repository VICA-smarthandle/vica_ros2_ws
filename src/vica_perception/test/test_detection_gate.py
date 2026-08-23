"""출발 조건 판정(`stable` · `approachable`) 순수 로직 검증 — ROS 없이 돈다.

정본은 `devlog/2026-08-23-사람접근-구현설계.md` §6.1 이다. 이 판정이 잘못되는
방향은 둘이고 둘 다 여기서 막는다.

1. **너무 쉽게 승인한다** — 걷는 사람에게 출발한다. 로봇 상한 0.5 m/s 대 보행
   1.2 m/s 라 절대 따라잡지 못하고, 출발-포기-복귀만 반복하며 통행을 막는다.
2. **영영 승인하지 않는다** — 경계값(1.0초·3.0초·0.3 m·1.5 m·4.0 m)에서 한 칸씩
   어긋나면 멈춰 있는 사람에게도 다가가지 못한다.

시각은 전부 정수 나노초(STEADY_TIME)다. 이 모듈은 `time.time()` 을 부르지 않으며,
테스트가 시각을 직접 만들어 넣는다.
"""
import math

import pytest

from vica_perception.detection_gate import (
    DEFAULT_DETECTION_GAP_S,
    DEFAULT_MAX_DISPLACEMENT_M,
    DEFAULT_MAX_DISTANCE_M,
    DEFAULT_MIN_CONFIDENCE,
    DEFAULT_MIN_DISTANCE_M,
    DEFAULT_STABLE_DURATION_S,
    DEFAULT_STILL_WINDOW_S,
    DetectionGate,
    DetectionSample,
    GateReason,
    GateThresholds,
    Point2D,
    sec_to_ns,
)


# ---- 표본 만들기 -------------------------------------------------------------


def sample(
    t_s,
    track_id=1,
    confidence=0.9,
    x=0.0,
    y=0.0,
    distance_m=2.5,
):
    """초 단위로 쓰고 나노초로 넣는다 — 테스트 가독성용 헬퍼."""
    return DetectionSample(
        stamp_ns=sec_to_ns(t_s),
        track_id=track_id,
        confidence=confidence,
        position=Point2D(x=x, y=y),
        distance_m=distance_m,
    )


def feed_still(gate, start_s, duration_s, period_s=0.2, **kwargs):
    """제자리에 선 사람을 `duration_s` 동안 주기적으로 넣는다.

    5 Hz(0.2초)는 설계 §6.2 의 추론 주기다.
    """
    verdict = None
    steps = int(round(duration_s / period_s))
    for i in range(steps + 1):
        verdict = gate.observe(sample(start_s + i * period_s, **kwargs))
    return verdict


def feed_path(gate, xs, start_s=0.0, period_s=0.2, **kwargs):
    """x 좌표를 순서대로 5 Hz 로 먹인다. 표본 간격은 결손 임계 안이다."""
    verdict = None
    for i, x in enumerate(xs):
        verdict = gate.observe(sample(start_s + i * period_s, x=x, **kwargs))
    return verdict


# ---- 기본값이 설계 문서와 같은가 ---------------------------------------------


def test_default_thresholds_match_design_doc():
    """§6.1 표의 값. 바꾸려면 문서를 먼저 바꾼다."""
    assert DEFAULT_MIN_CONFIDENCE == 0.6
    assert DEFAULT_STABLE_DURATION_S == 1.0
    assert DEFAULT_STILL_WINDOW_S == 3.0
    assert DEFAULT_MAX_DISPLACEMENT_M == 0.3
    assert DEFAULT_MIN_DISTANCE_M == 1.5
    assert DEFAULT_MAX_DISTANCE_M == 4.0


def test_thresholds_are_overridable():
    """신뢰도 0.6 은 `[미검증]` 가정값이다(§12). 실측으로 바꿀 수 있어야 한다."""
    thresholds = GateThresholds(min_confidence=0.35)
    gate = DetectionGate(thresholds)
    verdict = gate.observe(sample(0.0, confidence=0.4))
    assert verdict.reason is not GateReason.LOW_CONFIDENCE
    assert gate.thresholds.min_confidence == 0.35


def test_sec_to_ns_is_integer_nanoseconds():
    assert sec_to_ns(1.0) == 1_000_000_000
    assert sec_to_ns(0.2) == 200_000_000
    assert isinstance(sec_to_ns(3.0), int)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"min_confidence": 0.0},
        {"min_confidence": 1.5},
        {"stable_duration_s": 0.0},
        {"still_window_s": -1.0},
        # 3초 창이 1초 연속보다 짧으면 "출발까지 최소 3초"가 무너진다.
        {"stable_duration_s": 2.0, "still_window_s": 1.0},
        {"max_displacement_m": 0.0},
        # 하한이 상한보다 크면 어떤 거리도 통과하지 못한다.
        {"min_distance_m": 5.0, "max_distance_m": 4.0},
        {"detection_gap_s": 0.0},
        {"teleport_speed_mps": 0.0},
        {"max_samples_per_track": 1},
        {"max_tracks": 0},
    ],
)
def test_invalid_thresholds_are_rejected(kwargs):
    """잘못된 값은 조용히 굴러가는 것보다 기동 시점에 죽는 편이 안전하다."""
    with pytest.raises(ValueError):
        GateThresholds(**kwargs)


# ---- stable: 오탐 거르기 -----------------------------------------------------


def test_first_sample_is_not_stable():
    gate = DetectionGate()
    verdict = gate.observe(sample(0.0))
    assert verdict.stable is False
    assert verdict.approachable is False
    assert verdict.reason is GateReason.NOT_STABLE


def test_exactly_one_second_is_stable():
    """경계값은 포함이다 — 정확히 1.0초면 stable."""
    gate = DetectionGate()
    verdict = feed_still(gate, 0.0, 1.0)
    assert verdict.stable is True
    assert verdict.stable_for_ns == sec_to_ns(1.0)


def test_just_under_one_second_is_not_stable():
    gate = DetectionGate()
    feed_still(gate, 0.0, 0.8)
    verdict = gate.observe(sample(0.999))
    assert verdict.stable is False
    assert verdict.reason is GateReason.NOT_STABLE


def test_low_confidence_sample_is_not_evidence():
    gate = DetectionGate()
    verdict = gate.observe(sample(0.0, confidence=0.59))
    assert verdict.reason is GateReason.LOW_CONFIDENCE
    assert verdict.stable is False
    assert verdict.approachable is False
    # 이력이 만들어지지 않는다 — 저신뢰 스팸으로 메모리를 채우지 못한다.
    assert gate.track_ids() == ()


def test_confidence_boundary_is_inclusive():
    """정확히 0.6 은 채택한다."""
    gate = DetectionGate()
    gate.observe(sample(0.0, confidence=DEFAULT_MIN_CONFIDENCE))
    assert gate.sample_count(1) == 1


def test_single_low_confidence_flicker_does_not_reset_history():
    """한 프레임 튀었다고 3초를 다시 쌓게 하지 않는다.

    이력은 '채택된 표본'만으로 이뤄지고, 연속성은 채택 표본 사이의 간격으로만
    끊긴다. 저신뢰가 0.6초 넘게 이어지면 그때 결손으로 끊긴다.
    """
    gate = DetectionGate()
    feed_still(gate, 0.0, 3.0)
    gate.observe(sample(3.2, confidence=0.1))  # 한 프레임 튐
    verdict = gate.observe(sample(3.4))
    assert verdict.stable is True
    assert verdict.approachable is True


def test_sustained_low_confidence_breaks_the_streak():
    gate = DetectionGate()
    feed_still(gate, 0.0, 3.0)
    for i in range(1, 5):  # 0.8초 동안 저신뢰만 (gap 0.6초 초과)
        gate.observe(sample(3.0 + i * 0.2, confidence=0.2))
    verdict = gate.observe(sample(4.0))
    assert verdict.reason is GateReason.DETECTION_LOST
    assert verdict.stable is False


def test_invalid_sample_is_rejected():
    """depth 구멍은 NaN 으로 온다. 판정에 섞이면 비교가 전부 False 가 된다."""
    gate = DetectionGate()
    verdict = gate.observe(sample(0.0, distance_m=float("nan")))
    assert verdict.reason is GateReason.INVALID_SAMPLE
    assert gate.track_ids() == ()

    verdict = gate.observe(sample(0.0, x=float("inf")))
    assert verdict.reason is GateReason.INVALID_SAMPLE


# ---- approachable: 지나가는 사람 거르기 --------------------------------------


def test_three_seconds_still_in_range_is_approachable():
    gate = DetectionGate()
    verdict = feed_still(gate, 0.0, 3.0, distance_m=2.5)
    assert verdict.stable is True
    assert verdict.approachable is True
    assert verdict.reason is GateReason.OK


def test_approachable_implies_stable():
    """§8 — 신뢰도 그리고 1초 연속 그리고 3초 정지. 셋 다 AND 다."""
    gate = DetectionGate()
    for t in [i * 0.2 for i in range(16)]:
        verdict = gate.observe(sample(t))
        if verdict.approachable:
            assert verdict.stable is True


def test_walking_person_is_never_approachable():
    """1.2 m/s 로 지나가는 사람. 로봇은 이 사람을 절대 따라잡지 못한다."""
    gate = DetectionGate()
    approvals = []
    for i in range(51):  # 10초, 5 Hz
        t = i * 0.2
        verdict = gate.observe(
            sample(t, x=1.2 * t, distance_m=2.5)
        )
        approvals.append(verdict.approachable)
    assert not any(approvals)


# ---- 경계조건 4: 표본이 아직 3초어치 안 쌓였을 때 ----------------------------


def test_two_seconds_is_not_yet_approachable():
    """모르면 안 간다 — fail-safe."""
    gate = DetectionGate()
    verdict = feed_still(gate, 0.0, 2.0)
    assert verdict.stable is True          # 1초는 넘었다
    assert verdict.approachable is False
    assert verdict.reason is GateReason.INSUFFICIENT_HISTORY


def test_exactly_three_seconds_is_enough():
    """2.8초에는 안 되고 3.0초에 된다 — 경계값은 포함이다."""
    gate = DetectionGate()
    verdict = feed_still(gate, 0.0, 2.8)
    assert verdict.approachable is False
    verdict = gate.observe(sample(3.0))
    assert verdict.approachable is True


# ---- 경계조건 3: 거리 범위 경계 ----------------------------------------------


@pytest.mark.parametrize("distance_m", [1.5, 2.75, 4.0])
def test_distance_bounds_are_inclusive(distance_m):
    """1.5 m 와 4.0 m 는 범위 안이다."""
    gate = DetectionGate()
    verdict = feed_still(gate, 0.0, 3.0, distance_m=distance_m)
    assert verdict.approachable is True


def test_too_near_is_rejected():
    gate = DetectionGate()
    verdict = feed_still(gate, 0.0, 3.0, distance_m=1.49)
    assert verdict.approachable is False
    assert verdict.reason is GateReason.TOO_NEAR
    assert verdict.stable is True


def test_too_far_is_rejected():
    """4.0 m 를 넘으면 도착까지 10초를 넘겨 상황이 이미 달라진다(§6.1)."""
    gate = DetectionGate()
    verdict = feed_still(gate, 0.0, 3.0, distance_m=4.01)
    assert verdict.approachable is False
    assert verdict.reason is GateReason.TOO_FAR


def test_distance_uses_the_latest_sample():
    """범위 판정은 '지금 출발할 가치가 있는가'이므로 최신 거리로 본다."""
    gate = DetectionGate()
    feed_still(gate, 0.0, 3.0, distance_m=4.5)
    verdict = gate.observe(sample(3.2, distance_m=3.0))
    assert verdict.approachable is True


# ---- 이동량 경계 -------------------------------------------------------------


def test_displacement_at_threshold_is_moving():
    """'3초간 이동 0.3 m 미만'이므로 정확히 0.3 m 는 실패다."""
    gate = DetectionGate()
    feed_still(gate, 0.0, 2.8, x=0.0)
    verdict = gate.observe(sample(3.0, x=0.3))
    assert verdict.approachable is False
    assert verdict.reason is GateReason.MOVING
    assert verdict.displacement_m == pytest.approx(0.3)


def test_displacement_just_under_threshold_is_still():
    gate = DetectionGate()
    feed_still(gate, 0.0, 2.8, x=0.0)
    verdict = gate.observe(sample(3.0, x=0.29))
    assert verdict.approachable is True


def test_displacement_is_max_pairwise_not_endpoint_distance():
    """제자리로 돌아오는 왕복은 '정지'가 아니다.

    시작-끝만 비교하면 0.4 m 를 갔다 온 사람이 0 m 이동으로 보인다.
    """
    gate = DetectionGate()
    # 제자리 -> 0.4 m -> 제자리. 시작과 끝이 같다.
    verdict = feed_path(gate, [0.0] * 6 + [0.4] * 5 + [0.0] * 5)
    assert verdict.reason is GateReason.MOVING
    assert verdict.displacement_m == pytest.approx(0.4)


def test_window_slides_so_old_motion_stops_blocking():
    """창은 '최근 3초'다. 4초 전 이동은 더 이상 발목을 잡지 않는다."""
    gate = DetectionGate()
    gate.observe(sample(0.0, x=0.0))
    gate.observe(sample(0.2, x=0.4))   # 0.4 m 이동 (2.0 m/s, 순간이동 임계와 같다)
    assert gate.observe(sample(0.4, x=0.4)).approachable is False
    verdict = feed_still(gate, 0.6, 3.0, x=0.4)
    assert verdict.approachable is True


# ---- 경계조건 1: 탐지가 끊겼다가 다시 잡힌 경우 ------------------------------


def test_gap_beyond_threshold_restarts_the_streak():
    """연속 판정은 초기화되어야 한다 — '1.0초 연속'의 연속이 깨졌다."""
    gate = DetectionGate()
    feed_still(gate, 0.0, 3.0)
    verdict = gate.observe(sample(3.0 + DEFAULT_DETECTION_GAP_S + 0.01))
    assert verdict.reason is GateReason.DETECTION_LOST
    assert verdict.stable is False
    assert verdict.approachable is False
    assert gate.sample_count(1) == 1


def test_gap_at_threshold_keeps_the_streak():
    """경계값은 포함이다 — 정확히 0.6초 결손은 아직 연속으로 본다."""
    gate = DetectionGate()
    feed_still(gate, 0.0, 3.0)
    verdict = gate.observe(sample(3.0 + DEFAULT_DETECTION_GAP_S))
    assert verdict.stable is True
    assert verdict.approachable is True


def test_streak_rebuilds_after_a_gap():
    gate = DetectionGate()
    feed_still(gate, 0.0, 3.0)
    gate.observe(sample(5.0))              # 2초 결손 → 리셋
    verdict = feed_still(gate, 5.2, 3.0)
    assert verdict.approachable is True
    # 리셋 직후부터 다시 3초를 쌓았다.
    assert verdict.stable_for_ns >= sec_to_ns(3.0)


def test_evaluate_without_new_sample_reports_lost():
    """탐지가 끊긴 채 시간만 흐르면 stable 이 유지되면 안 된다."""
    gate = DetectionGate()
    feed_still(gate, 0.0, 3.0)
    fresh = gate.evaluate(1, sec_to_ns(3.5))
    assert fresh.approachable is True
    stale = gate.evaluate(1, sec_to_ns(4.0))
    assert stale.reason is GateReason.DETECTION_LOST
    assert stale.stable is False
    assert stale.approachable is False


def test_evaluate_unknown_track_is_none():
    gate = DetectionGate()
    assert gate.evaluate(99, sec_to_ns(1.0)) is None


def test_stable_time_does_not_grow_without_observation():
    """관측하지 않은 시간은 증거가 아니다."""
    gate = DetectionGate()
    gate.observe(sample(0.0))
    gate.observe(sample(0.4))
    verdict = gate.evaluate(1, sec_to_ns(0.9))   # 결손 임계 0.6초 안이다
    assert verdict.stable_for_ns == sec_to_ns(0.4)
    assert verdict.stable is False


# ---- 경계조건 2: 같은 track_id 인데 위치가 순간이동 --------------------------


def test_teleport_resets_history():
    """추적기가 ID 를 엉뚱한 사람에게 붙인 것으로 본다."""
    gate = DetectionGate()
    feed_still(gate, 0.0, 3.0, x=0.0)
    verdict = gate.observe(sample(3.2, x=5.0))   # 0.2초에 5 m
    assert verdict.reason is GateReason.TRACK_JUMP
    assert verdict.stable is False
    assert verdict.approachable is False
    assert gate.sample_count(1) == 1


def test_teleport_threshold_boundary():
    """0.2초에 0.4 m = 2.0 m/s 는 임계와 같다 — 아직 사람 이동으로 본다."""
    gate = DetectionGate()
    gate.observe(sample(0.0, x=0.0))
    verdict = gate.observe(sample(0.2, x=0.4))
    assert verdict.reason is not GateReason.TRACK_JUMP
    verdict = gate.observe(sample(0.4, x=0.9))   # 2.5 m/s
    assert verdict.reason is GateReason.TRACK_JUMP


def test_new_person_at_the_jumped_position_can_become_approachable():
    """리셋은 '영구 배제'가 아니라 '3초 다시 세기'다."""
    gate = DetectionGate()
    feed_still(gate, 0.0, 3.0, x=0.0)
    gate.observe(sample(3.2, x=5.0))
    verdict = feed_still(gate, 3.4, 3.0, x=5.0)
    assert verdict.approachable is True


def test_time_reversal_resets_history():
    """시계가 뒤로 뛰면 안전한 쪽으로 넘어진다(vica_safety/freshness.py 계약)."""
    gate = DetectionGate()
    feed_still(gate, 0.0, 3.0)
    verdict = gate.observe(sample(2.0))
    assert verdict.reason is GateReason.TIME_REVERSED
    assert verdict.stable is False
    assert gate.sample_count(1) == 1


def test_duplicate_stamp_with_movement_is_a_jump():
    gate = DetectionGate()
    gate.observe(sample(1.0, x=0.0))
    verdict = gate.observe(sample(1.0, x=2.0))
    assert verdict.reason is GateReason.TRACK_JUMP


# ---- 경계조건 5: 여러 track 이 동시에 approachable ---------------------------


def test_tracks_keep_independent_history():
    """여러 사람이 동시에 보인다. 한쪽이 끊겨도 다른 쪽은 그대로다."""
    gate = DetectionGate()
    for i in range(16):
        t = i * 0.2
        gate.observe(sample(t, track_id=1, x=0.0, distance_m=2.0))
        gate.observe(sample(t, track_id=2, x=1.2 * t, distance_m=3.0))
    assert gate.evaluate(1, sec_to_ns(3.0)).approachable is True
    assert gate.evaluate(2, sec_to_ns(3.0)).approachable is False


def test_module_does_not_choose_a_single_target():
    """고르는 것은 이 모듈의 책임이 아니다.

    설계 §3 — 탐지 결과는 '요청'이고 goal 권한은 Mission Manager 가 독점한다.
    재탐지 억제 60초·현재 미션 상태는 Mission 쪽 정보이므로 여기서는 자격을
    갖춘 후보를 전부 돌려준다.
    """
    gate = DetectionGate()
    for i in range(16):
        t = i * 0.2
        gate.observe(sample(t, track_id=7, distance_m=3.5))
        gate.observe(sample(t, track_id=3, distance_m=2.0))
        gate.observe(sample(t, track_id=5, distance_m=2.0))
    candidates = gate.approachable_tracks(sec_to_ns(3.0))
    assert [v.track_id for v in candidates] == [3, 5, 7]


def test_candidate_order_is_deterministic_nearest_first():
    """같은 입력에 같은 순서가 나와야 재현·시험이 된다."""
    gate = DetectionGate()
    for i in range(16):
        t = i * 0.2
        for track_id, distance_m in ((11, 3.9), (12, 1.6)):
            gate.observe(sample(t, track_id=track_id, distance_m=distance_m))
    candidates = gate.approachable_tracks(sec_to_ns(3.0))
    assert [v.track_id for v in candidates] == [12, 11]


def test_update_returns_every_live_track():
    gate = DetectionGate()
    for i in range(15):
        t = i * 0.2
        gate.observe(sample(t, track_id=1))
        gate.observe(sample(t, track_id=2, distance_m=9.0))
    verdicts = gate.update(
        [sample(3.0, track_id=1), sample(3.0, track_id=2, distance_m=9.0)],
        sec_to_ns(3.0),
    )
    assert {v.track_id for v in verdicts} == {1, 2}
    assert [v.approachable for v in verdicts] == [True, False]


# ---- 이력이 무한히 쌓이지 않는가 ---------------------------------------------


def test_samples_are_pruned_to_the_window():
    gate = DetectionGate()
    feed_still(gate, 0.0, 30.0)     # 150 프레임
    # 3초 창 + 5 Hz = 16개 안팎. 상한 64를 넘지 않는다.
    assert gate.sample_count(1) <= gate.thresholds.max_samples_per_track
    assert gate.sample_count(1) <= 20


def test_expired_tracks_are_forgotten():
    gate = DetectionGate()
    feed_still(gate, 0.0, 3.0, track_id=1)
    feed_still(gate, 0.0, 3.0, track_id=2)
    gate.prune(sec_to_ns(3.0 + gate.thresholds.track_expiry_s + 0.1))
    assert gate.track_ids() == ()


def test_track_count_is_capped():
    """추적기가 ID 를 폭주 발급해도 메모리가 무한히 늘지 않는다."""
    thresholds = GateThresholds(max_tracks=4)
    gate = DetectionGate(thresholds)
    for track_id in range(20):
        gate.observe(sample(0.1 * track_id, track_id=track_id))
    assert len(gate.track_ids()) == 4
    # 가장 오래 안 보인 track 부터 버린다.
    assert set(gate.track_ids()) == {16, 17, 18, 19}


def test_sample_overflow_is_fail_safe():
    """창 안 표본을 버려야 할 만큼 빠르면 이동량을 믿을 수 없다 → 승인 안 함."""
    thresholds = GateThresholds(max_samples_per_track=5)
    gate = DetectionGate(thresholds)
    verdict = feed_still(gate, 0.0, 3.0, period_s=0.1)
    assert verdict.approachable is False
    assert verdict.reason is GateReason.SAMPLE_OVERFLOW


def test_reset_clears_everything():
    gate = DetectionGate()
    feed_still(gate, 0.0, 3.0)
    gate.reset()
    assert gate.track_ids() == ()
    assert gate.sample_count(1) == 0


def test_forget_drops_one_track():
    gate = DetectionGate()
    feed_still(gate, 0.0, 3.0, track_id=1)
    feed_still(gate, 0.0, 3.0, track_id=2)
    gate.forget(1)
    assert gate.track_ids() == (2,)


# ---- 값 자료형 ---------------------------------------------------------------


def test_point_distance():
    assert Point2D(0.0, 0.0).distance_to(Point2D(3.0, 4.0)) == pytest.approx(5.0)


def test_verdict_is_immutable():
    """판정은 기록이다. 받은 쪽이 고쳐 쓰지 못하게 한다."""
    gate = DetectionGate()
    verdict = gate.observe(sample(0.0))
    with pytest.raises(Exception):
        verdict.approachable = True


def test_verdict_carries_diagnosis_fields():
    gate = DetectionGate()
    verdict = feed_still(gate, 0.0, 3.0, distance_m=2.2)
    assert verdict.track_id == 1
    assert verdict.confidence == pytest.approx(0.9)
    assert verdict.distance_m == pytest.approx(2.2)
    assert verdict.position == Point2D(0.0, 0.0)
    assert verdict.sample_count >= 2
    assert math.isfinite(verdict.displacement_m)
