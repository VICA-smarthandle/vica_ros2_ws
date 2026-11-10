"""person_detector_node 의 접근 conf 문턱 배선 시험.

판정 자체는 test_detection_gate 가 본다. 여기서는 노드 파라미터 이름이 감지
관문 문턱으로 제대로 옮겨지는지만 본다. rclpy 노드를 만들고 지우는 시험은
destroy_node 에서 가끔 멈춰서(2026-11-10, 약 1/6) 노드 없이 확인한다.
"""
from vica_perception.detection_gate import (
    DEFAULT_MAX_DISTANCE_M,
    DEFAULT_MIN_CONFIDENCE,
    DEFAULT_MIN_DISTANCE_M,
)
from vica_perception.person_detector_node import (
    APPROACH_PARAM_DEFAULTS,
    DEFAULT_APPROACH_MIN_CONFIDENCE,
    approach_gate_thresholds,
)


def test_default_approach_conf_is_0_8():
    """2026-11-10 평가셋 재생: 0.8 에서 진짜 1.8~7.2 m 9/9 유지, 오탐 장면 7→3."""
    assert DEFAULT_APPROACH_MIN_CONFIDENCE == 0.8
    assert APPROACH_PARAM_DEFAULTS["approach_min_confidence"] == 0.8


def test_gate_constant_is_left_alone():
    """detection_gate 의 상수는 그대로 두고 노드 파라미터로 올린다(그 파일 주석의 규칙)."""
    assert DEFAULT_MIN_CONFIDENCE == 0.6
    assert DEFAULT_APPROACH_MIN_CONFIDENCE > DEFAULT_MIN_CONFIDENCE


def test_distance_defaults_unchanged():
    assert APPROACH_PARAM_DEFAULTS["approach_min_distance_m"] == DEFAULT_MIN_DISTANCE_M
    assert APPROACH_PARAM_DEFAULTS["approach_max_distance_m"] == DEFAULT_MAX_DISTANCE_M


def test_defaults_reach_gate():
    t = approach_gate_thresholds(APPROACH_PARAM_DEFAULTS.__getitem__)
    assert t.min_confidence == 0.8
    assert t.min_distance_m == DEFAULT_MIN_DISTANCE_M
    assert t.max_distance_m == DEFAULT_MAX_DISTANCE_M


def test_each_param_goes_to_its_own_field():
    """이름이 엇갈려 들어가지 않는지 — 세 값을 모두 다르게 준다."""
    params = {
        "approach_min_confidence": 0.7,
        "approach_min_distance_m": 2.0,
        "approach_max_distance_m": 6.5,
    }
    t = approach_gate_thresholds(params.__getitem__)
    assert (t.min_confidence, t.min_distance_m, t.max_distance_m) == (0.7, 2.0, 6.5)
