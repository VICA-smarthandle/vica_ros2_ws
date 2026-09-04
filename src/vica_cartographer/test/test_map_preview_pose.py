"""미리보기 JSON 에 동봉하는 로봇 자세 필드의 규약을 고정한다.

앱(MapPreview.fromJson)은 robot_x·robot_y·robot_yaw 를 읽고, yaw 는
/robot_status 와 같은 도 단위·반시계 양수여야 화살표 공식이 맞는다.
"""

import math

from vica_cartographer.map_preview import (
    POSE_MAX_AGE_SEC,
    quaternion_to_yaw_degrees,
    robot_pose_fields,
)


def _quat_z(degrees: float):
    half = math.radians(degrees) / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def test_identity_quaternion_is_zero_degrees():
    assert quaternion_to_yaw_degrees(0.0, 0.0, 0.0, 1.0) == 0.0


def test_yaw_is_degrees_counter_clockwise_positive():
    # 왼쪽으로 90도 돈 로봇은 +90 이어야 앱의 90 - yaw 공식이 위쪽을 가리킨다.
    assert math.isclose(quaternion_to_yaw_degrees(*_quat_z(90.0)), 90.0)
    assert math.isclose(quaternion_to_yaw_degrees(*_quat_z(-45.0)), -45.0)


def test_unknown_pose_adds_no_fields():
    assert robot_pose_fields(None, None) == {}
    assert robot_pose_fields(None, 0.1) == {}


def test_fresh_pose_is_rounded_like_robot_status():
    fields = robot_pose_fields((1.23456, -2.34567, 91.2345), 0.5)
    assert fields == {'robot_x': 1.235, 'robot_y': -2.346, 'robot_yaw': 91.23}


def test_stale_pose_is_dropped():
    # Cartographer 가 죽어 자세가 멈추면 굳은 화살표 대신 아무것도 싣지 않는다.
    assert robot_pose_fields((0.0, 0.0, 0.0), POSE_MAX_AGE_SEC + 0.1) == {}
    assert robot_pose_fields((0.0, 0.0, 0.0), 2.0, max_age_sec=1.0) == {}


def test_clock_going_backwards_counts_as_fresh():
    assert robot_pose_fields((0.0, 0.0, 0.0), -3.0) != {}
