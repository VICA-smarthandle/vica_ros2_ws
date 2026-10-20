"""미리보기 JSON 에 동봉하는 로봇 자세 필드의 규약을 고정한다."""

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
    assert math.isclose(quaternion_to_yaw_degrees(*_quat_z(90.0)), 90.0)
    assert math.isclose(quaternion_to_yaw_degrees(*_quat_z(-45.0)), -45.0)


def test_unknown_pose_adds_no_fields():
    assert robot_pose_fields(None, None) == {}
    assert robot_pose_fields(None, 0.1) == {}


def test_fresh_pose_is_rounded_like_robot_status():
    fields = robot_pose_fields((1.23456, -2.34567, 91.2345), 0.5)
    assert fields == {'robot_x': 1.235, 'robot_y': -2.346, 'robot_yaw': 91.23}


def test_stale_pose_is_dropped():
    assert robot_pose_fields((0.0, 0.0, 0.0), POSE_MAX_AGE_SEC + 0.1) == {}
    assert robot_pose_fields((0.0, 0.0, 0.0), 2.0, max_age_sec=1.0) == {}


def test_clock_going_backwards_counts_as_fresh():
    assert robot_pose_fields((0.0, 0.0, 0.0), -3.0) != {}
