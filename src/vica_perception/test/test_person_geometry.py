"""person_geometry 순수 함수 시험.

노드 배선은 여기서 다루지 않는다 — 실기(카메라·TF)가 필요한 부분은
컨테이너에서 수동 확인한다. 여기서는 거리·투영 계산이 계약을 지키는지만 본다:

    - distance 를 못 재면 NaN 이어야 한다. 0.0 은 "거리 0 m"로 오독된다(msg 주석).
    - 몸통 창은 bbox 가장자리(지팡이·팔)를 배제해야 한다(설계 §12).
"""
import math

import numpy as np
import pytest

from vica_perception.person_geometry import (
    bbox_center,
    body_depth_median_m,
    pixel_to_camera,
)


def _depth(h=100, w=100, fill=2.0):
    return np.full((h, w), fill, dtype=np.float32)


class TestBodyDepthMedian:
    def test_균일한_depth_는_그_값이_중앙값이다(self):
        assert body_depth_median_m(_depth(fill=2.5), (10, 10, 90, 90)) == pytest.approx(2.5)

    def test_측정실패_0_픽셀은_표본에서_빠진다(self):
        d = _depth(fill=3.0)
        d[30:50, 30:50] = 0.0                    # RealSense 의 측정 실패 값
        assert body_depth_median_m(d, (10, 10, 90, 90)) == pytest.approx(3.0)

    def test_전부_무효면_NaN_이다_0이_아니라(self):
        d = _depth(fill=0.0)
        out = body_depth_median_m(d, (10, 10, 90, 90))
        assert math.isnan(out)                   # 0.0 반환은 계약 위반이다

    def test_bbox_가장자리의_지팡이는_중앙값에_안_섞인다(self):
        # 몸통(가운데)은 2.0 m, 왼쪽 가장자리 세로줄(지팡이)은 0.5 m.
        d = _depth(fill=2.0)
        d[:, 10:14] = 0.5
        out = body_depth_median_m(d, (10, 10, 90, 90))
        assert out == pytest.approx(2.0)         # 가장자리 30 % 는 창 밖이다

    def test_화면_밖으로_나간_bbox_도_안전하다(self):
        out = body_depth_median_m(_depth(), (-50, -50, 20, 20))
        assert math.isnan(out) or out > 0        # 예외 없이 값 또는 NaN

    def test_넓이가_0_인_bbox_는_NaN(self):
        assert math.isnan(body_depth_median_m(_depth(), (40, 40, 40, 80)))


class TestPixelToCamera:
    K = dict(fx=380.0, fy=380.0, cx=320.0, cy=240.0)

    def test_광학중심_픽셀은_정면_z_축_위다(self):
        out = pixel_to_camera(320.0, 240.0, 2.0, **self.K)
        assert out == pytest.approx((0.0, 0.0, 2.0))

    def test_오른쪽_픽셀은_x_양수다(self):
        x, y, z = pixel_to_camera(700.0, 240.0, 2.0, **self.K)
        assert x == pytest.approx((700 - 320) * 2.0 / 380.0)
        assert x > 0 and y == pytest.approx(0.0) and z == 2.0

    def test_무효_depth_는_None_이다_좌표를_지어내지_않는다(self):
        assert pixel_to_camera(320, 240, float("nan"), **self.K) is None
        assert pixel_to_camera(320, 240, 0.0, **self.K) is None
        assert pixel_to_camera(320, 240, -1.0, **self.K) is None

    def test_초점거리_0_은_None(self):
        assert pixel_to_camera(320, 240, 2.0, fx=0.0, fy=380.0, cx=320, cy=240) is None


def test_bbox_center():
    assert bbox_center((10.0, 20.0, 30.0, 60.0)) == (20.0, 40.0)
