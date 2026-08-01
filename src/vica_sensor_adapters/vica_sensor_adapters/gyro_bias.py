"""정지 중 자이로 편향 추정.

왜 필요한가 (2026-08-01 Jetson 실측):

    정지 상태 30초 측정에서 base_link 기준 gyro.z 평균이 +0.000917 rad/s였다.
    시간당 189°에 해당한다. 실제 EKF `/odom` yaw는 시간당 161° 드리프트했다.

    `ekf.yaml`의 두 입력이 모두 각'속도'만 준다. `/wheel/odom`은 vx·vy·vyaw,
    `/imu/base_link`는 vyaw다. **각도 그 자체를 주는 입력이 하나도 없다.**
    각속도를 적분해 각도를 만들므로 아무리 작은 편향도 시간에 비례해 무한히 쌓인다.

    nvblox는 `odom` 좌표계에서 동작하므로 이 드리프트가 3D 장애물 위치를 직접
    왜곡한다. 3 m 앞 벽이 yaw 오차 11°에서 약 0.57 m 어긋난다.

이 모듈은 증상 완화다. 구조적 해결은 절대 각도를 주는 입력(VSLAM 등)을 EKF에
추가하는 것이며 별도 작업이다. 편향은 온도에 따라 변하므로 기동 시 추정만으로는
완전히 없앨 수 없다.

가장 중요한 규칙은 `aborted`다. **보정 구간에 로봇이 움직였으면 편향을 적용하지
않는다.** 틀린 상수를 빼는 것은 드리프트보다 나쁘다 — 드리프트는 느리게 쌓이지만
틀린 상수는 즉시 모든 회전을 왜곡한다.
"""


class GyroBiasEstimator:
    """기동 직후 정지 구간의 각속도 평균을 편향으로 삼는다."""

    def __init__(self, sample_count: int, max_abs_rate: float):
        """표본 수와 정지 판정 임계(rad/s)를 받는다. sample_count 0이면 기능을 끈다."""
        self.sample_count = int(sample_count)
        self.max_abs_rate = float(max_abs_rate)

        self._sums = [0.0, 0.0, 0.0]
        self._collected = 0
        self._bias = (0.0, 0.0, 0.0)
        self._ready = False
        self._aborted = False

    @property
    def ready(self) -> bool:
        """편향이 확정되어 보정에 쓸 수 있는가."""
        return self._ready

    @property
    def aborted(self) -> bool:
        """보정 구간에 움직임이 감지되어 포기했는가."""
        return self._aborted

    @property
    def bias(self):
        """축별 편향 (rad/s). 확정 전에는 0이다."""
        return self._bias

    @property
    def collected(self) -> int:
        """지금까지 모은 표본 수."""
        return self._collected

    def add(self, gx: float, gy: float, gz: float) -> None:
        """표본을 하나 넣는다. 확정·포기 후에는 아무 일도 하지 않는다."""
        if self.sample_count <= 0 or self._ready or self._aborted:
            return

        # 세 축 중 하나라도 임계를 넘으면 정지 상태가 아니다.
        if max(abs(gx), abs(gy), abs(gz)) > self.max_abs_rate:
            self._aborted = True
            return

        self._sums[0] += gx
        self._sums[1] += gy
        self._sums[2] += gz
        self._collected += 1

        if self._collected >= self.sample_count:
            n = float(self._collected)
            self._bias = (
                self._sums[0] / n,
                self._sums[1] / n,
                self._sums[2] / n,
            )
            self._ready = True

    def correct(self, gx: float, gy: float, gz: float):
        """편향을 뺀 각속도를 돌려준다. 확정 전이면 원값 그대로다."""
        if not self._ready:
            return (gx, gy, gz)

        bx, by, bz = self._bias
        return (gx - bx, gy - by, gz - bz)
