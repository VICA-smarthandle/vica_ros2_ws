"""정지 중 자이로 편향 추정."""


class GyroBiasEstimator:
    """기동 직후 정지 구간의 각속도 평균을 편향으로 삼는다."""

    def __init__(
        self,
        sample_count: int,
        max_abs_rate: float,
        refresh_sample_count: int = 0,
        refresh_alpha: float = 0.2,
        max_abs_dev: float = 0.01,
        max_refresh_jump: float = 0.02,
    ):
        """표본 수와 정지 판정 임계(rad/s)를 받는다. sample_count 0이면 기능을 끈다."""
        self.sample_count = int(sample_count)
        self.max_abs_rate = float(max_abs_rate)
        self.refresh_sample_count = int(refresh_sample_count)
        self.refresh_alpha = float(refresh_alpha)
        self.max_abs_dev = float(max_abs_dev)
        self.max_refresh_jump = float(max_refresh_jump)

        self._sums = [0.0, 0.0, 0.0]
        self._collected = 0
        self._bias = (0.0, 0.0, 0.0)
        self._ready = False
        self._aborted = False

        self._r_sums = [0.0, 0.0, 0.0]
        self._r_first = None
        self._r_count = 0
        self._refresh_count = 0

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

    @property
    def refresh_count(self) -> int:
        """ZUPT 로 편향을 갱신한 횟수. 실주행에서 동작을 확인할 때 본다."""
        return self._refresh_count

    def add(self, gx: float, gy: float, gz: float) -> None:
        """표본을 하나 넣는다."""
        if self.sample_count <= 0:
            return
        if self._ready or self._aborted:
            self._add_refresh(gx, gy, gz)
            return

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

    def _add_refresh(self, gx: float, gy: float, gz: float) -> None:
        """정차 구간을 모아 편향을 갱신한다(ZUPT)."""
        if self.refresh_sample_count <= 0:
            return

        if max(abs(gx), abs(gy), abs(gz)) > self.max_abs_rate:
            self._reset_refresh()
            return

        if self._r_first is None:
            self._r_first = (gx, gy, gz)
        else:
            fx, fy, fz = self._r_first
            dev = max(abs(gx - fx), abs(gy - fy), abs(gz - fz))
            if dev > self.max_abs_dev:
                self._reset_refresh()
                return

        self._r_sums[0] += gx
        self._r_sums[1] += gy
        self._r_sums[2] += gz
        self._r_count += 1

        if self._r_count < self.refresh_sample_count:
            return

        n = float(self._r_count)
        mean = (self._r_sums[0] / n, self._r_sums[1] / n, self._r_sums[2] / n)
        self._reset_refresh()

        if self._ready:
            jump = max(abs(m - b) for m, b in zip(mean, self._bias))
            if jump > self.max_refresh_jump:
                return
            a = self.refresh_alpha
            self._bias = tuple(
                (1.0 - a) * b + a * m for b, m in zip(self._bias, mean)
            )
        else:
            self._bias = mean
            self._ready = True
        self._refresh_count += 1

    def _reset_refresh(self) -> None:
        """모으던 정차 구간을 버린다. 끊긴 구간을 이어붙이지 않는다."""
        self._r_sums = [0.0, 0.0, 0.0]
        self._r_first = None
        self._r_count = 0

    def correct(self, gx: float, gy: float, gz: float):
        """편향을 뺀 각속도를 돌려준다. 확정 전이면 원값 그대로다."""
        if not self._ready:
            return (gx, gy, gz)

        bx, by, bz = self._bias
        return (gx - bx, gy - by, gz - bz)
