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
추가하는 것이며 별도 작업이다.

ZUPT — 정차할 때마다 다시 재기 (2026-08-30)

    편향은 온도에 따라 변한다. 기동 시 한 번 잰 값이 30분 뒤에는 안 맞는다.
    그래서 주행 중 정차 구간을 찾아 조금씩 갱신한다.

    bag 실측(run1139)에서 wheel_base 를 고쳐 회전 스케일을 맞춘 뒤에도 AMCL yaw
    보정 추세가 -1.16 도/분 남았고, 그 정체가 이 편향(+0.013 도/초)이었다.
    스케일 오차는 회전한 양에, 편향은 시간에 비례하므로 둘은 따로 잡아야 한다.

    가장 큰 위험은 **직진을 정차로 착각하는 것**이다. 직진 중에도 자이로 평균은
    0 에 가까워서 크기만으로는 구별되지 않는다. 그런데 직진 중에는 마스트가
    흔들리므로 그 흔들림을 편향으로 잡으면 오히려 나빠진다. 실측이 둘을 가른다:

        정지 중   sigma 0.0018 rad/s
        직진 중   sigma 0.0939 rad/s      52배

    그래서 크기(max_abs_rate)와 함께 **흔들림 폭(max_abs_dev)** 을 본다. 구간의
    어느 표본이든 그 구간 첫 표본에서 max_abs_dev 넘게 벗어나면 정차가 아니다.

    갱신은 통째로 바꾸지 않고 alpha 만큼만 섞는다. 잘못 잰 한 회차가 전체를
    망치면 안 되기 때문이다. 옛 값과 max_refresh_jump 넘게 차이 나면 그 회차는
    아예 버린다 — 편향은 온도로 천천히 변하지 정차 한 번에 뛰지 않는다.

    실주행 검증 (run1202, 317초 · 44.4 m)

        AMCL yaw 보정 추세   -1.16 도/분  ->  +0.03 도/분
        주행 중 28회 갱신. 편향이 실제로 145 ~ 233 deg/hour 사이에서 움직였다.

    마지막 줄이 이 기능의 존재 이유다. **편향은 고정된 값이 아니다.** 기동 시
    한 번 재서 고정하면 그 순간 값에 갇히는데, 실제로는 주행 내내 60 % 폭으로
    변한다. 가장 긴 주행이었는데도 추세가 0 에 붙었다 — 편향은 시간에 비례해
    쌓이므로 길수록 잘 드러나는데 그렇지 않았다.

가장 중요한 규칙은 `aborted`다. **보정 구간에 로봇이 움직였으면 편향을 적용하지
않는다.** 틀린 상수를 빼는 것은 드리프트보다 나쁘다 — 드리프트는 느리게 쌓이지만
틀린 상수는 즉시 모든 회전을 왜곡한다.
"""


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
        """표본 수와 정지 판정 임계(rad/s)를 받는다. sample_count 0이면 기능을 끈다.

        refresh_* 는 ZUPT 용이다. refresh_sample_count 0이면 종전처럼 기동 시
        한 번만 확정하고 고정한다.

          refresh_sample_count  정차로 인정할 연속 표본 수
          refresh_alpha         새 측정을 섞는 비율 (1.0이면 통째로 교체)
          max_abs_dev           구간 안에서 허용하는 흔들림 폭 (rad/s)
          max_refresh_jump      옛 편향과의 차이가 이보다 크면 그 회차를 버린다
        """
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

        # ZUPT 구간 상태
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
        """표본을 하나 넣는다.

        기동 확정이 끝났거나 포기했으면 ZUPT 쪽으로 넘긴다.
        """
        if self.sample_count <= 0:
            return
        if self._ready or self._aborted:
            self._add_refresh(gx, gy, gz)
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

    def _add_refresh(self, gx: float, gy: float, gz: float) -> None:
        """정차 구간을 모아 편향을 갱신한다(ZUPT).

        구간이 끊기는 조건이 둘이다.
          ① 크기가 max_abs_rate 를 넘는다        — 확실히 움직인다
          ② 구간 첫 표본에서 max_abs_dev 넘게 벗어난다 — 흔들린다(직진 중)

        ②가 없으면 직진 중을 정차로 착각한다. 직진 중에도 평균은 0 에 가깝지만
        마스트 진동 때문에 폭이 50배 크다.
        """
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
            # 옛 값과 너무 다르면 정차 판정이 틀렸다고 본다. 편향은 온도로
            # 천천히 변하지 정차 한 번에 뛰지 않는다.
            jump = max(abs(m - b) for m, b in zip(mean, self._bias))
            if jump > self.max_refresh_jump:
                return
            a = self.refresh_alpha
            self._bias = tuple(
                (1.0 - a) * b + a * m for b, m in zip(self._bias, mean)
            )
        else:
            # 기동 때 못 쟀던 경우다. 첫 정차 값을 그대로 받는다.
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
