"""정지 상태 `/odom` yaw 드리프트 측정 도구.

**로봇을 움직이지 않는다.** `/odom`을 구독만 하며 어떤 명령도 발행하지 않는다.
`/cmd_vel*`도, Nav2 goal도 보내지 않는다.

회전 임계값(기본 25°)이 실주행에서 안전한지 판단하려면 "yaw가 얼마나 흔들리나"
보다 **"정지 상태에서 회전 오탐이 나는가"** 를 직접 재는 편이 낫다. 그래서
`turn_detector.TurnDetector`를 그대로 써서 실제 판정 경로로 측정한다. 로직을
복제하면 실제 노드와 달라져 측정 결과가 무의미해진다.

사용법 (Jetson에서):

    ros2 launch vica_localization wheel_ekf.launch.py     # 다른 터미널
    ros2 run vica_user_guidance yaw_drift_check --ros-args -p duration_sec:=300.0

**측정 중 로봇을 건드리지 말 것.** 사람이 기대거나 바닥이 흔들리면 IMU가 반응해
실제 드리프트보다 나쁘게 나온다.
"""

import math
import sys
from typing import List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

from .timebase import sec_to_ns
from .turn_detector import (
    DIRECTION_LEFT,
    DIRECTION_NONE,
    DIRECTION_RIGHT,
    TurnDetector,
    yaw_from_quaternion,
)


class YawDriftCheck(Node):
    """정지 상태에서 yaw 드리프트와 회전 오탐을 측정한다."""

    def __init__(self) -> None:
        super().__init__("yaw_drift_check")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("duration_sec", 300.0)
        self.declare_parameter("evaluate_rate_hz", 20.0)
        self.declare_parameter("csv_path", "")
        # /odom이 한 번도 오지 않으면 측정 시작 시각이 잡히지 않아 종료 조건에
        # 영원히 도달하지 못한다. 원인 설명 없이 매달리는 것을 막는다.
        self.declare_parameter("startup_timeout_sec", 15.0)

        # turn_guide_node와 같은 기본값이어야 실제 판정을 재현한다.
        self.declare_parameter("window_sec", 1.5)
        self.declare_parameter("enter_threshold_deg", 25.0)
        self.declare_parameter("exit_threshold_deg", 10.0)
        self.declare_parameter("min_duration_sec", 0.6)
        self.declare_parameter("odom_timeout_sec", 0.5)

        self.enter_deg = float(self.get_parameter("enter_threshold_deg").value)
        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.csv_path = str(self.get_parameter("csv_path").value)

        self.detector = TurnDetector(
            window_ns=sec_to_ns(float(self.get_parameter("window_sec").value)),
            enter_threshold_rad=math.radians(self.enter_deg),
            exit_threshold_rad=math.radians(
                float(self.get_parameter("exit_threshold_deg").value)
            ),
            min_duration_ns=sec_to_ns(
                float(self.get_parameter("min_duration_sec").value)
            ),
            odom_timeout_ns=sec_to_ns(
                float(self.get_parameter("odom_timeout_sec").value)
            ),
        )

        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.startup_timeout_ns = sec_to_ns(
            float(self.get_parameter("startup_timeout_sec").value)
        )
        self.node_start_ns = self.steady_clock.now().nanoseconds

        self.start_ns: Optional[int] = None
        self.first_yaw: Optional[float] = None
        self.last_yaw: Optional[float] = None
        self.unwrapped_deg: float = 0.0
        self.min_deg = 0.0
        self.max_deg = 0.0
        self.odom_count = 0
        self.max_window_accum_deg = 0.0
        self.false_left = 0
        self.false_right = 0
        self.prev_direction = DIRECTION_NONE
        self.series: List[Tuple[float, float]] = []   # (경과초, 펼친 yaw deg)
        self.done = False

        topic = str(self.get_parameter("odom_topic").value)
        self.create_subscription(Odometry, topic, self.cb_odom, 50)
        self.create_timer(
            1.0 / float(self.get_parameter("evaluate_rate_hz").value),
            self.tick,
            clock=self.steady_clock,
        )

        self.get_logger().info(
            f"구독: {topic} — 이 노드는 어떤 명령도 발행하지 않습니다."
        )
        self.get_logger().info(
            f"측정 시간 {self.duration_sec:.0f}초. 로봇을 건드리지 마세요."
        )

    def now_ns(self) -> int:
        return self.steady_clock.now().nanoseconds

    def cb_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        now = self.now_ns()

        if self.start_ns is None:
            self.start_ns = now
            self.first_yaw = yaw
        else:
            # ±pi 경계에서 튀지 않도록 연속 샘플 차이를 정규화해 누적한다.
            delta = yaw - self.last_yaw
            delta = math.atan2(math.sin(delta), math.cos(delta))
            self.unwrapped_deg += math.degrees(delta)

        self.last_yaw = yaw
        self.odom_count += 1
        self.min_deg = min(self.min_deg, self.unwrapped_deg)
        self.max_deg = max(self.max_deg, self.unwrapped_deg)
        self.series.append(((now - self.start_ns) / 1e9, self.unwrapped_deg))

        self.detector.add_odom(yaw, now)

    def tick(self) -> None:
        now = self.now_ns()

        if self.start_ns is None:
            if now - self.node_start_ns >= self.startup_timeout_ns:
                self.done = True   # report()가 원인을 안내한다
            return

        decision = self.detector.evaluate(now)

        # 회전 판정이 실제로 비교하는 값. "임계값에 얼마나 근접했나"를 답한다.
        accum_deg = abs(math.degrees(self.detector._window_accumulation()))
        self.max_window_accum_deg = max(self.max_window_accum_deg, accum_deg)

        # 전이 순간만 센다. 유지되는 동안 매 tick 세면 횟수가 아니라 시간이 된다.
        if decision.direction != self.prev_direction:
            if decision.direction == DIRECTION_LEFT:
                self.false_left += 1
                self.get_logger().warn(f"오탐 LEFT — 창 누적 {accum_deg:.2f}°")
            elif decision.direction == DIRECTION_RIGHT:
                self.false_right += 1
                self.get_logger().warn(f"오탐 RIGHT — 창 누적 {accum_deg:.2f}°")
            self.prev_direction = decision.direction

        elapsed = (now - self.start_ns) / 1e9
        if elapsed >= self.duration_sec:
            self.done = True

    # ── 보고 ───────────────────────────────────────────

    def report(self) -> None:
        if self.start_ns is None or self.odom_count < 2:
            print()
            print("측정 실패: /odom을 받지 못했습니다.")
            print("  ros2 topic hz /odom 으로 발행 여부를 먼저 확인하세요.")
            print("  EKF가 떠 있어도 /imu/base_link 나 /wheel/odom 이 없으면")
            print("  /odom이 나오지 않습니다.")
            return

        elapsed = self.series[-1][0]
        rate = self.odom_count / elapsed if elapsed > 0 else 0.0
        drift = self.unwrapped_deg
        per_min = drift / (elapsed / 60.0) if elapsed > 0 else 0.0
        margin = (
            self.enter_deg / self.max_window_accum_deg
            if self.max_window_accum_deg > 1e-9
            else float("inf")
        )
        false_total = self.false_left + self.false_right

        lines = [
            "",
            "=" * 58,
            "정지 상태 yaw 드리프트 측정 결과",
            "=" * 58,
            f"측정 시간   : {elapsed:.1f}초",
            f"/odom 샘플  : {self.odom_count}개 ({rate:.1f} Hz)",
            "",
            "[원시 yaw]",
            f"  총 드리프트 : {drift:+.3f}°  ({per_min:+.3f}°/분)",
            f"  최소/최대   : {self.min_deg:+.3f}° / {self.max_deg:+.3f}°",
            "",
            "[회전 판정이 실제로 비교하는 값]",
            f"  창 누적 |Δyaw| 최대 : {self.max_window_accum_deg:.3f}°",
            f"  진입 임계값         : {self.enter_deg:.1f}°",
            f"  여유                : {margin:.1f}배" if margin != float("inf")
            else "  여유                : (누적 0)",
            "",
            "[정지 상태 오탐]",
            f"  LEFT  : {self.false_left}회",
            f"  RIGHT : {self.false_right}회",
            "",
        ]

        if false_total > 0:
            lines += [
                "판정: 위험 — 정지 상태에서 회전 오탐이 발생했다.",
                "",
                "  임계값을 올리기 전에 EKF 설정을 먼저 확인한다.",
                "  vica_localization/config/ekf.yaml 의 imu0 융합이 원인일 수 있다.",
                "  AGENTS.md 6장이 D455 IMU 융합을 [미검증]으로 규정한다.",
            ]
        elif margin < 3.0:
            lines += [
                "판정: 주의 — 오탐은 없으나 임계값까지 여유가 3배 미만이다.",
                "",
                "  실주행에서는 진동이 더해지므로 오탐 가능성이 있다.",
                "  Phase 5b에서 임계값을 재검토한다.",
            ]
        else:
            lines += [
                "판정: 안전 — 정지 상태에서 오탐이 없고 임계값까지 여유가 충분하다.",
                "",
                "  단 이것은 정지 상태 결과다. 실주행 진동은 별도로 확인해야 한다.",
            ]

        lines.append("=" * 58)
        text = "\n".join(lines)
        print(text)

        if self.csv_path:
            try:
                with open(self.csv_path, "w", encoding="utf-8") as fh:
                    fh.write("elapsed_sec,unwrapped_yaw_deg\n")
                    for t, y in self.series:
                        fh.write(f"{t:.3f},{y:.6f}\n")
                print(f"\n원시 데이터: {self.csv_path}")
            except OSError as exc:
                print(f"\nCSV 저장 실패: {exc}")


def main(args=None) -> int:
    rclpy.init(args=args)
    node = YawDriftCheck()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\n중단됨 — 그때까지의 결과를 보고합니다.")
    finally:
        node.report()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
