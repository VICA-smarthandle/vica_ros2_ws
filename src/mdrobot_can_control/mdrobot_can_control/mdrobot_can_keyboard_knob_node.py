#!/usr/bin/env python3
import math
import time
import can

from diagnostic_msgs.msg import DiagnosticStatus

from diagnostic_updater import DiagnosticStatusWrapper, Updater

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import Twist

from std_msgs.msg import Bool

from .can_link import CanLink
from .can_preflight import require_can_interface_up
from .freshness import is_fresh_ns, sec_to_ns
from .motor_watchdog import motor_speed_ratio


# ============================================================
# ROS2 Humble + teleop_twist_keyboard + MDROBOT CAN 통합
#
# 목적:
#   safety_supervisor_node가 승인한 /cmd_vel_safe를 받아서 CAN으로 모터드라이버 구동
#   최고속도는 드라이버에서 올라오는 knob1 값으로 제한
#
# 모터 매핑:
#   MOT1 = 오른쪽 바퀴
#   MOT2 = 왼쪽 바퀴
#
# 주의:
#   왼쪽 바퀴가 MOT2라는 기존 조건 반영
# ============================================================


# CAN 경로에서 잡아야 하는 예외들.
# 닫힌 socketcan 소켓은 CanError도 OSError도 아닌 ValueError
# ('file descriptor cannot be a negative integer (-1)')를 던진다.
# 이를 빠뜨리면 예외가 타이머 콜백 밖으로 탈출해 노드가 죽는다.
CAN_FAILURES = (can.CanError, OSError, ValueError)


# ====== MDROBOT CAN 프로토콜 ======
PID_PNT_IO_MONITOR = 0xF1  # 241
PID_PNT_VEL_CMD    = 0xCF  # 207
PID_COMMAND        = 0x0A  # 10
CMD_PNT_IO_MON_ON  = 0x55  # 85

RET_TYPE_NONE = 0
RET_TYPE_ODOM = 5


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def le_i16_signed(value: int):
    value = int(value)
    value = clamp(value, -32768, 32767)
    u = value & 0xFFFF
    return u & 0xFF, (u >> 8) & 0xFF


class MdrobotCanKeyboardKnobNode(Node):
    def __init__(self):
        super().__init__("mdrobot_can_keyboard_knob_node")

        # =========================
        # 사용자 파라미터
        # =========================
        self.declare_parameter("can_iface", "can1")
        self.declare_parameter("driver_id", 0x001)

        # MDH100 Ø130mm → radius 0.065m
        self.declare_parameter("wheel_radius_m", 0.065)

        # 좌우 바퀴 중심거리. 실제 로봇에서 반드시 줄자로 재서 수정.
        self.declare_parameter("wheel_base_m", 0.37)

        # knob 100%일 때 허용할 최고 속도
        self.declare_parameter("max_linear_mps", 1.0)
        self.declare_parameter("max_angular_radps", 2.0)

        # 모터 자체 rpm 제한
        self.declare_parameter("max_rpm", 400)

        # CAN 송신 주기
        # 30Hz는 모터 버스에 너무 잦을 수 있습니다. 낮은 주기로 CAN 부하를 줄입니다.
        self.declare_parameter("send_hz", 30.0)

        # 동일한 명령을 반복 전송하지 않도록 하는 최소 재전송 간격(초)
        # 너무 길게 설정되어 있으면 제어가 뚝뚝 끊깁니다. 기본은 50ms.
        self.declare_parameter("resend_interval_sec", 0.05)

        # knob 값 0~100 중 이 값 이하는 정지로 처리
        self.declare_parameter("deadzone_pct", 5)

        # knob 패킷이 이 시간 이상 안 들어오면 안전 정지
        self.declare_parameter("knob_timeout_sec", 0.8)

        # /cmd_vel_safe가 이 시간 이상 안 들어오면 안전 정지
        self.declare_parameter("cmd_timeout_sec", 0.5)

        # 방향 보정. 전진 명령에서 바퀴가 반대로 돌면 True로 바꾸세요.
        self.declare_parameter("invert_mot1", False)  # 오른쪽 바퀴
        self.declare_parameter("invert_mot2", False)  # 왼쪽 바퀴

        # 낮은 rpm에서 모터가 꿈틀거리기만 하면 30~50 정도로 사용
        # 처음에는 0 추천
        self.declare_parameter("min_rpm_when_moving", 0)

        # CAN 실패 후 재연결을 시도하는 최소 간격(초)
        self.declare_parameter('can_reconnect_interval_sec', 1.0)

        # =========================
        # 파라미터 불러오기
        # =========================
        self.can_iface = self.get_parameter("can_iface").value
        self.driver_id = int(self.get_parameter("driver_id").value)

        self.wheel_radius_m = float(self.get_parameter("wheel_radius_m").value)
        self.wheel_base_m = float(self.get_parameter("wheel_base_m").value)

        self.max_linear_mps = float(self.get_parameter("max_linear_mps").value)
        self.max_angular_radps = float(self.get_parameter("max_angular_radps").value)
        self.max_rpm = int(self.get_parameter("max_rpm").value)

        self.send_hz = float(self.get_parameter("send_hz").value)
        self.resend_interval_sec = float(self.get_parameter("resend_interval_sec").value)
        self.deadzone_pct = int(self.get_parameter("deadzone_pct").value)
        self.knob_timeout_sec = float(self.get_parameter("knob_timeout_sec").value)
        self.cmd_timeout_sec = float(self.get_parameter("cmd_timeout_sec").value)

        # 최종 구동단 watchdog은 단일 STEADY_TIME clock과 정수 나노초를 쓴다.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.knob_timeout_ns = sec_to_ns(self.knob_timeout_sec)
        self.cmd_timeout_ns = sec_to_ns(self.cmd_timeout_sec)
        self.resend_interval_ns = sec_to_ns(self.resend_interval_sec)

        self.invert_mot1 = bool(self.get_parameter("invert_mot1").value)
        self.invert_mot2 = bool(self.get_parameter("invert_mot2").value)

        self.min_rpm_when_moving = int(
            self.get_parameter("min_rpm_when_moving").value
        )

        self.can_reconnect_interval_sec = float(
            self.get_parameter('can_reconnect_interval_sec').value
        )
        self.can_reconnect_interval_ns = sec_to_ns(
            self.can_reconnect_interval_sec
        )

        try:
            can_flags = require_can_interface_up(self.can_iface)
        except RuntimeError as exc:
            self.get_logger().fatal(str(exc))
            raise
        self.get_logger().info(
            f'CAN preflight passed: {self.can_iface} IFF_UP '
            f'flags=0x{can_flags:X}'
        )

        # =========================
        # 실행 상태
        # =========================
        self.knob1 = 0
        self.knob2 = 0
        self.last_knob_ns = None

        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        self.last_cmd_ns = None

        self.last_print_ns = None
        self.prev_rpm_mot1 = None
        self.prev_rpm_mot2 = None
        self.last_send_ns = None
        self.last_can_error_log_ns = None

        # =========================
        # CAN 초기화
        # =========================
        self.bus = can.interface.Bus(
            channel=self.can_iface,
            interface="socketcan"
        )

        self.can_link = CanLink(
            retry_interval_ns=self.can_reconnect_interval_ns
        )

        self.get_logger().info(f"CAN opened: {self.can_iface}")
        self.get_logger().info(f"Driver ID: 0x{self.driver_id:03X}")

        # 드라이버 PNT I/O monitor broadcasting ON
        self.send_pnt_io_monitor_on()
        time.sleep(0.05)
        self.send_pnt_io_monitor_on()

        # =========================
        # ROS 인터페이스
        # =========================
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            "/cmd_vel_safe",
            self.cmd_vel_callback,
            10
        )

        self.pub_can_ok = self.create_publisher(Bool, '/motor/can_ok', 10)

        self.diag_updater = Updater(self)
        self.diag_updater.setHardwareID(self.can_iface)
        self.diag_updater.add('CAN link', self.diagnose_can_link)

        self.timer = self.create_timer(
            1.0 / self.send_hz,
            self.control_loop,
            clock=self.steady_clock
        )

        self.get_logger().info("Subscribed: /cmd_vel_safe")
        self.get_logger().info("knob1 = 최고속도 제한기")
        self.get_logger().info("Ready.")

    def now_ns(self) -> int:
        """Return the current STEADY_TIME instant as integer nanoseconds."""
        return self.steady_clock.now().nanoseconds

    def log_can_error_throttled(self, phase: str, exc: BaseException) -> None:
        """Report a CAN failure at most once per reconnect interval."""
        now = self.now_ns()
        last_log_ns = self.last_can_error_log_ns
        due = (
            last_log_ns is None or
            (now - last_log_ns) >= self.can_reconnect_interval_ns
        )
        if due:
            self.get_logger().error(
                f'[CAN FAULT] phase={phase} iface={self.can_iface} '
                f'error={exc}; 출력을 0으로 유지합니다'
            )
            self.last_can_error_log_ns = now

    def diagnose_can_link(
        self,
        stat: DiagnosticStatusWrapper,
    ) -> DiagnosticStatusWrapper:
        """Report CAN link health for operators.

        초안 3.1에 따라 보고 전용이다. 정지는 control_loop이 즉시 수행하며
        이 진단이 늦거나 실패해도 정지에는 영향이 없다.
        """
        now = self.now_ns()
        if self.can_link.is_ok():
            stat.summary(DiagnosticStatus.OK, 'CAN link OK')
        else:
            stat.summary(
                DiagnosticStatus.ERROR,
                'CAN link FAILED; motor output forced to 0',
            )
        stat.add('iface', self.can_iface)
        # last_error는 현재 오류가 아니라 "마지막으로 관측된" 오류다.
        # record_success()가 이를 지우지 않으므로 CAN 복구 후에도 남는다.
        stat.add(
            'last_error',
            f'last observed (may predate recovery): '
            f'{self.can_link.last_error}',
        )
        stat.add('knob_age_sec', self.age_text(self.last_knob_ns, now))
        stat.add('cmd_age_sec', self.age_text(self.last_cmd_ns, now))
        return stat

    @staticmethod
    def age_text(last_ns, now_ns: int) -> str:
        """Render an age in seconds, or 'never' when nothing arrived yet."""
        if last_ns is None:
            return 'never'
        return f'{(now_ns - last_ns) / 1e9:.3f}'

    def try_reconnect_can(self, now_ns: int) -> None:
        """Reopen the CAN bus while the link is failed.

        걸쇠는 vica_safety가 소유하므로 재연결에 성공해도 주행이 스스로
        재개되지 않는다. 재연결이 없으면 `/motor/can_ok`가 복구되지 않아
        관리자 reset이 영원히 거부된다.

        재개방이 실패하면 `self.bus`는 반드시 None으로 남긴다. 닫힌 핸들을
        남기면 같은 사이클의 recv·send가 `ValueError`를 던져 노드가 죽는다.
        """
        if not self.can_link.should_retry(now_ns):
            return
        self.can_link.mark_retry_attempted(now_ns)
        old_bus, self.bus = self.bus, None
        try:
            if old_bus is not None:
                old_bus.shutdown()
        except Exception:  # noqa: BLE001 - 종료 실패는 재개방을 막지 않는다
            pass
        try:
            bus = can.interface.Bus(
                channel=self.can_iface,
                interface='socketcan'
            )
        except CAN_FAILURES as exc:
            self.can_link.record_error(exc, now_ns)
            return
        self.bus = bus
        try:
            self.send_pnt_io_monitor_on()
        except CAN_FAILURES as exc:
            self.can_link.record_error(exc, now_ns)
            return
        self.can_link.record_success()
        self.get_logger().info(
            f'[CAN RECOVERED] iface={self.can_iface}; '
            '주행 재개는 관리자 reset 이후에만 가능합니다'
        )

    # ============================================================
    # CAN 함수
    # ============================================================
    def send_pnt_io_monitor_on(self):
        # 예외를 삼키면 안 된다. socketcan은 상대 드라이버가 없어도 bind에
        # 성공하므로, 이 프로브의 예외가 try_reconnect_can이 복구 여부를
        # 판정하는 유일한 근거다. 감싸면 /motor/can_ok가 거짓으로 true가 되어
        # 구동단이 죽은 채로 관리자 reset이 수락된다.
        msg = can.Message(
            arbitration_id=self.driver_id,
            is_extended_id=False,
            data=bytes([
                PID_COMMAND,
                CMD_PNT_IO_MON_ON,
                0, 0, 0, 0, 0, 0
            ])
        )
        self.bus.send(msg)

    def send_vel_cmd(self, rpm1: int, rpm2: int, ret_type: int = RET_TYPE_NONE):
        """
        rpm1 = MOT1 = 오른쪽 바퀴
        rpm2 = MOT2 = 왼쪽 바퀴
        """
        rpm1 = int(clamp(rpm1, -self.max_rpm, self.max_rpm))
        rpm2 = int(clamp(rpm2, -self.max_rpm, self.max_rpm))

        lo1, hi1 = le_i16_signed(rpm1)
        lo2, hi2 = le_i16_signed(rpm2)

        data = bytes([
            PID_PNT_VEL_CMD,
            1,
            lo1,
            hi1,
            1,
            lo2,
            hi2,
            ret_type & 0xFF
        ])

        msg = can.Message(
            arbitration_id=self.driver_id,
            is_extended_id=False,
            data=data
        )

        if self.bus is None:
            return

        try:
            self.bus.send(msg)
        except CAN_FAILURES as exc:
            self.can_link.record_error(exc, self.now_ns())
            self.log_can_error_throttled('send', exc)

    def drain_can_rx(self, now_ns: int):
        """
        F1 monitor packet에서 knob1, knob2 값 읽기.

        기존 사용 코드 기준:
          d[0] == 0xF1
          d[1] == 0
          d[6] = knob1, 0~100
          d[7] = knob2, 0~100

        stamp는 호출자가 넘긴 사이클 기준 시각(`now_ns`)을 그대로 쓴다. 여기서
        시각을 다시 조회하면 판정 기준 `now`보다 나중이 되어, 방금 정상 수신한
        knob이 음수 age(시간 역전)로 stale 판정된다.
        """
        if self.bus is None:
            return

        try:
            # 버스 혼잡을 줄이기 위해 사이클당 읽는 CAN 프레임 수를 제한합니다.
            for _ in range(50):
                msg = self.bus.recv(timeout=0.0)
                if msg is None:
                    break

                d = msg.data
                if len(d) == 8 and d[0] == PID_PNT_IO_MONITOR:
                    if d[1] == 0:
                        self.knob1 = clamp(int(d[6]), 0, 100)
                        self.knob2 = clamp(int(d[7]), 0, 100)
                        self.last_knob_ns = now_ns
        except CAN_FAILURES as exc:
            self.can_link.record_error(exc, now_ns)
            self.log_can_error_throttled('recv', exc)

    # ============================================================
    # ROS 콜백
    # ============================================================
    def cmd_vel_callback(self, msg: Twist):
        self.cmd_linear_x = float(msg.linear.x)
        self.cmd_angular_z = float(msg.angular.z)
        self.last_cmd_ns = self.now_ns()

    # ============================================================
    # 메인 루프
    # ============================================================
    def control_loop(self):
        now = self.now_ns()

        self.drain_can_rx(now)

        # 재연결 시도보다 먼저 발행한다. 뒤에 두면 재연결 간격이 0인 설정에서
        # 장애와 복구가 같은 사이클에 일어나 false가 한 번도 나가지 않고,
        # 중앙 걸쇠가 걸리지 않은 채 주행이 이어진다. 복구 보고가 한 사이클
        # 늦어질 뿐이고, 장애는 반드시 최소 한 번 false로 보고된다.
        can_ok = self.can_link.is_ok()
        can_ok_msg = Bool()
        can_ok_msg.data = can_ok
        self.pub_can_ok.publish(can_ok_msg)

        if not can_ok:
            self.try_reconnect_can(now)

        # -------------------------
        # cmd·knob 신선도 판정 (단일 STEADY_TIME clock)
        # cmd 또는 knob 중 하나라도 stale·시간역전·미수신이면 0.0 → 정지
        # -------------------------
        speed_ratio = motor_speed_ratio(
            cmd_last_ns=self.last_cmd_ns,
            knob_last_ns=self.last_knob_ns,
            knob_pct=int(self.knob1),
            now_ns=now,
            cmd_timeout_ns=self.cmd_timeout_ns,
            knob_timeout_ns=self.knob_timeout_ns,
            deadzone_pct=self.deadzone_pct,
        )

        # CAN 링크가 비정상이면 cmd·knob 판정과 무관하게 0으로 막는다.
        # 정지는 여기서 즉시 이루어지며 /diagnostics를 기다리지 않는다.
        # 발행한 값과 같은 can_ok를 쓴다. 여기서 다시 조회하면 사이클 중간의
        # 재연결 성공이 같은 사이클의 출력을 풀어 준다.
        if not can_ok:
            speed_ratio = 0.0

        # -------------------------
        # /cmd_vel_safe 원시 명령 (stale이면 speed_ratio가 이미 0이지만
        # cmd 신선도를 다시 확인해 원시값도 0으로 강제, 이중 방어)
        # -------------------------
        cmd_alive = is_fresh_ns(
            self.last_cmd_ns,
            now_ns=now,
            timeout_ns=self.cmd_timeout_ns,
        )

        if not cmd_alive:
            raw_linear_x = 0.0
            raw_angular_z = 0.0
        else:
            raw_linear_x = self.cmd_linear_x
            raw_angular_z = self.cmd_angular_z

        # -------------------------
        # knob 기준 최고속도 계산
        # -------------------------
        allowed_linear = self.max_linear_mps * speed_ratio
        allowed_angular = self.max_angular_radps * speed_ratio

        # 키보드 입력값을 knob가 허용한 최고속도 안으로 제한
        limited_linear_x = clamp(
            raw_linear_x,
            -allowed_linear,
            allowed_linear
        )

        limited_angular_z = clamp(
            raw_angular_z,
            -allowed_angular,
            allowed_angular
        )

        # -------------------------
        # differential drive 계산
        #
        # ROS 기준:
        #   linear.x  + : 전진
        #   angular.z + : 좌회전
        #
        # v_left  = v - w * L/2
        # v_right = v + w * L/2
        # -------------------------
        v_left = limited_linear_x - limited_angular_z * self.wheel_base_m / 2.0
        v_right = limited_linear_x + limited_angular_z * self.wheel_base_m / 2.0

        rpm_left = self.mps_to_rpm(v_left)
        rpm_right = self.mps_to_rpm(v_right)

        # MOT1 = 오른쪽, MOT2 = 왼쪽
        rpm_mot1 = rpm_right
        rpm_mot2 = rpm_left

        # 방향 보정
        if self.invert_mot1:
            rpm_mot1 *= -1
        if self.invert_mot2:
            rpm_mot2 *= -1

        rpm_mot1 = int(round(clamp(rpm_mot1, -self.max_rpm, self.max_rpm)))
        rpm_mot2 = int(round(clamp(rpm_mot2, -self.max_rpm, self.max_rpm)))

        rpm_mot1 = self.apply_min_rpm(rpm_mot1)
        rpm_mot2 = self.apply_min_rpm(rpm_mot2)

        # 타이머마다 동일한 CAN 속도 명령을 반복 전송하지 않습니다.
        # 모터 명령이 바뀌었을 때만 전송하고, 최소 `resend_interval_ns`만큼 간격을 둡니다.
        resend_due = (
            self.last_send_ns is None or
            (now - self.last_send_ns) >= self.resend_interval_ns
        )
        if (
            self.prev_rpm_mot1 != rpm_mot1 or
            self.prev_rpm_mot2 != rpm_mot2 or
            resend_due
        ):
            self.send_vel_cmd(rpm_mot1, rpm_mot2, ret_type=RET_TYPE_ODOM)
            self.prev_rpm_mot1 = rpm_mot1
            self.prev_rpm_mot2 = rpm_mot2
            self.last_send_ns = now

        # -------------------------
        # 디버그 출력
        # -------------------------
        print_due = (
            self.last_print_ns is None or
            (now - self.last_print_ns) > sec_to_ns(0.2)
        )
        if print_due:
            self.get_logger().info(
                f"knob1={self.knob1:3d}% "
                f"limit=({allowed_linear:.2f}m/s,{allowed_angular:.2f}rad/s) "
                f"cmd=({raw_linear_x:+.2f},{raw_angular_z:+.2f}) "
                f"out=({limited_linear_x:+.2f},{limited_angular_z:+.2f}) "
                f"rpm MOT1/R={rpm_mot1:+4d}, MOT2/L={rpm_mot2:+4d}"
            )
            self.last_print_ns = now

    def mps_to_rpm(self, v_mps: float) -> float:
        circumference = 2.0 * math.pi * self.wheel_radius_m
        return (v_mps / circumference) * 60.0

    def apply_min_rpm(self, rpm: int) -> int:
        if self.min_rpm_when_moving <= 0:
            return rpm

        if rpm == 0:
            return 0

        if abs(rpm) < self.min_rpm_when_moving:
            return self.min_rpm_when_moving if rpm > 0 else -self.min_rpm_when_moving

        return rpm

    def stop_motors(self):
        try:
            self.send_vel_cmd(0, 0, ret_type=RET_TYPE_NONE)
            time.sleep(0.02)
            self.send_vel_cmd(0, 0, ret_type=RET_TYPE_NONE)
        except Exception as e:
            self.get_logger().warn(f"stop_motors failed: {e}")

    def destroy_node(self):
        self.get_logger().info("Stopping motors...")
        self.stop_motors()

        try:
            self.bus.shutdown()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = MdrobotCanKeyboardKnobNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
