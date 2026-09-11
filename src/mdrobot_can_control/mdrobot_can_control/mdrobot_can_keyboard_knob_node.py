#!/usr/bin/env python3
import math
import time

import can
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, Updater
from geometry_msgs.msg import Twist
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Bool

from .can_link import CanLink
from .can_preflight import require_can_interface_up
from .freshness import is_fresh_ns, sec_to_ns
from .motor_watchdog import motor_speed_ratio, normalize_knob_pct


CAN_FAILURES = (can.CanError, OSError, ValueError)


PID_PNT_IO_MONITOR = 0xF1
PID_PNT_VEL_CMD = 0xCF
PID_COMMAND = 0x0A
CMD_PNT_IO_MON_ON = 0x55

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
    """Drive the MDROBOT motors from /cmd_vel_safe over CAN."""

    def __init__(self):
        super().__init__('mdrobot_can_keyboard_knob_node')

        self.declare_parameter('can_iface', 'can1')
        self.declare_parameter('driver_id', 0x001)

        self.declare_parameter('wheel_radius_m', 0.065)

        self.declare_parameter('wheel_base_m', 0.37)

        self.declare_parameter('max_linear_mps', 1.0)
        self.declare_parameter('max_angular_radps', 2.0)

        self.declare_parameter('max_rpm', 400)

        self.declare_parameter('send_hz', 30.0)

        self.declare_parameter('resend_interval_sec', 0.05)

        self.declare_parameter('deadzone_pct', 5)

        self.declare_parameter('knob_min_pct', 0)
        self.declare_parameter('knob_max_pct', 100)

        self.declare_parameter('knob_timeout_sec', 0.8)

        self.declare_parameter('cmd_timeout_sec', 0.5)

        self.declare_parameter('invert_mot1', False)
        self.declare_parameter('invert_mot2', False)

        self.declare_parameter('min_rpm_when_moving', 0)

        self.declare_parameter('can_reconnect_interval_sec', 1.0)

        self.can_iface = self.get_parameter('can_iface').value
        self.driver_id = int(self.get_parameter('driver_id').value)

        self.wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)
        self.wheel_base_m = float(self.get_parameter('wheel_base_m').value)

        self.max_linear_mps = float(self.get_parameter('max_linear_mps').value)
        self.max_angular_radps = float(self.get_parameter('max_angular_radps').value)
        self.max_rpm = int(self.get_parameter('max_rpm').value)

        self.send_hz = float(self.get_parameter('send_hz').value)
        self.resend_interval_sec = float(self.get_parameter('resend_interval_sec').value)
        self.deadzone_pct = int(self.get_parameter('deadzone_pct').value)
        self.knob_min_pct = int(self.get_parameter('knob_min_pct').value)
        self.knob_max_pct = int(self.get_parameter('knob_max_pct').value)
        self.knob_timeout_sec = float(self.get_parameter('knob_timeout_sec').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)

        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.knob_timeout_ns = sec_to_ns(self.knob_timeout_sec)
        self.cmd_timeout_ns = sec_to_ns(self.cmd_timeout_sec)
        self.resend_interval_ns = sec_to_ns(self.resend_interval_sec)

        self.invert_mot1 = bool(self.get_parameter('invert_mot1').value)
        self.invert_mot2 = bool(self.get_parameter('invert_mot2').value)

        self.min_rpm_when_moving = int(
            self.get_parameter('min_rpm_when_moving').value
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

        self.bus = can.interface.Bus(
            channel=self.can_iface,
            interface='socketcan'
        )

        self.can_link = CanLink(
            retry_interval_ns=self.can_reconnect_interval_ns
        )

        self.get_logger().info(f'CAN opened: {self.can_iface}')
        self.get_logger().info(f'Driver ID: 0x{self.driver_id:03X}')

        self.send_pnt_io_monitor_on()
        time.sleep(0.05)
        self.send_pnt_io_monitor_on()

        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel_safe',
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

        self.get_logger().info('Subscribed: /cmd_vel_safe')
        self.get_logger().info('knob1 = 최고속도 제한기')
        self.get_logger().info('Ready.')

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
        """Report CAN link health for operators."""
        now = self.now_ns()
        if self.can_link.is_ok():
            stat.summary(DiagnosticStatus.OK, 'CAN link OK')
        else:
            stat.summary(
                DiagnosticStatus.ERROR,
                'CAN link FAILED; motor output forced to 0',
            )
        stat.add('iface', self.can_iface)
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
        """Reopen the CAN bus while the link is failed."""
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

    def send_pnt_io_monitor_on(self):
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
        """Send both wheel speeds as one CAN frame."""
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
        """F1 monitor packet에서 knob1, knob2 값 읽기."""
        if self.bus is None:
            return

        try:
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

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_linear_x = float(msg.linear.x)
        self.cmd_angular_z = float(msg.angular.z)
        self.last_cmd_ns = self.now_ns()

    def control_loop(self):
        now = self.now_ns()

        self.drain_can_rx(now)

        can_ok = self.can_link.is_ok()
        can_ok_msg = Bool()
        can_ok_msg.data = can_ok
        self.pub_can_ok.publish(can_ok_msg)

        if not can_ok:
            self.try_reconnect_can(now)

        knob_norm = normalize_knob_pct(
            int(self.knob1), self.knob_min_pct, self.knob_max_pct)
        speed_ratio = motor_speed_ratio(
            cmd_last_ns=self.last_cmd_ns,
            knob_last_ns=self.last_knob_ns,
            knob_pct=knob_norm,
            now_ns=now,
            cmd_timeout_ns=self.cmd_timeout_ns,
            knob_timeout_ns=self.knob_timeout_ns,
            deadzone_pct=self.deadzone_pct,
        )

        if not can_ok:
            speed_ratio = 0.0

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

        allowed_linear = self.max_linear_mps * speed_ratio
        allowed_angular = self.max_angular_radps * speed_ratio

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

        v_left = limited_linear_x - limited_angular_z * self.wheel_base_m / 2.0
        v_right = limited_linear_x + limited_angular_z * self.wheel_base_m / 2.0

        rpm_left = self.mps_to_rpm(v_left)
        rpm_right = self.mps_to_rpm(v_right)

        rpm_mot1 = rpm_right
        rpm_mot2 = rpm_left

        if self.invert_mot1:
            rpm_mot1 *= -1
        if self.invert_mot2:
            rpm_mot2 *= -1

        rpm_mot1 = int(round(clamp(rpm_mot1, -self.max_rpm, self.max_rpm)))
        rpm_mot2 = int(round(clamp(rpm_mot2, -self.max_rpm, self.max_rpm)))

        rpm_mot1 = self.apply_min_rpm(rpm_mot1)
        rpm_mot2 = self.apply_min_rpm(rpm_mot2)

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

        print_due = (
            self.last_print_ns is None or
            (now - self.last_print_ns) > sec_to_ns(0.2)
        )
        if print_due:
            self.get_logger().info(
                f'knob1={self.knob1:3d}%->{knob_norm:3d}% '
                f'limit=({allowed_linear:.2f}m/s,{allowed_angular:.2f}rad/s) '
                f'cmd=({raw_linear_x:+.2f},{raw_angular_z:+.2f}) '
                f'out=({limited_linear_x:+.2f},{limited_angular_z:+.2f}) '
                f'rpm MOT1/R={rpm_mot1:+4d}, MOT2/L={rpm_mot2:+4d}'
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
            self.get_logger().warn(f'stop_motors failed: {e}')

    def destroy_node(self):
        self.get_logger().info('Stopping motors...')
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


if __name__ == '__main__':
    main()
