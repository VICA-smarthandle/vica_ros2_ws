#!/usr/bin/env python3
"""
mdrobot_can_estop_safe_node.py (아두이노 미사용 버전)
 
기존 mdrobot_can_keyboard_knob_node.py 기반. SLAM + 가변저항(knob) 속도
제한 운용 그대로 유지하면서, 아래 안전 기능을 추가한다. 전부 프로젝트에
제공된 문서(CAN 통신사양 V1.9) 근거 범위 내에서만 구현.
 
[버튼 감지 - CAN만 사용, 아두이노 불필요]
  PID 241(0xF1) 첫 패킷의 D2(BIT4=START/STOP1), D3(BIT4=START/STOP2)
  입력비트를 읽어 비상정지 버튼 상태를 감지한다.
  ※ 매뉴얼에 비트 극성(눌림=0? 1?)이 명시되어 있지 않으므로(해결 불가
    항목), 먼저 f1_bit_monitor.py 로 실측한 뒤 estop_bit_pressed_value
    파라미터(0 또는 1)를 설정해야 한다. 미설정(-1) 시 버튼 감지는
    비활성이고 경고만 출력한다(기존 knob 노드와 동일하게 동작).
 
[비상정지 시 CAN 정지 명령 - 문서 근거]
  - 0xCF(207) 0 rpm
  - 0xAF(175, PID_PNT_BRAKE) : 전자브레이크(모터 구속)
      프레임 [175, ID1, ID2, x,...]  ※ 비고란 "PNT50, MD750T" 표기로
      MD200T 적용 여부 문서상 미확정 → 아래 132도 함께 송신
  - 0x84(132, PID_PNT_CMD) CMD_TYPE=1(PNT_CMD_BRAKE) : 듀얼채널 확장 명령
      프레임 [132, 1, x,...]
 
[드라이버 자체 워치독 - 문서 근거]
  - 0xB9(185, PID_COM_WATCH_DELAY): 통신 두절 시 드라이버가 스스로 정지.
    기동 시 설정 송신(기본 0.5s). 전원 재투입 후 유지 여부는 문서에
    없으므로(확인 불가) 매 기동 시 재송신.
 
[래치]
  - 비상정지 발생 후 버튼이 복귀해도 자동 재출발하지 않음.
  - 해제: 버튼 복귀 상태에서 `ros2 service call /estop_reset
    std_srvs/srv/Trigger` 호출 시에만.
 
[이 코드가 보장하지 못하는 것 - 반드시 인지]
  - 기존 테스트에서 0 rpm 송신 중에도 바퀴 회전이 관찰됨. 그 원인
    (PID_SLOW_DOWN 등 드라이버 설정 여부)이 규명되기 전에는 본 코드의
    정지 명령도 실제 정지를 보장하지 못한다. → 최종 안전정지는 하드웨어
    회로로 별도 확보 필요.
 
토픽/서비스:
  구독:  /cmd_vel (geometry_msgs/Twist)
         /emergency_stop (std_msgs/Bool)  ← 선택 입력. SLAM/기타 노드가
             True를 발행하면 동일하게 래치 진입(외부 S/W 정지 요청 용도).
             발행자가 없어도 무방(버튼 감지와 독립).
  발행:  /estop_state (std_msgs/Bool)     ← 현재 래치 상태(다른 노드 참고용)
  서비스: /estop_reset (std_srvs/Trigger)
"""
import math
import time
import can
 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
 
 
# ====== MDROBOT CAN 프로토콜 (CAN 통신사양 V1.9 근거) ======
PID_PNT_IO_MONITOR  = 0xF1  # 241 : I/O 모니터(knob + 입력비트)
PID_PNT_VEL_CMD     = 0xCF  # 207 : 속도명령
PID_COMMAND         = 0x0A  # 10
CMD_PNT_IO_MON_ON   = 0x55  # 85
PID_PNT_TQ_OFF      = 0xAE  # 174 : 토크 제거(모터 FREE)
PID_PNT_BRAKE       = 0xAF  # 175 : 전자브레이크(모터 구속)
PID_PNT_CMD         = 0x84  # 132 : 듀얼채널 확장 명령
PNT_CMD_TQ_OFF      = 0
PNT_CMD_BRAKE       = 1
PID_COM_WATCH_DELAY = 0xB9  # 185 : 통신 두절 시 드라이버 자체 정지(0.1s)
 
RET_TYPE_NONE = 0
RET_TYPE_ODOM = 5
 
BIT_START_STOP = 4  # F1 첫 패킷 D2/D3의 BIT4 = START/STOP (매뉴얼 명시)
 
 
def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v
 
 
def le_i16_signed(value: int):
    value = int(value)
    value = clamp(value, -32768, 32767)
    u = value & 0xFFFF
    return u & 0xFF, (u >> 8) & 0xFF
 
 
class MdrobotCanEstopSafeNode(Node):
    def __init__(self):
        super().__init__("mdrobot_can_estop_safe_node")
 
        # ========= 기존 파라미터 (원 노드와 동일) =========
        self.declare_parameter("can_iface", "can1")
        self.declare_parameter("driver_id", 0x001)
        self.declare_parameter("wheel_radius_m", 0.065)
        self.declare_parameter("wheel_base_m", 0.37)
        self.declare_parameter("max_linear_mps", 1.0)
        self.declare_parameter("max_angular_radps", 2.0)
        self.declare_parameter("max_rpm", 400)
        self.declare_parameter("send_hz", 30.0)
        self.declare_parameter("resend_interval_sec", 0.05)
        self.declare_parameter("deadzone_pct", 5)
        self.declare_parameter("knob_timeout_sec", 0.8)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("invert_mot1", False)
        self.declare_parameter("invert_mot2", False)
        self.declare_parameter("min_rpm_when_moving", 0)
 
        # ========= 안전 계층 파라미터 (신규) =========
        # F1 D2/D3 BIT4가 "버튼 눌림"일 때 갖는 값(0 또는 1).
        # 매뉴얼에 극성 명시 없음 → f1_bit_monitor.py 로 실측 후 설정.
        # -1 = 미설정: 버튼 감지 비활성(경고 출력, 기존 노드처럼 동작)
        self.declare_parameter("estop_bit_pressed_value", -1)
        # F1 미수신(knob_timeout과 동일 시계) 시 래치 진입 여부.
        # True 권장: F1이 안 오면 버튼 상태를 알 수 없으므로 안전측.
        self.declare_parameter("estop_on_f1_timeout", True)
        # 정지 방식: "brake"(구속, 경사면 권장) | "tq_off"(모터 프리)
        self.declare_parameter("estop_mode", "brake")
        # 드라이버 워치독(0.1s 단위). 0이면 설정 안 함. 5 → 0.5s
        self.declare_parameter("com_watch_delay_0p1s", 5)
        # 비상정지 중 정지명령 반복 주기
        self.declare_parameter("estop_cmd_hz", 20.0)
        # 기동 직후 래치 상태로 시작할지. True 권장(안전측).
        # 기존 운용 흐름 유지를 원하면 False로.
        self.declare_parameter("start_latched", True)
 
        # ========= 파라미터 로드 =========
        gp = self.get_parameter
        self.can_iface = gp("can_iface").value
        self.driver_id = int(gp("driver_id").value)
        self.wheel_radius_m = float(gp("wheel_radius_m").value)
        self.wheel_base_m = float(gp("wheel_base_m").value)
        self.max_linear_mps = float(gp("max_linear_mps").value)
        self.max_angular_radps = float(gp("max_angular_radps").value)
        self.max_rpm = int(gp("max_rpm").value)
        self.send_hz = float(gp("send_hz").value)
        self.resend_interval_sec = float(gp("resend_interval_sec").value)
        self.deadzone_pct = int(gp("deadzone_pct").value)
        self.knob_timeout_sec = float(gp("knob_timeout_sec").value)
        self.cmd_timeout_sec = float(gp("cmd_timeout_sec").value)
        self.invert_mot1 = bool(gp("invert_mot1").value)
        self.invert_mot2 = bool(gp("invert_mot2").value)
        self.min_rpm_when_moving = int(gp("min_rpm_when_moving").value)
 
        self.estop_bit_pressed_value = int(
            gp("estop_bit_pressed_value").value)
        self.estop_on_f1_timeout = bool(gp("estop_on_f1_timeout").value)
        self.estop_mode = str(gp("estop_mode").value)
        self.com_watch_delay_0p1s = int(gp("com_watch_delay_0p1s").value)
        self.estop_cmd_hz = float(gp("estop_cmd_hz").value)
        self.start_latched = bool(gp("start_latched").value)
 
        # ========= 실행 상태 =========
        self.knob1 = 0
        self.knob2 = 0
        self.last_knob_time = 0.0
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        self.last_cmd_time = 0.0
        self.last_print_time = 0.0
        self.prev_rpm_mot1 = None
        self.prev_rpm_mot2 = None
        self.last_send_time = 0.0
 
        # ---- 안전 상태 ----
        self.button_pressed = False       # F1 비트 기반 버튼 상태
        self.ext_estop = False            # /emergency_stop 토픽 요청
        self.estop_latched = self.start_latched
        self.last_estop_cmd_time = 0.0
        self.f1_timeout_warned = False
 
        # ========= CAN 초기화 =========
        self.bus = can.interface.Bus(
            channel=self.can_iface, interface="socketcan")
        self.get_logger().info(f"CAN opened: {self.can_iface}")
        self.get_logger().info(f"Driver ID: 0x{self.driver_id:03X}")
 
        if self.com_watch_delay_0p1s > 0:
            self.send_com_watch_delay(self.com_watch_delay_0p1s)
            time.sleep(0.02)
            self.send_com_watch_delay(self.com_watch_delay_0p1s)
            self.get_logger().info(
                f"PID_COM_WATCH_DELAY(185)={self.com_watch_delay_0p1s} "
                f"({self.com_watch_delay_0p1s*0.1:.1f}s) 송신. "
                "실제 반영 여부는 CAN 두절 실기 테스트로 검증할 것")
 
        self.send_pnt_io_monitor_on()
        time.sleep(0.05)
        self.send_pnt_io_monitor_on()
 
        # ========= ROS 인터페이스 =========
        self.sub_cmd_vel = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.sub_ext_estop = self.create_subscription(
            Bool, "/emergency_stop", self.ext_estop_callback, 10)
        self.pub_state = self.create_publisher(Bool, "/estop_state", 10)
        self.srv_reset = self.create_service(
            Trigger, "/estop_reset", self.estop_reset_callback)
 
        self.timer = self.create_timer(1.0 / self.send_hz, self.control_loop)
 
        if self.estop_bit_pressed_value not in (0, 1):
            self.get_logger().warn(
                "estop_bit_pressed_value 미설정(-1) → F1 기반 버튼 감지 "
                "비활성. f1_bit_monitor.py 로 눌림 시 비트값(0/1)을 실측한 "
                "뒤 파라미터로 지정하세요. (매뉴얼에 극성 미기재 - 실측 필수)")
        if self.estop_latched:
            self.get_logger().warn(
                "기동: ESTOP LATCHED 상태로 시작(start_latched=True). "
                "해제: ros2 service call /estop_reset std_srvs/srv/Trigger")
        self.get_logger().warn(
            "※ 본 노드는 보조 안전 계층. 0rpm 송신 중 바퀴 회전이 관찰된 "
            "기존 테스트 결과가 있으므로 최종 안전정지는 H/W로 확보할 것.")
 
    # ============================================================
    # CAN 송신
    # ============================================================
    def _send(self, data8: bytes):
        msg = can.Message(arbitration_id=self.driver_id,
                          is_extended_id=False, data=data8)
        self.bus.send(msg)
 
    def send_pnt_io_monitor_on(self):
        self._send(bytes([PID_COMMAND, CMD_PNT_IO_MON_ON, 0, 0, 0, 0, 0, 0]))
 
    def send_com_watch_delay(self, delay_0p1s: int):
        w = clamp(int(delay_0p1s), 0, 65535)
        self._send(bytes([PID_COM_WATCH_DELAY, w & 0xFF, (w >> 8) & 0xFF,
                          0, 0, 0, 0, 0]))
 
    def send_vel_cmd(self, rpm1: int, rpm2: int, ret_type: int = RET_TYPE_NONE):
        rpm1 = int(clamp(rpm1, -self.max_rpm, self.max_rpm))
        rpm2 = int(clamp(rpm2, -self.max_rpm, self.max_rpm))
        lo1, hi1 = le_i16_signed(rpm1)
        lo2, hi2 = le_i16_signed(rpm2)
        self._send(bytes([PID_PNT_VEL_CMD, 1, lo1, hi1, 1, lo2, hi2,
                          ret_type & 0xFF]))
 
    def send_pnt_brake(self):
        d = self.driver_id & 0xFF
        self._send(bytes([PID_PNT_BRAKE, d, d, 0, 0, 0, 0, 0]))
 
    def send_pnt_tq_off(self):
        d = self.driver_id & 0xFF
        self._send(bytes([PID_PNT_TQ_OFF, d, d, 0, 0, 0, 0, 0]))
 
    def send_pnt_cmd_brake(self):
        self._send(bytes([PID_PNT_CMD, PNT_CMD_BRAKE, 0, 0, 0, 0, 0, 0]))
 
    def send_pnt_cmd_tq_off(self):
        self._send(bytes([PID_PNT_CMD, PNT_CMD_TQ_OFF, 0, 0, 0, 0, 0, 0]))
 
    # ============================================================
    # CAN 수신: knob + START/STOP 입력비트 (F1 첫 패킷)
    # ============================================================
    def drain_can_rx(self):
        for _ in range(50):
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                break
            d = msg.data
            if len(d) == 8 and d[0] == PID_PNT_IO_MONITOR and d[1] == 0:
                # D6, D7 = knob (기존과 동일: CAN data[6], data[7])
                self.knob1 = clamp(int(d[6]), 0, 100)
                self.knob2 = clamp(int(d[7]), 0, 100)
                self.last_knob_time = time.time()
 
                # D2 BIT4 = START/STOP1, D3 BIT4 = START/STOP2
                # (D3 배치는 D2와 동일하다고 가정 - 매뉴얼 미상세, 실측 검증)
                if self.estop_bit_pressed_value in (0, 1):
                    ss1 = (d[2] >> BIT_START_STOP) & 1
                    ss2 = (d[3] >> BIT_START_STOP) & 1
                    pressed = (ss1 == self.estop_bit_pressed_value or
                               ss2 == self.estop_bit_pressed_value)
                    if pressed and not self.button_pressed:
                        self.get_logger().error(
                            f"E-STOP 버튼 눌림 감지(F1: SS1={ss1} SS2={ss2})")
                    self.button_pressed = pressed
 
    # ============================================================
    # ROS 콜백
    # ============================================================
    def cmd_vel_callback(self, msg: Twist):
        self.cmd_linear_x = float(msg.linear.x)
        self.cmd_angular_z = float(msg.angular.z)
        self.last_cmd_time = time.time()
 
    def ext_estop_callback(self, msg: Bool):
        if bool(msg.data) and not self.ext_estop:
            self.get_logger().error("/emergency_stop=True 수신 → 래치 진입")
        self.ext_estop = bool(msg.data)
 
    def estop_reset_callback(self, request, response):
        now = time.time()
        f1_alive = (self.last_knob_time > 0.0 and
                    (now - self.last_knob_time) <= self.knob_timeout_sec)
 
        if self.estop_bit_pressed_value in (0, 1) and not f1_alive:
            response.success = False
            response.message = ("F1 미수신 상태 → 버튼 상태 확인 불가로 "
                                "해제 거부. CAN/드라이버 전원 확인.")
            return response
        if self.button_pressed:
            response.success = False
            response.message = "버튼이 아직 눌린 상태 → 해제 거부."
            return response
        if self.ext_estop:
            response.success = False
            response.message = ("/emergency_stop 토픽이 아직 True → "
                                "해제 거부.")
            return response
 
        self.estop_latched = False
        self.get_logger().warn("E-STOP 래치 해제. 주행 명령 다시 허용.")
        response.success = True
        response.message = "래치 해제 완료."
        return response
 
    # ============================================================
    # 메인 루프
    # ============================================================
    def control_loop(self):
        now = time.time()
        self.drain_can_rx()
 
        f1_alive = (self.last_knob_time > 0.0 and
                    (now - self.last_knob_time) <= self.knob_timeout_sec)
 
        # ---- 비상정지 판정 ----
        estop_now = self.button_pressed or self.ext_estop
        if self.estop_on_f1_timeout and not f1_alive \
                and self.estop_bit_pressed_value in (0, 1):
            # 버튼 감지를 쓰는 상태에서 F1이 끊기면 버튼 상태 불명 → 안전측
            estop_now = True
            if not self.f1_timeout_warned:
                self.get_logger().error(
                    "F1 미수신 → 버튼 상태 불명, 비상정지 간주(fail-safe)")
                self.f1_timeout_warned = True
        elif f1_alive:
            self.f1_timeout_warned = False
 
        if estop_now and not self.estop_latched:
            self.estop_latched = True
 
        # 상태 발행
        st = Bool()
        st.data = bool(self.estop_latched)
        self.pub_state.publish(st)
 
        if self.estop_latched:
            self.run_estop_output(now)
            return
 
        # ---- 이하 정상 주행 로직 (원 노드와 동일) ----
        knob_pct = int(self.knob1) if f1_alive else 0
        if knob_pct <= self.deadzone_pct:
            knob_pct = 0
        speed_ratio = knob_pct / 100.0
 
        cmd_alive = (self.last_cmd_time > 0.0 and
                     (now - self.last_cmd_time) <= self.cmd_timeout_sec)
        raw_linear_x = self.cmd_linear_x if cmd_alive else 0.0
        raw_angular_z = self.cmd_angular_z if cmd_alive else 0.0
 
        allowed_linear = self.max_linear_mps * speed_ratio
        allowed_angular = self.max_angular_radps * speed_ratio
        limited_linear_x = clamp(raw_linear_x, -allowed_linear, allowed_linear)
        limited_angular_z = clamp(raw_angular_z,
                                  -allowed_angular, allowed_angular)
 
        v_left = limited_linear_x - limited_angular_z * self.wheel_base_m / 2.0
        v_right = limited_linear_x + limited_angular_z * self.wheel_base_m / 2.0
 
        rpm_mot1 = self.mps_to_rpm(v_right)   # MOT1 = 오른쪽
        rpm_mot2 = self.mps_to_rpm(v_left)    # MOT2 = 왼쪽
        if self.invert_mot1:
            rpm_mot1 *= -1
        if self.invert_mot2:
            rpm_mot2 *= -1
 
        rpm_mot1 = int(round(clamp(rpm_mot1, -self.max_rpm, self.max_rpm)))
        rpm_mot2 = int(round(clamp(rpm_mot2, -self.max_rpm, self.max_rpm)))
        rpm_mot1 = self.apply_min_rpm(rpm_mot1)
        rpm_mot2 = self.apply_min_rpm(rpm_mot2)
 
        if (self.prev_rpm_mot1 != rpm_mot1 or
                self.prev_rpm_mot2 != rpm_mot2 or
                (now - self.last_send_time) >= self.resend_interval_sec):
            self.send_vel_cmd(rpm_mot1, rpm_mot2, ret_type=RET_TYPE_ODOM)
            self.prev_rpm_mot1 = rpm_mot1
            self.prev_rpm_mot2 = rpm_mot2
            self.last_send_time = now
 
        if now - self.last_print_time > 0.2:
            self.get_logger().info(
                f"knob1={self.knob1:3d}% "
                f"cmd=({raw_linear_x:+.2f},{raw_angular_z:+.2f}) "
                f"rpm MOT1/R={rpm_mot1:+4d}, MOT2/L={rpm_mot2:+4d}")
            self.last_print_time = now
 
    # ------------------------------------------------------------
    def run_estop_output(self, now: float):
        """비상정지 래치 중: 0rpm + 브레이크/TQ_OFF 반복 송신.
        175와 132를 모두 송신 - MD200T에서 어느 쪽이 유효한지 문서상
        미확정이므로 두 후보를 다 보낸다(실기 검증 후 정리 가능)."""
        if (now - self.last_estop_cmd_time) < (1.0 / self.estop_cmd_hz):
            return
        self.last_estop_cmd_time = now
        try:
            self.send_vel_cmd(0, 0, ret_type=RET_TYPE_NONE)
            if self.estop_mode == "tq_off":
                self.send_pnt_tq_off()
                self.send_pnt_cmd_tq_off()
            else:
                self.send_pnt_brake()
                self.send_pnt_cmd_brake()
        except Exception as e:
            self.get_logger().error(f"ESTOP CAN 송신 실패: {e}")
 
        if now - self.last_print_time > 0.5:
            self.get_logger().error(
                f"[E-STOP LATCHED] 0rpm + {self.estop_mode} 반복 송신 중. "
                "해제: 버튼 복귀 후 /estop_reset 호출")
            self.last_print_time = now
 
    # ------------------------------------------------------------
    def mps_to_rpm(self, v_mps: float) -> float:
        circumference = 2.0 * math.pi * self.wheel_radius_m
        return (v_mps / circumference) * 60.0
 
    def apply_min_rpm(self, rpm: int) -> int:
        if self.min_rpm_when_moving <= 0:
            return rpm
        if rpm == 0:
            return 0
        if abs(rpm) < self.min_rpm_when_moving:
            return self.min_rpm_when_moving if rpm > 0 \
                else -self.min_rpm_when_moving
        return rpm
 
    def stop_motors(self):
        try:
            self.send_vel_cmd(0, 0, ret_type=RET_TYPE_NONE)
            self.send_pnt_brake()
            self.send_pnt_cmd_brake()
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
    node = MdrobotCanEstopSafeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
