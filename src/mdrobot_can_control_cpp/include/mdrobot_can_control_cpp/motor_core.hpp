// keyboard_knob 의 판정·계산 핵심. ROS·소켓에 의존하지 않아 시험이 노드 없이 돈다.
//
// 파이썬 판을 한 줄씩 옮겼다. 원본(같은 저장소):
//   mdrobot_can_control/mdrobot_can_keyboard_knob_node.py  control_loop 와 CAN 함수
//   mdrobot_can_control/motor_watchdog.py, freshness.py, can_link.py
//
// "기능 차이 0" 을 지키려고 맞춘 것:
//   - 파이썬 round() 는 짝수 쪽 반올림이다 → std::nearbyint. NaN 이면 파이썬처럼 예외로
//     프로세스가 죽는다(출력이 끊겨 /motor/can_ok 가 멈추면 중앙 래치가 걸린다).
//   - 파이썬 clamp 는 NaN 을 그대로 흘린다 → 같은 비교식(clampd).
//   - 곱·더하기를 한 명령(FMA)으로 합치면 끝자리가 달라진다 → CMake 에서 -ffp-contract=off.
//   - 로그 글자는 f-string 과 같은 printf 서식, 오류 글자는 python-can 의 str(exc) 와 같다.
// 2026-10-12 대조: 파이썬 원본 메서드와 같은 무작위 시나리오 36만 사이클에서 발행·송신
// 프레임·로그가 글자까지 같았다(test/parity).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace mdrobot
{

// ====== MDROBOT CAN 프로토콜 ======
constexpr uint8_t PID_PNT_IO_MONITOR = 0xF1;  // 241
constexpr uint8_t PID_PNT_VEL_CMD = 0xCF;  // 207
constexpr uint8_t PID_COMMAND = 0x0A;  // 10
constexpr uint8_t CMD_PNT_IO_MON_ON = 0x55;  // 85

constexpr int RET_TYPE_NONE = 0;
constexpr int RET_TYPE_ODOM = 5;

// math.pi 와 같은 double.
constexpr double kPi = 3.14159265358979323846;

// 파이썬의 CAN_FAILURES(can.CanError, OSError, ValueError) 자리.
// what() 은 파이썬 str(exc) 와 같은 글자다 — 진단 last_error 에 그대로 실린다.
struct CanFailure : std::runtime_error
{
  using std::runtime_error::runtime_error;
};

struct Frame
{
  uint32_t id = 0;
  uint8_t dlc = 0;
  std::array<uint8_t, 8> data{};
};

// freshness.sec_to_ns: int(seconds * 1_000_000_000) — 0 쪽으로 자른다. NaN·inf 파라미터는
// 파이썬처럼 기동 때 예외로 죽는다. int64 를 넘는 값은 파이썬 큰 정수와 비교 결과가 같게 막는다.
inline int64_t sec_to_ns(double seconds)
{
  const double ns = seconds * 1000000000.0;
  if (std::isnan(ns)) {
    throw std::domain_error("cannot convert float NaN to integer");
  }
  if (std::isinf(ns)) {
    throw std::overflow_error("cannot convert float infinity to integer");
  }
  if (ns >= 9.2e18) {
    return INT64_MAX;
  }
  if (ns <= -9.2e18) {
    return INT64_MIN;
  }
  return static_cast<int64_t>(ns);
}

// freshness.is_fresh_ns: 미수신(None)·시간 역전(음수 age)·timeout 이면 false.
inline bool is_fresh_ns(std::optional<int64_t> last_ns, int64_t now_ns, int64_t timeout_ns)
{
  if (!last_ns) {
    return false;
  }
  const int64_t age_ns = now_ns - *last_ns;
  return 0 <= age_ns && age_ns <= timeout_ns;
}

// 파이썬 round(x) 를 int 로: 짝수 쪽 반올림. NaN 은 ValueError, inf 는 OverflowError 처럼 던진다.
inline int64_t py_round(double x)
{
  if (std::isnan(x)) {
    throw std::domain_error("cannot convert float NaN to integer");
  }
  if (std::isinf(x)) {
    throw std::overflow_error("cannot convert float infinity to integer");
  }
  const double r = std::nearbyint(x);
  // 파이썬은 큰 정수를 그대로 쓴다. 여기서는 int64 끝에서 멈춘다(뒤따르는 clamp 결과는 같다).
  if (r >= 9223372036854775808.0) {
    return INT64_MAX;
  }
  if (r < -9223372036854775808.0) {
    return INT64_MIN;
  }
  return static_cast<int64_t>(r);
}

// 파이썬 clamp(v, lo, hi): lo if v < lo else hi if v > hi else v  (NaN 은 그대로 통과)
inline double clampd(double v, double lo, double hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

inline int64_t clampi(int64_t v, int64_t lo, int64_t hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

// -v. INT64_MIN 에서도 정의된 값(INT64_MAX)을 낸다 — 말이 안 되는 max_rpm 에서의 UB 방지.
inline int64_t neg_sat(int64_t v) {return v == INT64_MIN ? INT64_MAX : -v;}

// a - b 를 double 로. 파이썬 정수 뺄셈은 넘치지 않으므로, 넘칠 때는 정확한 long double 로 뺀다.
inline double sub_to_double(int64_t a, int64_t b)
{
  int64_t d = 0;
  if (!__builtin_sub_overflow(a, b, &d)) {
    return static_cast<double>(d);
  }
  return static_cast<double>(static_cast<long double>(a) - static_cast<long double>(b));
}

// motor_watchdog.normalize_knob_pct: 줄이 실제로 움직이는 구간(min~max)을 0~100 으로 편다.
// 상한 <= 하한 은 설정 실수라 원값을 그대로 흘린다(조용히 0 을 내면 원인을 찾기 어렵다).
inline int64_t normalize_knob_pct(int64_t raw_pct, int64_t min_pct, int64_t max_pct)
{
  if (max_pct <= min_pct) {
    return raw_pct;
  }
  const int64_t scaled = py_round(
    sub_to_double(raw_pct, min_pct) * 100.0 / sub_to_double(max_pct, min_pct));
  return std::max<int64_t>(0, std::min<int64_t>(100, scaled));
}

// motor_watchdog.motor_speed_ratio: cmd·knob 중 하나라도 stale 이면 0.0(정지).
inline double motor_speed_ratio(
  std::optional<int64_t> cmd_last_ns, std::optional<int64_t> knob_last_ns,
  int64_t knob_pct, int64_t now_ns, int64_t cmd_timeout_ns, int64_t knob_timeout_ns,
  int64_t deadzone_pct)
{
  const bool cmd_fresh = is_fresh_ns(cmd_last_ns, now_ns, cmd_timeout_ns);
  const bool knob_fresh = is_fresh_ns(knob_last_ns, now_ns, knob_timeout_ns);
  if (!cmd_fresh || !knob_fresh) {
    return 0.0;
  }
  if (knob_pct <= deadzone_pct) {
    return 0.0;
  }
  return static_cast<double>(knob_pct) / 100.0;
}

// can_link.CanLink: CAN 예외를 상태로 바꾸고 재연결 시점을 정한다. 걸쇠가 아니다 —
// 걸쇠는 vica_safety 의 중앙 래치가 소유한다. 시각은 호출자가 넣는다.
class CanLink
{
public:
  explicit CanLink(int64_t retry_interval_ns)
  : retry_interval_ns_(retry_interval_ns) {}

  void record_success()
  {
    ok_ = true;
    last_attempt_ns_.reset();
  }

  void record_error(const std::string & what, int64_t now_ns)
  {
    ok_ = false;
    last_error = what;
    last_attempt_ns_ = now_ns;
  }

  bool is_ok() const {return ok_;}

  // 시간 역전에서도 true — 재연결이 영구히 막히면 관리자 reset 이 불가능해진다.
  bool should_retry(int64_t now_ns) const
  {
    if (ok_) {
      return false;
    }
    if (!last_attempt_ns_) {
      return true;
    }
    const int64_t elapsed = now_ns - *last_attempt_ns_;
    if (elapsed < 0) {
      return true;
    }
    return elapsed >= retry_interval_ns_;
  }

  void mark_retry_attempted(int64_t now_ns) {last_attempt_ns_ = now_ns;}

  // 파이썬 None 은 비어 있음. 진단에는 "None" 으로 찍는다.
  std::optional<std::string> last_error;

private:
  int64_t retry_interval_ns_;
  bool ok_ = true;
  std::optional<int64_t> last_attempt_ns_;
};

// 노드 파라미터. 기본값은 파이썬 판 declare_parameter 와 같다.
struct Params
{
  std::string can_iface = "can1";
  int64_t driver_id = 0x001;
  double wheel_radius_m = 0.065;
  double wheel_base_m = 0.37;
  double max_linear_mps = 1.0;
  double max_angular_radps = 2.0;
  int64_t max_rpm = 400;
  double send_hz = 30.0;
  double resend_interval_sec = 0.05;
  int64_t deadzone_pct = 5;
  int64_t knob_min_pct = 0;
  int64_t knob_max_pct = 100;
  double knob_timeout_sec = 0.8;
  double cmd_timeout_sec = 0.5;
  bool invert_mot1 = false;
  bool invert_mot2 = false;
  int64_t min_rpm_when_moving = 0;
  double can_reconnect_interval_sec = 1.0;
};

// 바깥 세계(소켓·발행·로그)와는 이 함수들로만 만난다. 노드와 시험이 각자 채운다.
struct Io
{
  std::function<bool()> has_bus;  // self.bus is not None
  std::function<std::optional<Frame>()> recv_nowait;  // bus.recv(timeout=0.0), 실패는 CanFailure
  std::function<void(const Frame &)> send;  // bus.send(msg), 실패는 CanFailure
  // try_reconnect_can 의 "old_bus.shutdown(); Bus()". 실패는 CanFailure 이고 그때 has_bus() 는 false.
  std::function<void()> reopen;
  std::function<void(bool)> publish_can_ok;
  std::function<void(const std::string &)> log_info;
  std::function<void(const std::string &)> log_error;
};

class Core
{
public:
  Params p;
  int64_t knob_timeout_ns;
  int64_t cmd_timeout_ns;
  int64_t resend_interval_ns;
  int64_t can_reconnect_interval_ns;

  int64_t knob1 = 0;
  int64_t knob2 = 0;
  std::optional<int64_t> last_knob_ns;

  double cmd_linear_x = 0.0;
  double cmd_angular_z = 0.0;
  std::optional<int64_t> last_cmd_ns;

  std::optional<int64_t> last_print_ns;
  std::optional<int64_t> prev_rpm_mot1;
  std::optional<int64_t> prev_rpm_mot2;
  std::optional<int64_t> last_send_ns;
  std::optional<int64_t> last_can_error_log_ns;

  CanLink can_link;
  Io io;
  // 파이썬 self.now_ns(). 원본이 그 자리에서 시각을 다시 읽는 곳은 여기서도 다시 읽는다.
  std::function<int64_t()> now_fn;

  explicit Core(const Params & params)
  : p(params),
    knob_timeout_ns(sec_to_ns(params.knob_timeout_sec)),
    cmd_timeout_ns(sec_to_ns(params.cmd_timeout_sec)),
    resend_interval_ns(sec_to_ns(params.resend_interval_sec)),
    can_reconnect_interval_ns(sec_to_ns(params.can_reconnect_interval_sec)),
    can_link(sec_to_ns(params.can_reconnect_interval_sec)) {}

  // CAN 실패를 재연결 간격에 한 번만 알린다.
  void log_can_error_throttled(const std::string & phase, const std::string & exc)
  {
    const int64_t now = now_fn();
    const bool due = !last_can_error_log_ns ||
      (now - *last_can_error_log_ns) >= can_reconnect_interval_ns;
    if (due) {
      io.log_error(
        "[CAN FAULT] phase=" + phase + " iface=" + p.can_iface + " error=" + exc +
        "; 출력을 0으로 유지합니다");
      last_can_error_log_ns = now;
    }
  }

  // python-can build_can_frame 은 id 를 struct "=I" 로 싼다. 범위 밖이면 struct.error 로
  // 프로세스가 죽는다(CAN_FAILURES 가 아니다). 송신 직전에 같은 글자로 던진다.
  void check_arbitration_id() const
  {
    if (p.driver_id < 0) {
      throw std::invalid_argument("argument out of range");
    }
    if (p.driver_id > 0xFFFFFFFFLL) {
      throw std::invalid_argument("'I' format requires 0 <= number <= 4294967295");
    }
  }

  Frame monitor_on_frame() const
  {
    Frame f;
    f.id = static_cast<uint32_t>(p.driver_id);
    f.dlc = 8;
    f.data = {PID_COMMAND, CMD_PNT_IO_MON_ON, 0, 0, 0, 0, 0, 0};
    return f;
  }

  // 예외를 삼키면 안 된다. socketcan 은 상대 드라이버가 없어도 bind 에 성공하므로, 이
  // 프로브의 예외가 재연결 성공을 판정하는 유일한 근거다. 삼키면 /motor/can_ok 가
  // 거짓으로 true 가 되어 구동단이 죽은 채 관리자 reset 이 수락된다.
  void send_pnt_io_monitor_on()
  {
    const Frame f = monitor_on_frame();
    check_arbitration_id();
    io.send(f);
  }

  static std::pair<uint8_t, uint8_t> le_i16_signed(int64_t value)
  {
    value = clampi(value, -32768, 32767);
    const uint32_t u = static_cast<uint32_t>(value) & 0xFFFF;
    return {static_cast<uint8_t>(u & 0xFF), static_cast<uint8_t>((u >> 8) & 0xFF)};
  }

  // 두 바퀴 속도를 한 프레임으로. rpm1 = MOT1 = 오른쪽, rpm2 = MOT2 = 왼쪽(바꾸면 조향이 반대).
  void send_vel_cmd(int64_t rpm1, int64_t rpm2, int ret_type)
  {
    rpm1 = clampi(rpm1, neg_sat(p.max_rpm), p.max_rpm);
    rpm2 = clampi(rpm2, neg_sat(p.max_rpm), p.max_rpm);
    const auto [lo1, hi1] = le_i16_signed(rpm1);
    const auto [lo2, hi2] = le_i16_signed(rpm2);
    Frame f;
    f.id = static_cast<uint32_t>(p.driver_id);
    f.dlc = 8;
    f.data = {PID_PNT_VEL_CMD, 1, lo1, hi1, 1, lo2, hi2, static_cast<uint8_t>(ret_type & 0xFF)};
    if (!io.has_bus()) {
      return;
    }
    check_arbitration_id();
    try {
      io.send(f);
    } catch (const CanFailure & e) {
      can_link.record_error(e.what(), now_fn());
      log_can_error_throttled("send", e.what());
    }
  }

  // F1 monitor 프레임에서 knob1(d[6])·knob2(d[7]) 를 읽는다. stamp 는 호출자가 넘긴 사이클
  // 시각을 그대로 쓴다 — 여기서 다시 읽으면 방금 받은 knob 이 음수 age 로 stale 판정된다.
  void drain_can_rx(int64_t now_ns)
  {
    if (!io.has_bus()) {
      return;
    }
    try {
      // 사이클당 읽는 프레임 수를 50 개로 제한한다.
      for (int i = 0; i < 50; ++i) {
        const auto msg = io.recv_nowait();
        if (!msg) {
          break;
        }
        const Frame & d = *msg;
        if (d.dlc == 8 && d.data[0] == PID_PNT_IO_MONITOR) {
          if (d.data[1] == 0) {
            knob1 = clampi(d.data[6], 0, 100);
            knob2 = clampi(d.data[7], 0, 100);
            last_knob_ns = now_ns;
          }
        }
      }
    } catch (const CanFailure & e) {
      can_link.record_error(e.what(), now_ns);
      log_can_error_throttled("recv", e.what());
    }
  }

  // 링크가 실패한 동안 버스를 다시 연다. 재연결에 성공해도 주행은 스스로 재개되지 않는다
  // (걸쇠는 vica_safety 소유). 재개방이 실패하면 버스는 없는 채로 남는다(io.reopen 계약).
  void try_reconnect_can(int64_t now_ns)
  {
    if (!can_link.should_retry(now_ns)) {
      return;
    }
    can_link.mark_retry_attempted(now_ns);
    try {
      io.reopen();
    } catch (const CanFailure & e) {
      can_link.record_error(e.what(), now_ns);
      return;
    }
    try {
      send_pnt_io_monitor_on();
    } catch (const CanFailure & e) {
      can_link.record_error(e.what(), now_ns);
      return;
    }
    can_link.record_success();
    io.log_info(
      "[CAN RECOVERED] iface=" + p.can_iface + "; 주행 재개는 관리자 reset 이후에만 가능합니다");
  }

  double mps_to_rpm(double v_mps) const
  {
    const double circumference = 2.0 * kPi * p.wheel_radius_m;
    // 파이썬은 0(또는 -0.0)으로 나누면 그 자리에서 ZeroDivisionError 로 죽는다. C++ 는 inf 가
    // 되어 clamp 뒤 최고 rpm 을 보낼 수 있으므로 반드시 같은 자리에서 던진다.
    if (circumference == 0.0) {
      throw std::domain_error("float division by zero");
    }
    return (v_mps / circumference) * 60.0;
  }

  int64_t apply_min_rpm(int64_t rpm) const
  {
    if (p.min_rpm_when_moving <= 0) {
      return rpm;
    }
    if (rpm == 0) {
      return 0;
    }
    if (rpm > -p.min_rpm_when_moving && rpm < p.min_rpm_when_moving) {  // abs(rpm) < min
      return rpm > 0 ? p.min_rpm_when_moving : -p.min_rpm_when_moving;
    }
    return rpm;
  }

  // cmd_vel_callback
  void on_cmd(double linear_x, double angular_z)
  {
    cmd_linear_x = linear_x;
    cmd_angular_z = angular_z;
    last_cmd_ns = now_fn();
  }

  void control_loop()
  {
    const int64_t now = now_fn();

    drain_can_rx(now);

    // 재연결 시도보다 먼저 발행한다. 장애는 반드시 최소 한 번 false 로 보고된다.
    const bool can_ok = can_link.is_ok();
    io.publish_can_ok(can_ok);

    if (!can_ok) {
      try_reconnect_can(now);
    }

    // cmd·knob 신선도 판정. 하나라도 stale·시간역전·미수신이면 0.0 → 정지.
    const int64_t knob_norm = normalize_knob_pct(knob1, p.knob_min_pct, p.knob_max_pct);
    double speed_ratio = motor_speed_ratio(
      last_cmd_ns, last_knob_ns, knob_norm, now, cmd_timeout_ns, knob_timeout_ns,
      p.deadzone_pct);

    // 발행한 값과 같은 can_ok 를 쓴다. 다시 조회하면 사이클 중간의 재연결 성공이 출력을 푼다.
    if (!can_ok) {
      speed_ratio = 0.0;
    }

    // cmd 신선도를 다시 확인해 원시값도 0 으로 강제(이중 방어).
    const bool cmd_alive = is_fresh_ns(last_cmd_ns, now, cmd_timeout_ns);
    double raw_linear_x = 0.0;
    double raw_angular_z = 0.0;
    if (cmd_alive) {
      raw_linear_x = cmd_linear_x;
      raw_angular_z = cmd_angular_z;
    }

    // knob 기준 최고속도 안으로 제한
    const double allowed_linear = p.max_linear_mps * speed_ratio;
    const double allowed_angular = p.max_angular_radps * speed_ratio;
    const double limited_linear_x = clampd(raw_linear_x, -allowed_linear, allowed_linear);
    const double limited_angular_z = clampd(raw_angular_z, -allowed_angular, allowed_angular);

    // differential drive: v_left = v - w*L/2, v_right = v + w*L/2
    const double v_left = limited_linear_x - limited_angular_z * p.wheel_base_m / 2.0;
    const double v_right = limited_linear_x + limited_angular_z * p.wheel_base_m / 2.0;

    // MOT1 = 오른쪽, MOT2 = 왼쪽
    double rpm_mot1_f = mps_to_rpm(v_right);
    double rpm_mot2_f = mps_to_rpm(v_left);
    if (p.invert_mot1) {
      rpm_mot1_f *= -1;
    }
    if (p.invert_mot2) {
      rpm_mot2_f *= -1;
    }
    const double max_rpm = static_cast<double>(p.max_rpm);
    int64_t rpm_mot1 = py_round(clampd(rpm_mot1_f, -max_rpm, max_rpm));
    int64_t rpm_mot2 = py_round(clampd(rpm_mot2_f, -max_rpm, max_rpm));
    rpm_mot1 = apply_min_rpm(rpm_mot1);
    rpm_mot2 = apply_min_rpm(rpm_mot2);

    // 명령이 바뀌었거나 resend 간격이 지났을 때만 보낸다.
    const bool resend_due = !last_send_ns || (now - *last_send_ns) >= resend_interval_ns;
    if (prev_rpm_mot1 != rpm_mot1 || prev_rpm_mot2 != rpm_mot2 || resend_due) {
      send_vel_cmd(rpm_mot1, rpm_mot2, RET_TYPE_ODOM);
      prev_rpm_mot1 = rpm_mot1;
      prev_rpm_mot2 = rpm_mot2;
      last_send_ns = now;
    }

    const bool print_due = !last_print_ns || (now - *last_print_ns) > sec_to_ns(0.2);
    if (print_due) {
      // 아주 큰 명령값(1e200 등)은 한 줄이 수백 자가 된다 — 길이를 먼저 재서 자르지 않는다.
      const auto format = [&](char * buf, size_t size) {
          return std::snprintf(
            buf, size,
            "knob1=%3lld%%->%3lld%% limit=(%.2fm/s,%.2frad/s) cmd=(%+.2f,%+.2f) "
            "out=(%+.2f,%+.2f) rpm MOT1/R=%+4lld, MOT2/L=%+4lld",
            static_cast<long long>(knob1), static_cast<long long>(knob_norm),
            allowed_linear, allowed_angular, raw_linear_x, raw_angular_z,
            limited_linear_x, limited_angular_z,
            static_cast<long long>(rpm_mot1), static_cast<long long>(rpm_mot2));
        };
      std::string line(static_cast<size_t>(format(nullptr, 0)), '\0');
      format(line.data(), line.size() + 1);
      io.log_info(line);
      last_print_ns = now;
    }
  }

  // 진단의 age 칸: 미수신이면 'never', 아니면 초 단위 소수 셋째 자리.
  static std::string age_text(std::optional<int64_t> last_ns, int64_t now_ns)
  {
    if (!last_ns) {
      return "never";
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.3f", static_cast<double>(now_ns - *last_ns) / 1e9);
    return buf;
  }
};

}  // namespace mdrobot
