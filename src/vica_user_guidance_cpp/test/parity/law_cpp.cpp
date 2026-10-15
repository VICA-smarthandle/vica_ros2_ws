// 대조 시험의 C++ 쪽: law_py.py 와 같은 시나리오를 DriverCore 에 넣고 같은 꼴로 찍는다.
// 명령·출력 형식은 law_py.py 머리말을 본다.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "vica_user_guidance_cpp/driver_core.hpp"
#include "vica_user_guidance_cpp/pyfmt.hpp"
#include "vica_user_guidance_cpp/pyjson.hpp"

using namespace vica_guidance;   // NOLINT

namespace
{

std::vector<std::string> OUT;

void emit(const std::string & line) {OUT.push_back(line);}

std::string esc(const std::string & s)
{
  std::string o;
  for (char c : s) {
    if (c == '\\') {o += "\\\\";} else if (c == '\n') {o += "\\n";} else if (c == '\r') {
      o += "\\r";
    } else {o.push_back(c);}
  }
  return o;
}

Bytes unhex(const std::string & h)
{
  Bytes b;
  for (std::size_t k = 0; k + 1 < h.size(); k += 2) {
    b.push_back(static_cast<uint8_t>(std::stoi(h.substr(k, 2), nullptr, 16)));
  }
  return b;
}

std::string text(const std::string & h)
{
  const Bytes b = unhex(h);
  return std::string(b.begin(), b.end());
}

std::string hex(const uint8_t * p, std::size_t n)
{
  std::string o;
  char buf[4];
  for (std::size_t k = 0; k < n; ++k) {std::snprintf(buf, sizeof(buf), "%02x", p[k]); o += buf;}
  return o;
}

std::string f32(float v)
{
  uint8_t b[4];
  std::memcpy(b, &v, 4);   // aarch64·x86 은 little-endian — struct.pack('<f') 와 같은 순서
  return hex(b, 4);
}

std::string stamp(int64_t ns)
{
  return std::to_string(ns / 1000000000) + " " + std::to_string(ns % 1000000000);
}

class FakePort;

struct State
{
  int64_t t = 0, r = 0;
  bool openfail = false;
  int writefail = 0, flushfail = 0, inwfail = 0, readfail = 0;
  bool closefail = false;
  std::map<std::string, int> tf;
  FakePort * port = nullptr;
} ST;

class FakePort : public SerialPort
{
public:
  Bytes buf;
  ~FakePort() override
  {
    if (ST.port == this) {ST.port = nullptr;}
  }
  void write(uint8_t byte) override
  {
    if (ST.writefail > 0) {--ST.writefail; emit("WFAIL"); throw std::runtime_error("write failed");}
    emit("W " + hex(&byte, 1));
  }
  void flush() override
  {
    if (ST.flushfail > 0) {--ST.flushfail; emit("FFAIL"); throw std::runtime_error("flush failed");}
    emit("FLUSH");
  }
  std::size_t in_waiting() override
  {
    if (ST.inwfail > 0) {
      --ST.inwfail; emit("IFAIL"); throw std::runtime_error("in_waiting failed");
    }
    return buf.size();
  }
  Bytes read(std::size_t n) override
  {
    if (ST.readfail > 0) {--ST.readfail; emit("RFAIL"); throw std::runtime_error("read failed");}
    const std::size_t k = std::min(n, buf.size());
    Bytes out(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(k));
    buf.erase(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(k));
    emit("RD " + std::to_string(k));
    return out;
  }
  void close() override
  {
    emit("CLOSE");
    if (ST.port == this) {ST.port = nullptr;}
    if (ST.closefail) {throw std::runtime_error("close failed");}
  }
};

std::unique_ptr<SerialPort> fake_factory(
  const std::string & port, int64_t baud, double timeout, double write_timeout)
{
  emit("OPEN " + port + " " + std::to_string(baud) + " " + py_float(timeout) + " " +
    py_float(write_timeout));
  if (ST.openfail) {emit("OPENFAIL"); throw std::runtime_error("could not open port");}
  auto p = std::make_unique<FakePort>();
  ST.port = p.get();
  return p;
}

class RecordIo : public DriverIo
{
public:
  std::vector<std::string> topics;
  int64_t steady_now_ns() override {return ST.t;}
  int64_t ros_now_ns() override {return ST.r;}
  void log(LogLevel level, const std::string & s) override
  {
    const char * l = level == LogLevel::Info ? "I" : level == LogLevel::Warn ? "W" : "E";
    emit(std::string("L ") + l + " " + esc(s));
  }
  void log_warn_throttled(double period, const std::string & s) override
  {
    emit("LT " + py_float(period) + " " + esc(s));
  }
  void publish_range(const RangeOut & r) override
  {
    emit("PR " + topics.at(static_cast<std::size_t>(r.channel)) + " " + stamp(r.stamp_ns) + " " +
      r.frame_id + " " + std::to_string(r.radiation_type) + " " + f32(r.field_of_view) + " " +
      f32(r.min_range) + " " + f32(r.max_range) + " " + f32(r.range));
  }
  void publish_state(const StateOut & s) override
  {
    emit("PS " + stamp(s.stamp_ns) + " " + s.frame_id + " " + std::to_string(s.connected) + " " +
      std::to_string(s.user_contact) + " " + std::to_string(s.uplink_fresh) + " " +
      std::to_string(s.servo_ok) + " " + std::to_string(s.left_led_ok) + " " +
      std::to_string(s.right_led_ok) + " " + std::to_string(s.haptic_ok) + " " +
      std::to_string(s.fault_code) + " " + std::to_string(s.last_state_code) + " " +
      std::to_string(s.write_error_count));
  }
  bool can_transform(const std::string & target, const std::string & source, int64_t ns) override
  {
    emit("Q " + target + " " + source + " " + stamp(ns));
    const auto it = ST.tf.find(source);
    const int mode = it == ST.tf.end() ? 1 : it->second;
    return mode == 1;   // 2(파이썬 TransformException) 도 노드는 false 로 받는다
  }
};

double num(const JsonValue & v)
{
  if (v.kind == JsonValue::Kind::Int) {return static_cast<double>(v.i);}
  return v.d;
}

void apply_params(const JsonValue & obj, DriverParams & p)
{
  for (const auto & kv : obj.o) {
    const std::string & k = kv.first;
    const JsonValue & v = kv.second;
    auto strs = [&]() {std::vector<std::string> o; for (auto & e : v.a) {o.push_back(e.s);} return o;};
    auto dbls = [&]() {std::vector<double> o; for (auto & e : v.a) {o.push_back(num(e));} return o;};
    auto bools = [&]() {std::vector<bool> o; for (auto & e : v.a) {o.push_back(e.b);} return o;};
    auto ints = [&]() {std::vector<int64_t> o; for (auto & e : v.a) {o.push_back(e.i);} return o;};
    if (k == "serial_port") {p.serial_port = v.s;} else if (k == "baudrate") {p.baudrate = v.i;} else if (k == "send_rate_hz") {p.send_rate_hz = num(v);} else if (k == "diag_rate_hz") {p.diag_rate_hz = num(v);} else if (k == "write_timeout_sec") {p.write_timeout_sec = num(v);} else if (k == "cue_timeout_sec") {p.cue_timeout_sec = num(v);} else if (k == "estop_timeout_sec") {p.estop_timeout_sec = num(v);} else if (k == "estop_required") {p.estop_required = v.b;} else if (k == "reconnect_interval_sec") {p.reconnect_interval_sec = num(v);} else if (k == "arrival_hold_sec") {p.arrival_hold_sec = num(v);} else if (k == "enable_serial") {p.enable_serial = v.b;} else if (k == "ultrasonic_enabled") {p.ultrasonic_enabled = v.b;} else if (k == "ultrasonic_rate_hz") {p.ultrasonic_rate_hz = num(v);} else if (k == "uplink_rate_hz") {p.uplink_rate_hz = num(v);} else if (k == "touch_enabled") {p.touch_enabled = v.b;} else if (k == "touch_stale_sec") {p.touch_stale_sec = num(v);} else if (k == "touch_publish_min_interval_sec") {p.touch_publish_min_interval_sec = num(v);} else if (k == "ultrasonic_topics") {p.ultrasonic_topics = strs();} else if (k == "ultrasonic_frame_ids") {p.ultrasonic_frame_ids = strs();} else if (k == "ultrasonic_fov_rad") {p.ultrasonic_fov_rad = num(v);} else if (k == "ultrasonic_fov_rad_per_channel") {p.ultrasonic_fov_rad_per_channel = dbls();} else if (k == "ultrasonic_min_range_m") {p.ultrasonic_min_range_m = num(v);} else if (k == "ultrasonic_max_range_m") {p.ultrasonic_max_range_m = num(v);} else if (k == "ultrasonic_max_range_m_per_channel") {p.ultrasonic_max_range_m_per_channel = dbls();} else if (k == "ultrasonic_channel_enabled") {p.ultrasonic_channel_enabled = bools();} else if (k == "ultrasonic_measurement_delay_ms") {p.ultrasonic_measurement_delay_ms = ints();} else if (k == "ultrasonic_stale_warn_sec") {p.ultrasonic_stale_warn_sec = num(v);} else if (k == "ultrasonic_tf_gate") {p.ultrasonic_tf_gate = v.b;} else if (k == "ultrasonic_tf_target_frame") {p.ultrasonic_tf_target_frame = v.s;} else {
      std::fprintf(stderr, "모르는 파라미터: %s\n", k.c_str());
      std::exit(2);
    }
  }
}

}  // namespace

int main()
{
  std::string line;
  DriverParams params;
  RecordIo io;
  std::unique_ptr<DriverCore> core;
  while (std::getline(std::cin, line)) {
    if (line.empty()) {continue;}
    const auto sp = line.find(' ');
    const std::string cmd = line.substr(0, sp);
    const std::string arg = sp == std::string::npos ? "" : line.substr(sp + 1);
    try {
      if (cmd == "P") {
        const auto doc = py_json_loads(arg);
        if (!doc) {std::fprintf(stderr, "P 를 못 읽음\n"); return 2;}
        params = DriverParams{};
        apply_params(*doc, params);
        io.topics = params.ultrasonic_topics;
      } else if (cmd == "T") {
        ST.t = std::stoll(arg);
      } else if (cmd == "R") {
        ST.r = std::stoll(arg);
      } else if (cmd == "openfail") {
        ST.openfail = arg == "1";
      } else if (cmd == "writefail") {
        ST.writefail = std::stoi(arg);
      } else if (cmd == "flushfail") {
        ST.flushfail = std::stoi(arg);
      } else if (cmd == "inwfail") {
        ST.inwfail = std::stoi(arg);
      } else if (cmd == "readfail") {
        ST.readfail = std::stoi(arg);
      } else if (cmd == "closefail") {
        ST.closefail = arg == "1";
      } else if (cmd == "tf") {
        const auto s2 = arg.find(' ');
        ST.tf[text(arg.substr(0, s2))] = std::stoi(arg.substr(s2 + 1));
      } else if (cmd == "rx") {
        const Bytes data = unhex(arg);
        if (ST.port) {
          ST.port->buf.insert(ST.port->buf.end(), data.begin(), data.end());
        } else {
          emit("RXDROP " + std::to_string(data.size()));
        }
      } else if (cmd == "start") {
        core = std::make_unique<DriverCore>(params, io, fake_factory);
      } else if (cmd == "turn") {
        core->on_turn(std::stoi(arg));
      } else if (cmd == "estop") {
        core->on_estop(arg == "1");
      } else if (cmd == "goal") {
        core->on_goal(text(arg));
      } else if (cmd == "haptic") {
        core->on_haptic(text(arg));
      } else if (cmd == "send") {
        core->send_loop();
      } else if (cmd == "diag") {
        core->diag_loop();
      } else if (cmd == "uplink") {
        core->uplink_loop();
      } else if (cmd == "shutdown") {
        core->shutdown_to_neutral();
      } else {
        std::fprintf(stderr, "모르는 명령: %s\n", cmd.c_str());
        return 2;
      }
    } catch (const PyError & e) {
      emit("X " + e.type() + ": " + esc(e.what()));
      if (core) {
        try {
          core->shutdown_to_neutral();
        } catch (...) {
        }
      }
      break;
    }
  }
  for (const auto & l : OUT) {std::cout << l << "\n";}
  return 0;
}
