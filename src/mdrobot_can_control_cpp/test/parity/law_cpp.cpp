#include <cstdio>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <sstream>
#include <string>

#include "mdrobot_can_control_cpp/motor_core.hpp"
#include "mdrobot_can_control_cpp/socketcan.hpp"

namespace
{

void apply_param(mdrobot::Params & p, const std::string & key, const std::string & value)
{
  const auto i64 = [&]() {return static_cast<int64_t>(std::stoll(value, nullptr, 0));};
  const auto f64 = [&]() {return std::strtod(value.c_str(), nullptr);};
  if (key == "invert_mot1") {
    p.invert_mot1 = value == "1";
  } else if (key == "invert_mot2") {
    p.invert_mot2 = value == "1";
  } else if (key == "min_rpm_when_moving") {
    p.min_rpm_when_moving = i64();
  } else if (key == "max_rpm") {
    p.max_rpm = i64();
  } else if (key == "deadzone_pct") {
    p.deadzone_pct = i64();
  } else if (key == "knob_min_pct") {
    p.knob_min_pct = i64();
  } else if (key == "knob_max_pct") {
    p.knob_max_pct = i64();
  } else if (key == "driver_id") {
    p.driver_id = i64();
  } else if (key == "wheel_radius_m") {
    p.wheel_radius_m = f64();
  } else if (key == "wheel_base_m") {
    p.wheel_base_m = f64();
  } else if (key == "max_linear_mps") {
    p.max_linear_mps = f64();
  } else if (key == "max_angular_radps") {
    p.max_angular_radps = f64();
  } else if (key == "resend_interval_sec") {
    p.resend_interval_sec = f64();
  } else if (key == "knob_timeout_sec") {
    p.knob_timeout_sec = f64();
  } else if (key == "cmd_timeout_sec") {
    p.cmd_timeout_sec = f64();
  } else if (key == "can_reconnect_interval_sec") {
    p.can_reconnect_interval_sec = f64();
  } else {
    std::fprintf(stderr, "unknown PARAMS key: %s\n", key.c_str());
    std::exit(2);
  }
}

}

int main()
{
  mdrobot::Params p;
  p.knob_min_pct = 55;
  p.knob_max_pct = 98;
  if (const char * env = std::getenv("PARAMS")) {
    std::istringstream params(env);
    std::string kv;
    while (params >> kv) {
      const auto eq = kv.find('=');
      apply_param(p, kv.substr(0, eq), kv.substr(eq + 1));
    }
  }
  mdrobot::Core core(p);
  int64_t now = 0;
  std::deque<mdrobot::Frame> rxq;
  bool fail_send = false;
  bool fail_recv = false;
  bool reopen_ok = true;
  bool has_bus = true;

  core.now_fn = [&]() {return now;};
  auto & io = core.io;
  io.has_bus = [&]() {return has_bus;};
  io.recv_nowait = [&]() -> std::optional<mdrobot::Frame> {
      if (fail_recv) {
        fail_recv = false;
        throw mdrobot::CanFailure("Failed to receive: Network is down [Error Code 100]");
      }
      if (rxq.empty()) {
        return std::nullopt;
      }
      auto f = rxq.front();
      rxq.pop_front();
      return f;
    };
  io.send = [&](const mdrobot::Frame & f) {
      if (fail_send) {
        fail_send = false;
        throw mdrobot::CanFailure(
          "Failed to transmit: No buffer space available [Error Code 105]");
      }
      std::cout << "S ";
      for (const uint8_t byte : mdrobot::pack_classic_frame(f)) {
        char b[3];
        std::snprintf(b, sizeof(b), "%02x", byte);
        std::cout << b;
      }
      std::cout << "\n";
    };
  io.reopen = [&]() {
      has_bus = false;
      if (!reopen_ok) {
        throw mdrobot::CanFailure("[Errno 19] No such device");
      }
      has_bus = true;
    };
  io.publish_can_ok = [&](bool ok) {std::cout << "P " << (ok ? 1 : 0) << "\n";};
  io.log_info = [&](const std::string & s) {std::cout << "I " << s << "\n";};
  io.log_error = [&](const std::string & s) {std::cout << "E " << s << "\n";};

  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream in(line);
    std::string k;
    in >> k;
    try {
      if (k == "T") {
        in >> now;
        std::cout << "-- " << now << "\n";
        core.control_loop();
      } else if (k == "C") {
        std::string sx, sz;
        in >> now >> sx >> sz;
        core.on_cmd(std::stod(sx), std::stod(sz));
      } else if (k == "R") {
        unsigned id = 0;
        unsigned dlc = 0;
        std::string hex;
        in >> std::hex >> id >> std::dec >> dlc >> hex;
        mdrobot::Frame f;
        f.id = id;
        f.dlc = static_cast<uint8_t>(dlc);
        for (unsigned i = 0; i < 8; ++i) {
          f.data[i] = i < dlc ?
            static_cast<uint8_t>(std::stoul(hex.substr(2 * i, 2), nullptr, 16)) : 0;
        }
        rxq.push_back(f);
      } else if (k == "FS") {
        fail_send = true;
      } else if (k == "FR") {
        fail_recv = true;
      } else if (k == "RO") {
        std::string v;
        in >> v;
        reopen_ok = v == "ok";
      }
    } catch (const std::exception & e) {
      std::cout << "X " << e.what() << "\n";
      return 0;
    }
  }
  return 0;
}
