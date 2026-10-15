// 대조 시험(방향지시)의 C++ 쪽: turn_law_py.py 와 같은 시나리오를 TurnCore 에 넣고 같은 꼴로 찍는다.
// 명령·출력 형식은 turn_law_py.py 머리말을 본다. TF 는 진짜 tf2::BufferCore — 노드가 쓰는 조회 함수
// (BufferCore::lookupTransform)와 파이썬 lookup_transform_core 가 같은 라이브러리라 실패 글도 비교된다.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/buffer_core.h"
#include "tf2/exceptions.h"

#include "vica_user_guidance_cpp/pyfmt.hpp"
#include "vica_user_guidance_cpp/pyjson.hpp"
#include "vica_user_guidance_cpp/turn_core.hpp"

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

std::string text(const std::string & h)
{
  std::string o;
  for (std::size_t k = 0; k + 1 < h.size(); k += 2) {
    o.push_back(static_cast<char>(std::stoi(h.substr(k, 2), nullptr, 16)));
  }
  return o;
}

std::string f32(float v)
{
  uint8_t b[4];
  std::memcpy(b, &v, 4);   // little-endian — struct.pack('<f') 와 같은 순서
  std::string o;
  char buf[4];
  for (uint8_t x : b) {std::snprintf(buf, sizeof(buf), "%02x", x); o += buf;}
  return o;
}

std::string stamp(int64_t ns)
{
  return std::to_string(ns / 1000000000) + " " + std::to_string(ns % 1000000000);
}

double num(const JsonValue & v)
{
  if (v.kind == JsonValue::Kind::Int) {return static_cast<double>(v.i);}
  return v.d;
}

struct RecordIo : public TurnIo
{
  int64_t t = 0;
  int64_t r = 0;
  std::unique_ptr<tf2::BufferCore> tf;

  int64_t steady_now_ns() override {return t;}
  int64_t ros_now_ns() override {return r;}
  void log(LogLevel level, const std::string & s) override
  {
    const char * l = level == LogLevel::Info ? "I" : (level == LogLevel::Warn ? "W" : "E");
    emit(std::string("L ") + l + " " + esc(s));
  }
  void publish_guide(const GuideOut & g) override
  {
    emit(
      "G " + stamp(g.stamp_ns) + " " + g.frame_id + " " + std::to_string(g.direction) + " " +
      std::to_string(g.phase) + " " + f32(g.distance_m) + " " + f32(g.turn_angle_deg) + " " +
      std::to_string(g.sequence_id) + " " + stamp(g.valid_until_ns) + " " +
      std::to_string(g.source_stale ? 1 : 0));
  }
  std::optional<TfPose> lookup_transform(
    const std::string & target, const std::string & source, std::string & error) override
  {
    try {
      const auto x = tf->lookupTransform(target, source, tf2::TimePointZero);
      TfPose o;
      o.x = x.transform.translation.x;
      o.y = x.transform.translation.y;
      o.qx = x.transform.rotation.x;
      o.qy = x.transform.rotation.y;
      o.qz = x.transform.rotation.z;
      o.qw = x.transform.rotation.w;
      return o;
    } catch (const tf2::TransformException & e) {
      error = e.what();
      return std::nullopt;
    }
  }
  void trace_reason(const std::string & reason) override {emit("RS " + reason);}
};

void apply_params(const JsonValue & obj, TurnParams & p)
{
  for (const auto & kv : obj.o) {
    const std::string & k = kv.first;
    const JsonValue & v = kv.second;
    double * d = nullptr;
    if (k == "odom_topic") {p.odom_topic = v.s; continue;}
    if (k == "rail_topic") {p.rail_topic = v.s; continue;}
    if (k == "map_frame") {p.map_frame = v.s; continue;}
    if (k == "base_frame") {p.base_frame = v.s; continue;}
    if (k == "enable_rail_forecast") {p.enable_rail_forecast = v.b; continue;}
    if (k == "window_sec") {d = &p.window_sec;} else if (k == "enter_threshold_deg") {
      d = &p.enter_threshold_deg;
    } else if (k == "exit_threshold_deg") {d = &p.exit_threshold_deg;} else if (k == "min_duration_sec") {
      d = &p.min_duration_sec;
    } else if (k == "odom_timeout_sec") {d = &p.odom_timeout_sec;} else if (k == "publish_rate_hz") {
      d = &p.publish_rate_hz;
    } else if (k == "cue_valid_sec") {d = &p.cue_valid_sec;} else if (k == "rail_timeout_sec") {
      d = &p.rail_timeout_sec;
    } else if (k == "rail_corner_threshold_deg") {d = &p.rail_corner_threshold_deg;} else if (
      k == "rail_lead_time_sec")
    {
      d = &p.rail_lead_time_sec;
    } else if (k == "rail_max_offtrack_m") {d = &p.rail_max_offtrack_m;} else if (k == "rail_handoff_m") {
      d = &p.rail_handoff_m;
    } else if (k == "rail_merge_gap_m") {d = &p.rail_merge_gap_m;} else if (k == "rail_goal_change_m") {
      d = &p.rail_goal_change_m;
    } else if (k == "rail_behind_start_m") {d = &p.rail_behind_start_m;} else if (
      k == "rail_min_prepare_dist_m")
    {
      d = &p.rail_min_prepare_dist_m;
    } else if (k == "rail_rehold_end_m") {d = &p.rail_rehold_end_m;} else if (
      k == "controller_lookahead_time_sec")
    {
      d = &p.controller_lookahead_time_sec;
    } else if (k == "controller_lookahead_min_m") {d = &p.controller_lookahead_min_m;} else if (
      k == "controller_lookahead_max_m")
    {
      d = &p.controller_lookahead_max_m;
    } else if (k == "minor_turn_deg") {d = &p.minor_turn_deg;} else if (k == "minor_turn_min_speed_mps") {
      d = &p.minor_turn_min_speed_mps;
    }
    if (d == nullptr) {
      std::fprintf(stderr, "모르는 파라미터: %s\n", k.c_str());
      std::exit(2);
    }
    *d = num(v);
  }
}

// line.split(' ') — 빈 칸도 남긴다(빈 frame_id 의 16진수는 빈 글자다).
std::vector<std::string> split(const std::string & line)
{
  std::vector<std::string> out;
  std::size_t start = 0;
  while (true) {
    const auto sp = line.find(' ', start);
    out.push_back(line.substr(start, sp == std::string::npos ? std::string::npos : sp - start));
    if (sp == std::string::npos) {break;}
    start = sp + 1;
  }
  return out;
}

double dbl(const std::string & s) {return std::strtod(s.c_str(), nullptr);}

}  // namespace

int main()
{
  std::string line;
  TurnParams params;
  RecordIo io;
  std::unique_ptr<TurnCore> core;
  while (std::getline(std::cin, line)) {
    if (line.empty()) {continue;}
    const auto sp = line.find(' ');
    const std::string cmd = line.substr(0, sp);
    try {
      if (cmd == "P") {
        const auto doc = py_json_loads(line.substr(2));
        if (!doc) {std::fprintf(stderr, "P 를 못 읽음\n"); return 2;}
        apply_params(*doc, params);
        continue;
      }
      const auto w = split(line);
      if (cmd == "T") {
        io.t = std::stoll(w[1]);
      } else if (cmd == "R") {
        io.r = std::stoll(w[1]);
      } else if (cmd == "start") {
        // 노드와 같은 순서(turn_guide_node.cpp 생성자). 다 끝나야 노드가 생긴다 — 도중에 멈추면 파이썬처럼
        // 노드가 없어 종료 IDLE 도 없다.
        auto fresh = std::make_unique<TurnCore>(params, io);
        if (fresh->rail_enabled()) {
          io.tf = std::make_unique<tf2::BufferCore>();
          fresh->log_rail_forecast_on();
        }
        fresh->publish_period_ns();
        fresh->log_started();
        core = std::move(fresh);
      } else if (cmd == "odom") {
        core->on_odom(dbl(w[1]), dbl(w[2]), dbl(w[3]), dbl(w[4]), dbl(w[5]));
      } else if (cmd == "rail") {
        const std::size_t n = std::stoul(w[2]);
        std::vector<Point> xy;
        for (std::size_t i = 0; i < n; ++i) {xy.emplace_back(dbl(w[3 + 2 * i]), dbl(w[4 + 2 * i]));}
        if (core->rail_enabled()) {core->on_rail(text(w[1]), xy);}
      } else if (cmd == "tf") {
        if (!io.tf) {continue;}
        geometry_msgs::msg::TransformStamped t;
        t.header.frame_id = text(w[1]);
        t.child_frame_id = text(w[2]);
        t.header.stamp.sec = static_cast<int32_t>(std::stol(w[3]));
        t.header.stamp.nanosec = static_cast<uint32_t>(std::stoul(w[4]));
        t.transform.translation.x = dbl(w[5]);
        t.transform.translation.y = dbl(w[6]);
        t.transform.translation.z = dbl(w[7]);
        t.transform.rotation.x = dbl(w[8]);
        t.transform.rotation.y = dbl(w[9]);
        t.transform.rotation.z = dbl(w[10]);
        t.transform.rotation.w = dbl(w[11]);
        io.tf->setTransform(t, "default_authority", w[12] == "1");
      } else if (cmd == "tick") {
        core->publish_loop();
      } else if (cmd == "idle") {
        core->publish_idle_once();
      } else {
        std::fprintf(stderr, "모르는 명령: %s\n", cmd.c_str());
        return 2;
      }
    } catch (const PyError & e) {
      emit("X " + e.type() + ": " + esc(e.what()));
      if (core) {
        try {
          core->publish_idle_once();
        } catch (...) {
        }
      }
      break;
    }
  }
  for (const auto & l : OUT) {std::cout << l << "\n";}
  return 0;
}
