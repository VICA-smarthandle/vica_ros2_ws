#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <deque>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "vica_docking/approach.hpp"
#include "vica_docking/dock_spot.hpp"
#include "vica_docking/undock.hpp"
#include "vica_docking/v_detector.hpp"

namespace vd = vica_docking;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr int kBeams = 720;
constexpr double kAmin = -kPi;
constexpr double kInc = 2.0 * kPi / kBeams;

struct Seg
{
  vd::Vec2 a, b;
};

struct Robot
{
  double x, y, th;
  vd::Vec2 pos() const {return {x, -y};}
  double heading() const {return kPi + th;}
};

std::vector<Seg> dock_segments(bool wall, std::optional<Seg> extra = std::nullopt)
{
  const double c = std::cos(kPi / 3.0) * 0.20, s = std::sin(kPi / 3.0) * 0.20;
  std::vector<Seg> v{{{0, 0}, {c, s}}, {{0, 0}, {c, -s}}};
  if (wall) {
    v.push_back({{0.0, 0.18}, {0.0, 1.2}});
    v.push_back({{0.0, -1.2}, {0.0, -0.18}});
  }
  if (extra) {
    v.push_back(*extra);
  }
  return v;
}

std::vector<float> synth_scan(const Robot & r, const std::vector<Seg> & segs, unsigned seed = 1)
{
  const vd::RobotGeometry g;
  const double h = r.heading();
  const vd::Vec2 o = r.pos() + vd::Vec2{
    std::cos(h) * g.laser_x - std::sin(h) * g.laser_y, std::sin(h) * g.laser_x + std::cos(h) * g.laser_y};
  std::vector<float> out(kBeams, std::numeric_limits<float>::infinity());
  unsigned s = seed * 2654435761u + 12345u;
  for (int i = 0; i < kBeams; ++i) {
    const double a = h + g.laser_yaw + kAmin + i * kInc;
    const vd::Vec2 d{std::cos(a), std::sin(a)};
    double best = 1e9;
    for (const auto & sg : segs) {
      const vd::Vec2 e = sg.b - sg.a;
      const double den = d.x * e.y - d.y * e.x;
      if (std::abs(den) < 1e-12) {
        continue;
      }
      const vd::Vec2 w = sg.a - o;
      const double t = (w.x * e.y - w.y * e.x) / den;
      const double u = (w.x * d.y - w.y * d.x) / den;
      if (t > 0 && u >= 0 && u <= 1 && t < best) {
        best = t;
      }
    }
    if (best < 12.0 && best > 0.15) {
      s = s * 1103515245u + 12345u;
      const double noise = ((s >> 8) % 3001) / 1e6 - 0.0015;
      out[i] = static_cast<float>(best + noise);
    }
  }
  return out;
}

struct Sim
{
  Robot r;
  double v{0}, w{0};
  double tau{0.27};
  void step(const vd::Cmd & c, double dt)
  {
    v += (c.v - v) * dt / tau;
    w += (c.w - w) * dt / 0.10;
    vd::Vec2 p = r.pos();
    double h = r.heading();
    p.x += v * std::cos(h) * dt;
    p.y += v * std::sin(h) * dt;
    h += w * dt;
    r.x = p.x;
    r.y = -p.y;
    r.th = vd::wrap_angle(h - kPi);
  }
};

}

TEST(VDetector, FindsDockPoseFromSyntheticScan)
{
  for (const auto & r : {Robot{1.15, 0.0, 0.0}, Robot{1.15, 0.20, vd::rad(15)},
      Robot{0.60, -0.10, vd::rad(-8)}, Robot{0.30, 0.005, vd::rad(0.5)}})
  {
    vd::VDetector det;
    const auto d = det.detect(synth_scan(r, dock_segments(true)), kAmin, kInc, 0.15);
    ASSERT_TRUE(d.has_value()) << r.x << " " << r.y;
    const auto p = vd::pose_in_dock(*d);
    EXPECT_NEAR(p.x, r.x, 0.004);
    EXPECT_NEAR(p.y, r.y, 0.004);
    EXPECT_NEAR(p.th, r.th, vd::rad(0.6));
    EXPECT_NEAR(d->inner_deg, 120.0, 2.0);
  }
}

TEST(VDetector, NothingWithoutDock)
{
  vd::VDetector det;
  const std::vector<Seg> wall_only{{{0.0, -1.2}, {0.0, 1.2}}};
  EXPECT_FALSE(det.detect(synth_scan({1.0, 0.0, 0.0}, wall_only), kAmin, kInc, 0.15).has_value());
}

TEST(DockSpot, DetectionToMapRoundTrip)
{
  const Robot r{1.10, 0.07, vd::rad(6)};
  vd::VDetector det;
  const auto d = det.detect(synth_scan(r, dock_segments(true)), kAmin, kInc, 0.15);
  ASSERT_TRUE(d.has_value());
  const vd::Pose2 robot_map{3.0, -1.5, 0.8};
  auto s = vd::spot_from_detection(robot_map, *d);
  const auto back = vd::spot_to_map(s, vd::pose_in_dock(*d));
  EXPECT_NEAR(back.x, robot_map.x, 1e-9);
  EXPECT_NEAR(back.y, robot_map.y, 1e-9);
  EXPECT_NEAR(vd::wrap_angle(back.yaw - robot_map.yaw), 0.0, 1e-9);
  const vd::Pose2 st = vd::staging_pose(s, 0.151);
  EXPECT_NEAR(std::hypot(st.x - s.apex_x, st.y - s.apex_y), 0.151 + 1.0, 1e-9);
  EXPECT_NEAR(vd::wrap_angle(st.yaw - s.fwd_yaw), 0.0, 1e-12);

  s.map_id = "test_map";
  s.saved_at = "2026-11-18T20:00:00";
  const std::string path = "/tmp/vica_docking_test_dock.yaml";
  ASSERT_TRUE(vd::write_dock_spot(path, s));
  const auto rd = vd::read_dock_spot(path);
  ASSERT_TRUE(rd.has_value());
  EXPECT_EQ(rd->map_id, "test_map");
  EXPECT_NEAR(rd->apex_x, s.apex_x, 1e-4);
  EXPECT_NEAR(rd->fwd_yaw, s.fwd_yaw, 1e-5);
  EXPECT_FALSE(rd->visited_ok);
  std::remove(path.c_str());
}

TEST(DockSpot, PoseRoundTripAndRefineFields)
{
  vd::DockSpot s;
  s.map_id = "m";
  s.apex_x = -7.03;
  s.apex_y = 2.43;
  s.fwd_yaw = 3.11;
  const vd::DockPose in_dock{0.151 + 0.14, -0.008, vd::rad(1.7)};
  const vd::Pose2 robot = vd::spot_to_map(s, in_dock);
  const vd::DockSpot back = vd::spot_from_pose(robot, in_dock);
  EXPECT_NEAR(back.apex_x, s.apex_x, 1e-9);
  EXPECT_NEAR(back.apex_y, s.apex_y, 1e-9);
  EXPECT_NEAR(vd::wrap_angle(back.fwd_yaw - s.fwd_yaw), 0.0, 1e-9);

  s.source = "map_pick";
  s.refined = true;
  s.refined_from = "map_pick";
  s.refined_cm = 18.2;
  s.refined_deg = 9.1;
  const std::string path = "/tmp/vica_dock_spot_refine_test.yaml";
  ASSERT_TRUE(vd::write_dock_spot(path, s));
  const auto r = vd::read_dock_spot(path);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->source, "map_pick");
  EXPECT_TRUE(r->refined);
  EXPECT_EQ(r->refined_from, "map_pick");
  EXPECT_NEAR(r->refined_cm, 18.2, 0.05);
  EXPECT_NEAR(r->refined_deg, 9.1, 0.05);
  std::remove(path.c_str());
  vd::DockSpot old;
  old.map_id = "m";
  ASSERT_TRUE(vd::write_dock_spot(path, old));
  const auto o = vd::read_dock_spot(path);
  ASSERT_TRUE(o.has_value());
  EXPECT_EQ(o->source, "v_detect");
  EXPECT_FALSE(o->refined);
  std::remove(path.c_str());
}

namespace
{
vd::ApproachController run_approach(
  Robot start, std::optional<Seg> extra, double t_end = 70.0, double extra_until = 1e9,
  double dead = 0.0, double extra_from = 0.0, bool dock_gone_after_from = false)
{
  vd::ApproachController c;
  Sim sim{start};
  c.start(0.0);
  const auto segs = dock_segments(true, extra);
  const auto clear = dock_segments(true);
  double t = 0.0, next_scan = 0.0, next_tick = 0.05;
  vd::Cmd cmd;
  std::deque<std::pair<double, vd::Cmd>> pending;
  unsigned seed = 1;
  while (t < t_end && !c.finished()) {
    if (t >= next_scan) {
      const bool has_extra = t >= extra_from && t < extra_until;
      const std::vector<Seg> gone{{{0.0, -1.2}, {0.0, 1.2}}};
      const auto & scene = (dock_gone_after_from && t >= extra_from) ? gone : (has_extra ? segs : clear);
      c.on_scan(t, synth_scan(sim.r, scene, seed++), kAmin, kInc, 0.15);
      next_scan += 1.0 / 12.1;
    }
    if (t >= next_tick) {
      pending.emplace_back(t + dead, c.on_timer(t));
      next_tick += 0.05;
    }
    while (!pending.empty() && pending.front().first <= t + 1e-9) {
      cmd = pending.front().second;
      pending.pop_front();
    }
    sim.step(cmd, 0.01);
    t += 0.01;
  }
  return c;
}
}

TEST(Approach, DocksFromStagingWithOffset)
{
  const auto c = run_approach({1.151, 0.15, vd::rad(12)}, std::nullopt);
  ASSERT_EQ(c.phase(), vd::ApproachPhase::kDone) << c.result().why;
  EXPECT_TRUE(c.result().success);
  EXPECT_LT(std::abs(c.result().err_x_cm), 1.5);
  EXPECT_LT(std::abs(c.result().err_y_cm), 1.5);
  EXPECT_LT(std::abs(c.result().err_th_deg), 3.0);
}

TEST(Approach, NoFalseNoMotionWhenAlignTurnsIntoApproach)
{
  int runs = 0;
  for (double y : {-0.10, -0.05, -0.02, 0.02, 0.05, 0.10}) {
    for (double th_deg : {-6.0, -2.0, 0.0, 2.0, 6.0}) {
      const auto c = run_approach({1.18, y, vd::rad(th_deg)}, std::nullopt, 70.0, 1e9, 0.20);
      EXPECT_NE(c.result().fail, vd::Fail::kNoMotion) << "y " << y << " th " << th_deg;
      const bool arrived = c.result().success || c.result().fail == vd::Fail::kMisaligned;
      EXPECT_TRUE(arrived) << "y " << y << " th " << th_deg << " " << c.result().why;
      EXPECT_LT(std::abs(c.result().err_x_cm), 2.0) << "y " << y << " th " << th_deg;
      ++runs;
    }
  }
  EXPECT_EQ(runs, 30);
}

const Seg kBox{{0.55, -0.21}, {0.55, -0.16}};

TEST(Approach, WaitsForFrontObstacleThenFailsAfter30s)
{
  const auto c = run_approach({1.151, 0.0, 0.0}, kBox, 80.0);
  EXPECT_EQ(c.phase(), vd::ApproachPhase::kAbort);
  EXPECT_EQ(c.result().fail, vd::Fail::kBlocked);
  EXPECT_EQ(c.result().front_holds, 1);
  ASSERT_TRUE(c.last_pose().has_value());
  EXPECT_GT(c.last_pose()->x - 0.151, 0.55);
}

TEST(Approach, ResumesAfterFrontObstacleLeaves)
{
  const auto c = run_approach({1.151, 0.0, 0.0}, kBox, 80.0, 15.0);
  ASSERT_EQ(c.phase(), vd::ApproachPhase::kDone) << c.result().why;
  EXPECT_TRUE(c.result().success);
  EXPECT_EQ(c.result().front_holds, 1);
}

const Seg kHide{{0.45, -0.22}, {0.45, 0.22}};

TEST(Approach, WaitsWhenObstacleHidesDockThenResumes)
{
  const auto c = run_approach({1.151, 0.0, 0.0}, kHide, 80.0, 15.0, 0.0, 2.0);
  ASSERT_EQ(c.phase(), vd::ApproachPhase::kDone) << c.result().why;
  EXPECT_TRUE(c.result().success);
  EXPECT_GE(c.result().front_holds, 1);
}

TEST(Approach, HiddenDockFailsAsBlockedAfter30s)
{
  const auto c = run_approach({1.151, 0.0, 0.0}, kHide, 80.0, 1e9, 0.0, 2.0);
  EXPECT_EQ(c.phase(), vd::ApproachPhase::kAbort);
  EXPECT_EQ(c.result().fail, vd::Fail::kBlocked) << c.result().why;
  ASSERT_TRUE(c.last_pose().has_value());
  EXPECT_GT(c.last_pose()->x - 0.151, 0.45 + 0.30);
}

TEST(Approach, LostAfter3sWhenDockVanishes)
{
  const auto at5 = run_approach({1.151, 0.0, 0.0}, std::nullopt, 5.5, 1e9, 0.0, 3.0, true);
  EXPECT_FALSE(at5.finished());
  const auto at7 = run_approach({1.151, 0.0, 0.0}, std::nullopt, 7.0, 1e9, 0.0, 3.0, true);
  ASSERT_TRUE(at7.finished()) << "phase " << vd::phase_name(at7.phase()) << " blocked " << at7.front_blocked()
                              << " holds " << at7.result().front_holds << " hits " << at7.front_hits();
  EXPECT_EQ(at7.result().fail, vd::Fail::kLost) << at7.result().why;
}

TEST(Approach, LostNotBlockedWhenDockLostUpClose)
{
  const auto full = dock_segments(true);
  const std::vector<Seg> half{full[0], full[2], full[3]};
  {
    vd::VDetector d;
    ASSERT_FALSE(d.detect(synth_scan({0.45, 0.0, 0.0}, half), kAmin, kInc, 0.15).has_value());
  }
  vd::ApproachController c;
  Sim sim{{1.151, 0.0, 0.0}};
  c.start(0.0);
  double t = 0.0, next_scan = 0.0, next_tick = 0.05, t_lost = -1.0;
  vd::Cmd cmd;
  unsigned seed = 1;
  while (!c.finished() && t < 60.0 && (t_lost < 0 || t < t_lost + 10.0)) {
    if (t_lost < 0 && sim.r.x - 0.151 < 0.30) {
      t_lost = t;
    }
    if (t >= next_scan) {
      c.on_scan(t, synth_scan(sim.r, t_lost < 0 ? full : half, seed++), kAmin, kInc, 0.15);
      next_scan += 1.0 / 12.1;
    }
    if (t >= next_tick) {
      cmd = c.on_timer(t);
      next_tick += 0.05;
    }
    sim.step(cmd, 0.01);
    t += 0.01;
  }
  ASSERT_GT(t_lost, 0.0);
  ASSERT_TRUE(c.finished()) << "blocked " << c.front_blocked() << " hits " << c.front_hits();
  EXPECT_EQ(c.result().fail, vd::Fail::kLost) << c.result().why;
  EXPECT_LT(t - t_lost, 4.5);
  EXPECT_GT(sim.r.x - 0.151, 0.30 - 0.05) << "범퍼~꼭짓점 " << (sim.r.x - 0.151);
}

TEST(Approach, RefusesStartFarOffAxis)
{
  const auto c = run_approach({1.151, 0.40, 0.0}, std::nullopt, 10.0);
  EXPECT_EQ(c.result().fail, vd::Fail::kOutOfRange);
}

TEST(Approach, NotFoundWithoutDock)
{
  vd::ApproachController c;
  c.start(0.0);
  const std::vector<float> empty(kBeams, std::numeric_limits<float>::infinity());
  for (double t = 0.0; t < 6.0 && !c.finished(); t += 0.05) {
    c.on_scan(t, empty, kAmin, kInc, 0.15);
    c.on_timer(t);
  }
  EXPECT_EQ(c.result().fail, vd::Fail::kNotFound);
}

namespace
{
struct UndockRun
{
  vd::UndockPhase phase;
  vd::UndockResult res;
  double first_move_t;
};

UndockRun run_undock(double rear_block_until)
{
  vd::UndockController c;
  Sim sim{{0.151 + 0.14, 0.0, 0.0}};
  sim.tau = 0.20;
  c.start(0.0);
  const auto segs = dock_segments(true);
  double t = 0.0, next_scan = 0.0, next_tick = 0.05, next_gyro = 0.0, first_move = -1.0;
  double next_us[4] = {0.0, 0.1, 0.25, 0.35};
  vd::Cmd cmd;
  unsigned seed = 7;
  while (t < 130.0 && !c.finished()) {
    if (t >= next_scan) {
      c.on_scan(t, synth_scan(sim.r, segs, seed++), kAmin, kInc, 0.15);
      next_scan += 1.0 / 12.1;
    }
    if (t >= next_gyro) {
      c.on_gyro(t, sim.w);
      next_gyro += 0.02;
    }
    for (int i = 0; i < 4; ++i) {
      if (t >= next_us[i]) {
        const double r = (i == vd::kRearRight && t < rear_block_until) ? 0.50 : 0.80;
        c.on_ultrasonic(i, t, r, 0.80);
        next_us[i] += 0.5;
      }
    }
    if (t >= next_tick) {
      cmd = c.on_timer(t);
      if (first_move < 0 && std::abs(cmd.v) > 1e-3) {
        first_move = t;
      }
      next_tick += 0.05;
    }
    sim.step(cmd, 0.01);
    t += 0.01;
  }
  return {c.phase(), c.result(), first_move};
}
}

TEST(Undock, BacksTo85cmThenTurns180)
{
  const auto r = run_undock(-1.0);
  ASSERT_EQ(r.phase, vd::UndockPhase::kDone) << r.res.why;
  EXPECT_NEAR(r.res.back_cm, 85.0 - 14.0, 3.0);
  EXPECT_NEAR(r.res.turned_deg, 180.0, 6.0);
}

TEST(Undock, WaitsWhileRearBlockedThenGoes)
{
  const auto r = run_undock(4.0);
  ASSERT_EQ(r.phase, vd::UndockPhase::kDone) << r.res.why;
  EXPECT_GE(r.first_move_t, 4.0 + 1.0 - 0.6);
}

namespace
{
double drive_undock(vd::UndockController & c, Sim & sim, double t, double t_stop)
{
  const auto segs = dock_segments(true);
  double next_scan = t, next_tick = t + 0.05, next_gyro = t;
  double next_us[4] = {t, t + 0.1, t + 0.25, t + 0.35};
  vd::Cmd cmd;
  unsigned seed = 11;
  while (t < t_stop && !c.finished()) {
    if (t >= next_scan) {
      c.on_scan(t, synth_scan(sim.r, segs, seed++), kAmin, kInc, 0.15);
      next_scan += 1.0 / 12.1;
    }
    if (t >= next_gyro) {
      c.on_gyro(t, sim.w);
      next_gyro += 0.02;
    }
    for (int i = 0; i < 4; ++i) {
      if (t >= next_us[i]) {
        c.on_ultrasonic(i, t, 0.80, 0.80);
        next_us[i] += 0.5;
      }
    }
    if (t >= next_tick) {
      cmd = c.on_timer(t);
      next_tick += 0.05;
    }
    sim.step(cmd, 0.01);
    t += 0.01;
  }
  for (int k = 0; k < 100; ++k) {
    sim.step({}, 0.01);
    t += 0.01;
  }
  return t;
}
}

TEST(Undock, ResumesRemainingTurnAfterStopMidTurn)
{
  Sim sim{{0.151 + 0.14, 0.0, 0.0}};
  sim.tau = 0.20;
  vd::UndockController first;
  first.start(0.0);
  double t = drive_undock(first, sim, 0.0, 16.0);
  ASSERT_EQ(first.phase(), vd::UndockPhase::kTurn);
  first.abort(vd::Fail::kEstop, "비상정지");
  EXPECT_EQ(first.abort_phase(), vd::UndockPhase::kTurn);
  const double remaining = first.turn_remaining_deg();
  EXPECT_GT(remaining, 40.0);
  EXPECT_LT(remaining, 140.0);

  vd::UndockController second;
  second.start_turn_only(t, remaining);
  drive_undock(second, sim, t, t + 60.0);
  ASSERT_EQ(second.phase(), vd::UndockPhase::kDone) << second.result().why;
  EXPECT_TRUE(second.result().success);
  EXPECT_NEAR(std::abs(vd::deg(vd::wrap_angle(sim.r.th))), 180.0, 8.0);
  EXPECT_NEAR(sim.r.x, 0.151 + 0.85, 0.06);
}

TEST(Undock, ResumeReportsPhaseAndRemaining)
{
  Sim sim{{0.151 + 0.14, 0.0, 0.0}};
  sim.tau = 0.20;
  vd::UndockController c;
  c.start(0.0);
  drive_undock(c, sim, 0.0, 5.0);
  ASSERT_EQ(c.phase(), vd::UndockPhase::kBack);
  c.abort(vd::Fail::kCanceled, "사용자가 멈춤");
  EXPECT_EQ(c.abort_phase(), vd::UndockPhase::kBack);
  EXPECT_EQ(c.turn_remaining_deg(), 0.0);

  vd::UndockController done;
  done.start_turn_only(0.0, 9.5);
  EXPECT_EQ(done.phase(), vd::UndockPhase::kDone);
  EXPECT_TRUE(done.result().success);
  vd::UndockController back;
  back.start_turn_only(0.0, -8.0);
  EXPECT_EQ(back.phase(), vd::UndockPhase::kDone);
  vd::UndockController more;
  more.start_turn_only(0.0, 12.0);
  EXPECT_EQ(more.phase(), vd::UndockPhase::kTurn);
}
