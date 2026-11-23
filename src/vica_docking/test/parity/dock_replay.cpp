#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "vica_docking/approach.hpp"
#include "vica_docking/v_detector.hpp"

namespace vd = vica_docking;

namespace
{
template<typename T>
bool rd(T & v)
{
  return static_cast<bool>(std::cin.read(reinterpret_cast<char *>(&v), sizeof(T)));
}
}

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr, "usage: dock_replay vdet|approach [key=val ...]\n");
    return 2;
  }
  const std::string mode = argv[1];
  std::map<std::string, double> kv;
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    const auto eq = a.find('=');
    if (eq != std::string::npos) {
      kv[a.substr(0, eq)] = std::stod(a.substr(eq + 1));
    }
  }
  char magic[4];
  if (!std::cin.read(magic, 4) || std::memcmp(magic, "VDR1", 4) != 0) {
    std::fprintf(stderr, "bad magic\n");
    return 2;
  }
  int32_t n = 0, nb = 0;
  double amin = 0, inc = 0, rmin = 0;
  if (!rd(n) || !rd(nb) || !rd(amin) || !rd(inc) || !rd(rmin)) {
    return 2;
  }
  std::vector<float> r(static_cast<size_t>(nb));

  if (mode == "vdet") {
    vd::VDetector det;
    for (int i = 0; i < n; ++i) {
      double t = 0;
      if (!rd(t) || !std::cin.read(reinterpret_cast<char *>(r.data()), sizeof(float) * nb)) {
        return 2;
      }
      const auto d = det.detect(r, amin, inc, rmin);
      if (!d) {
        std::printf("%d 0\n", i);
        continue;
      }
      const auto p = vd::pose_in_dock(*d);
      std::printf(
        "%d 1 %.9g %.9g %.9g %.9g %.9g %.9g %d %d %.9g %.9g %.9g %.9g %.9g\n", i, p.x, p.y, p.th,
        d->inner_deg, d->la, d->lb, d->na, d->nb, d->apex.x, d->apex.y, d->u.x, d->u.y, d->score);
    }
    return 0;
  }

  if (mode == "approach") {
    vd::ApproachParams ap;
    auto get = [&kv](const char * k, double & dst) {
        if (kv.count(k)) {
          dst = kv[k];
        }
      };
    get("target_cm", ap.target_cm);
    get("stage_cm", ap.stage_cm);
    get("lead", ap.lead);
    get("vmax", ap.vmax);
    get("vfinal", ap.vfinal);
    get("wmax", ap.wmax);
    get("k_b", ap.k_b);
    get("deadband_deg", ap.deadband_deg);
    get("align_tol_deg", ap.align_tol_deg);
    get("max_time", ap.max_time);
    get("start_max_lateral", ap.start_max_lateral);
    get("start_max_heading_deg", ap.start_max_heading_deg);
    get("find_timeout", ap.find_timeout);
    get("no_motion_s", ap.no_motion_s);
    vd::ApproachController c(ap);
    c.start(0.0);
    double tick = 0.05;
    for (int i = 0; i < n; ++i) {
      double t = 0;
      if (!rd(t) || !std::cin.read(reinterpret_cast<char *>(r.data()), sizeof(float) * nb)) {
        return 2;
      }
      while (tick < t) {
        c.on_timer(tick);
        tick += 0.05;
      }
      c.on_scan(t, r, amin, inc, rmin);
      const auto cmd = c.planned_cmd();
      std::printf(
        "%d %s %.9g %.9g %d %d\n", i, vd::phase_name(c.phase()), cmd.v, cmd.w, c.front_hits(),
        c.front_blocked() ? 1 : 0);
    }
    const auto & res = c.result();
    std::printf(
      "RESULT %s %d %s %.9g %.9g %.9g front_holds=%d\n", vd::phase_name(c.phase()),
      res.success ? 1 : 0, vd::fail_name(res.fail), res.err_x_cm, res.err_y_cm, res.err_th_deg,
      res.front_holds);
    return 0;
  }
  std::fprintf(stderr, "unknown mode %s\n", mode.c_str());
  return 2;
}
