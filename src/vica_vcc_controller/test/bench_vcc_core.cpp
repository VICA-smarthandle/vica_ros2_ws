// 젯슨에서 VCC 한 주기 계산 시간을 잰다. 로봇을 움직이지 않는다.
// 합격: p99 <= 19.1 ms (100 ms 주기의 19.1 % = 사용자 허용 DWB 수준, 설계서 9절)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>
#include "vica_vcc_controller/core/geometry.hpp"
#include "vica_vcc_controller/core/vcc_core.hpp"

using namespace vica_vcc_controller::core;

int main()
{
  const Polygon fp{{0.151, 0.225}, {0.151, -0.225}, {-0.459, -0.225},
    {-0.569, -0.035}, {-0.569, 0.035}, {-0.459, 0.225}};
  CoreParams p;
  p.footprint = padFootprint(fp, 0.05);
  VccCore core;
  core.configure(p);

  ClearanceField field;
  field.setFootprint(p.footprint);
  Path path;
  for (double x = 0.0; x <= 3.0; x += 0.05) {path.push_back({x, 0.0, 0.0});}

  std::vector<double> ms;
  double v = 0.4;
  double y = 0.0;   // 레일에서 로봇의 옆 위치. 로봇은 자기 차선을 따라간다고 본다.
  for (int i = 0; i < 1000; ++i) {
    const auto t0 = std::chrono::steady_clock::now();
    // 매 주기 거리장을 새로 만든다(실제 플러그인과 같은 일). 로봇 좌표계라 세상은 -y 만큼 옮겨 놓는다
    // (로봇이 서 있기만 하면 d 재동기(I4)가 d 를 계속 0 으로 되돌려 계산이 싸게 나온다).
    const int sh = static_cast<int>(std::lround(-y / 0.05));
    field.grid().reset(-2.5, -2.5, 0.05, 100, 100);
    for (int ix = 0; ix < 100; ++ix) {field.grid().markLethal(ix, 20 + sh); field.grid().markLethal(ix, 80 + sh);}
    for (int iy = 48; iy < 52; ++iy) {field.grid().markLethal(74 + (i % 5), iy + sh);}   // 움직이는 기둥
    field.grid().compute();
    Path rail = path;
    for (auto & q : rail) {q.y = -y;}
    CoreInputs in;
    in.now = 0.1 * i; in.dt = 0.1; in.path = rail; in.goal = {10, -y, 0};
    in.measured = {v, 0.0}; in.speed_cap = 0.5;
    in.clearance = [&field](const Pose2D & q) {return field.clearance(q);};
    const CoreOutput out = core.step(in);
    v = out.cmd.v;
    y = out.offset;
    const auto t1 = std::chrono::steady_clock::now();
    ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
  }
  std::sort(ms.begin(), ms.end());
  std::printf("VCC step ms: p50=%.2f p90=%.2f p99=%.2f max=%.2f\n",
    ms[500], ms[900], ms[990], ms.back());
  return ms[990] <= 19.1 ? 0 : 1;
}
