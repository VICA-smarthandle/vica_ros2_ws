#include <gtest/gtest.h>
#include <cmath>
#include "vica_vcc_controller/core/clearance.hpp"
#include "vica_vcc_controller/core/geometry.hpp"

using namespace vica_vcc_controller::core;

namespace
{
const Polygon kFootprint{{0.151, 0.225}, {0.151, -0.225}, {-0.459, -0.225},
  {-0.569, -0.035}, {-0.569, 0.035}, {-0.459, 0.225}};

// 원점 중심 5 x 5 m, 0.05 m 격자
ClearanceField makeField()
{
  ClearanceField f;
  f.setFootprint(padFootprint(kFootprint, 0.05));
  f.grid().reset(-2.5, -2.5, 0.05, 100, 100);
  return f;
}
void markRectangle(ClearanceField & f, double x0, double x1, double y0, double y1)
{
  for (int ix = 0; ix < 100; ++ix) {
    for (int iy = 0; iy < 100; ++iy) {
      const double cx = -2.5 + (ix + 0.5) * 0.05, cy = -2.5 + (iy + 0.5) * 0.05;
      if (cx >= x0 && cx <= x1 && cy >= y0 && cy <= y1) {f.grid().markLethal(ix, iy);}
    }
  }
  f.grid().compute();
}
}  // namespace

TEST(Clearance, EmptyWorldIsFar)
{
  ClearanceField f = makeField();
  f.grid().compute();
  EXPECT_GT(f.clearance({0, 0, 0}), 5.0);
}

TEST(Clearance, WallBesideRobotMeasuresFromBodyEdge)
{
  // 셀 중심 y = 1.025 줄이 벽. 몸 옆면(padding 포함) y = 0.275.
  ClearanceField f = makeField();
  markRectangle(f, -2.5, 2.5, 1.0, 1.05);
  EXPECT_NEAR(f.clearance({0, 0, 0}), 1.025 - 0.275 - 0.025, 0.04);
  // 90도 돌면 앞면(0.201)이 벽을 본다
  EXPECT_NEAR(f.clearance({0, 0, M_PI / 2}), 1.025 - 0.201 - 0.025, 0.04);
}

TEST(Clearance, LethalInsideBodyIsContact)
{
  ClearanceField f = makeField();
  markRectangle(f, -0.1, 0.0, -0.05, 0.05);   // 몸 한가운데 작은 물체
  EXPECT_LT(f.clearance({0, 0, 0}), 0.0);
}

TEST(Clearance, InflationSizedGapHasNoBlindSpot)
{
  // backlog NAV2-B3: 벽에서 0.55~0.625 떨어진 띠는 inflation 비용 0 이라 비용 기반 검사가 몸통을 건너뛴다.
  // 몸 뒤쪽 모서리가 벽에 닿는 자세를 만들고, 중심은 벽에서 0.58 m 떨어뜨린다.
  ClearanceField f = makeField();
  markRectangle(f, -2.5, 2.5, 0.575, 0.625);  // 벽 셀 중심 ≈ 0.575~0.625
  // 90도 회전하면 뒤쪽 꼭짓점(-0.619)이 +y 쪽으로 간다 → 벽을 뚫는다
  EXPECT_LT(f.clearance({0, 0, -M_PI / 2}), 0.0);
}

TEST(Clearance, UltrasonicPointsCountAsObstacles)
{
  ClearanceField f = makeField();
  f.grid().compute();
  f.setPoints({{0.5, 0.0}});
  EXPECT_NEAR(f.clearance({0, 0, 0}), 0.5 - 0.201, 1e-6);
  f.setPoints({{0.0, 0.0}});
  EXPECT_LT(f.clearance({0, 0, 0}), 0.0);
}
