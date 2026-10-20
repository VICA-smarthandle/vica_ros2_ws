#include <gtest/gtest.h>
#include <cmath>
#include "vica_vcc_controller/core/clearance.hpp"
#include "vica_vcc_controller/core/geometry.hpp"

using namespace vica_vcc_controller::core;

namespace
{
const Polygon kFootprint{{0.151, 0.225}, {0.151, -0.225}, {-0.459, -0.225},
  {-0.569, -0.035}, {-0.569, 0.035}, {-0.459, 0.225}};

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
}

TEST(Clearance, EmptyWorldIsFar)
{
  ClearanceField f = makeField();
  f.grid().compute();
  EXPECT_GT(f.clearance({0, 0, 0}), 5.0);
}

TEST(Clearance, WallBesideRobotMeasuresFromBodyEdge)
{
  ClearanceField f = makeField();
  markRectangle(f, -2.5, 2.5, 1.0, 1.05);
  EXPECT_NEAR(f.clearance({0, 0, 0}), 1.025 - 0.275 - 0.025, 0.04);
  EXPECT_NEAR(f.clearance({0, 0, M_PI / 2}), 1.025 - 0.201 - 0.025, 0.04);
}

TEST(Clearance, LethalInsideBodyIsContact)
{
  ClearanceField f = makeField();
  markRectangle(f, -0.1, 0.0, -0.05, 0.05);
  EXPECT_LT(f.clearance({0, 0, 0}), 0.0);
}

TEST(Clearance, InflationSizedGapHasNoBlindSpot)
{
  ClearanceField f = makeField();
  markRectangle(f, -2.5, 2.5, 0.575, 0.625);
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

TEST(Clearance, PoseFarOutsideWindowIsFreeAndCheap)
{
  ClearanceField f = makeField();
  f.grid().markLethal(50, 50);
  f.grid().compute();
  EXPECT_GT(f.clearance({1000.0, -1000.0, 0.3}), 5.0);

  ClearanceField f2 = makeField();
  markRectangle(f2, 2.3, 2.45, -0.05, 0.05);
  EXPECT_LT(f2.clearance({2.4, 0.0, 0.0}), 0.0);
}
