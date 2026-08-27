/**
 * @file common_earth_occultation_test.cpp
 * @brief 验证有限视线弦与地球圆球的遮挡判定（common 单源）。
 */

#include <cmath>

#include <gtest/gtest.h>

#include "1q/coordinate/types.h"
#include "common/geometry/EarthOccultation.h"

namespace oneq {
namespace common {
namespace geometry {
namespace {

using oneq::coordinate::EcefPositionM;

constexpr double kEarthRadiusM = kMeanEarthRadiusM;

TEST(CommonEarthOccultationTest, FiniteSegmentBehindEarthIsOcculted) {
  const EcefPositionM observer(7000000.0, 0.0, 0.0);
  const EcefPositionM target(-7000000.0, 0.0, 0.0);
  EXPECT_TRUE(IsEarthOcculted(observer, target, kEarthRadiusM));
}

TEST(CommonEarthOccultationTest, OutwardLineOfSightIsNotOcculted) {
  const EcefPositionM observer(7000000.0, 0.0, 0.0);
  const EcefPositionM target(8000000.0, 0.0, 0.0);
  EXPECT_FALSE(IsEarthOcculted(observer, target, kEarthRadiusM));
}

TEST(CommonEarthOccultationTest, MarginIsNegativeWhenOcculted) {
  // 对侧地表上方：LOS 穿过地球 → 余量为负（遮挡深度）。
  const EcefPositionM observer(7000000.0, 0.0, 0.0);
  const EcefPositionM target(-6500000.0, 0.0, 0.0);
  EXPECT_LT(ComputeEarthOccultationMarginM(observer, target, kEarthRadiusM), 0.0);
}

TEST(CommonEarthOccultationTest, MarginIsPositiveWhenClear) {
  const EcefPositionM observer(7000000.0, 0.0, 0.0);
  const EcefPositionM target(8000000.0, 0.0, 0.0);
  EXPECT_GT(ComputeEarthOccultationMarginM(observer, target, kEarthRadiusM), 0.0);
}

TEST(CommonEarthOccultationTest, GrazingTangentIsOcculted) {
  // 从 (2R, 0, 0) 引出的切线切点为 (R/2, R√3/2, 0)；目标取切点镜像，使切点落在线段内。
  const double r = kEarthRadiusM;
  const EcefPositionM observer(2.0 * r, 0.0, 0.0);
  const EcefPositionM target(-r, r * std::sqrt(3.0), 0.0);
  EXPECT_TRUE(IsEarthOcculted(observer, target, r));
  EXPECT_NEAR(ComputeEarthOccultationMarginM(observer, target, r), 0.0, 1.0e-6);
}

TEST(CommonEarthOccultationTest, SurfaceTargetAtSegmentEndIsNotOcculted) {
  // 观测点在球外沿径向看向球面上的落点：最近点是终点，段外判定 → 不遮挡。
  const EcefPositionM observer(kEarthRadiusM + 1000.0, 0.0, 0.0);
  const EcefPositionM target(kEarthRadiusM, 0.0, 0.0);
  EXPECT_FALSE(IsEarthOcculted(observer, target, kEarthRadiusM));
  EXPECT_DOUBLE_EQ(ComputeEarthOccultationMarginM(observer, target, kEarthRadiusM), kEarthRadiusM);
}

}  // namespace
}  // namespace geometry
}  // namespace common
}  // namespace oneq
