/**
 * @file fusion_relay_prediction_test.cpp
 * @brief 验证接力覆盖估算纯函数：视线角速率与「视场宽度 ÷ 角速率」覆盖时长锚点。
 */

#include <gtest/gtest.h>

#include <cmath>

#include "fusion/FusionAcceptanceRecords.h"

namespace fusion {
namespace {

TEST(FusionRelayPredictionTest, AngularSpeedDegPerSec) {
  // 匀速直线扫掠：方位每秒 +2°，俯仰不动 → ω = 2°/s。
  EXPECT_NEAR(AngularSpeedDegPerSec(10.0, 5.0, 12.0, 5.0, 1.0), 2.0, 1.0e-9);
  // 两轴合成：Δ=(3°,4°) → |Δ|=5°，dt=0.5s → ω = 10°/s。
  EXPECT_NEAR(AngularSpeedDegPerSec(0.0, 0.0, 3.0, 4.0, 0.5), 10.0, 1.0e-9);
  // 角度不变 → ω = 0。
  EXPECT_DOUBLE_EQ(AngularSpeedDegPerSec(7.0, -3.0, 7.0, -3.0, 2.0), 0.0);
  // dt 非正：不可用约定返回 0。
  EXPECT_DOUBLE_EQ(AngularSpeedDegPerSec(0.0, 0.0, 5.0, 0.0, 0.0), 0.0);
}

TEST(FusionRelayPredictionTest, RelayCoverageSec) {
  // 20° 视场、2°/s 扫掠 → 覆盖 10s。
  EXPECT_NEAR(RelayCoverageSec(20.0, 2.0), 10.0, 1.0e-9);
  // 角速率近零或视场非正 → -1（不可用，调用方写无）。
  EXPECT_DOUBLE_EQ(RelayCoverageSec(20.0, 0.0), -1.0);
  EXPECT_DOUBLE_EQ(RelayCoverageSec(0.0, 2.0), -1.0);
}

}  // namespace
}  // namespace fusion
