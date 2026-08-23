/*
 * @file fusion_relay_prediction_test.cpp
 * @brief 验证接力覆盖估算纯函数：方位差分归一、滑窗最小二乘角速率与
 *        「(视场宽度 − 已扫过角) ÷ 角速率」剩余覆盖锚点。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "fusion/FusionAcceptanceRecords.h"

namespace fusion {
namespace {

RelayAngularSample MakeSample(double t_sec, double az_deg, double el_deg) {
  RelayAngularSample sample;
  sample.time_sec = t_sec;
  sample.az_deg = az_deg;
  sample.el_deg = el_deg;
  return sample;
}

TEST(FusionRelayPredictionTest, WrappedAzimuthDeltaDeg) {
  // 普通差分原样返回。
  EXPECT_NEAR(WrappedAzimuthDeltaDeg(10.0, 12.0), 2.0, 1.0e-9);
  // 跨 ±180 缠绕：179°→−179° 真实 +2°，不产生 −358° 假差。
  EXPECT_NEAR(WrappedAzimuthDeltaDeg(179.0, -179.0), 2.0, 1.0e-9);
  EXPECT_NEAR(WrappedAzimuthDeltaDeg(-170.0, 170.0), -20.0, 1.0e-9);
  // 恰好 ±180：约定归到 (−180, 180] 的 +180 侧。
  EXPECT_NEAR(WrappedAzimuthDeltaDeg(0.0, 180.0), 180.0, 1.0e-9);
}

TEST(FusionRelayPredictionTest, LeastSquaresAngularSpeedFitsConstantSweep) {
  // 匀速扫掠：方位每秒 +2°、俯仰不动 → 最小二乘斜率 ω = 2°/s。
  std::vector<RelayAngularSample> samples;
  for (int i = 0; i < 8; ++i) {
    samples.push_back(MakeSample(static_cast<double>(i), 10.0 + 2.0 * i, 5.0));
  }
  EXPECT_NEAR(LeastSquaresAngularSpeedDegPerSec(samples), 2.0, 1.0e-9);
}

TEST(FusionRelayPredictionTest, LeastSquaresAngularSpeedCombinesAxes) {
  // 两轴合成：方位 +3°/s、俯仰 +4°/s → ω = 5°/s。
  std::vector<RelayAngularSample> samples;
  for (int i = 0; i < 6; ++i) {
    const double t = 0.5 * static_cast<double>(i);
    samples.push_back(MakeSample(t, 3.0 * t, 4.0 * t));
  }
  EXPECT_NEAR(LeastSquaresAngularSpeedDegPerSec(samples), 5.0, 1.0e-9);
}

TEST(FusionRelayPredictionTest, LeastSquaresAngularSpeedUnwrapsAzimuth) {
  // 匀速 +2°/s 跨 ±180：原始方位 179→181 应写 −179（缠绕表示）——解缠后
  // 斜率仍为 +2°/s（逐拍差分会得 −358/1 的荒谬速率）。
  std::vector<RelayAngularSample> samples;
  for (int i = 0; i < 8; ++i) {
    const double az = 175.0 + 2.0 * i;  // 175→189
    samples.push_back(MakeSample(static_cast<double>(i),
                                 az > 180.0 ? az - 360.0 : az, 0.0));
  }
  EXPECT_NEAR(LeastSquaresAngularSpeedDegPerSec(samples), 2.0, 1.0e-9);
}

TEST(FusionRelayPredictionTest, LeastSquaresAngularSpeedUnavailableCases) {
  // 单采样 / 全部同拍：不可用返回 -1。
  std::vector<RelayAngularSample> single{MakeSample(0.0, 10.0, 5.0)};
  EXPECT_DOUBLE_EQ(LeastSquaresAngularSpeedDegPerSec(single), -1.0);
  std::vector<RelayAngularSample> same_time{MakeSample(3.0, 10.0, 5.0),
                                            MakeSample(3.0, 12.0, 5.0)};
  EXPECT_DOUBLE_EQ(LeastSquaresAngularSpeedDegPerSec(same_time), -1.0);
  // 空窗：不可用。
  EXPECT_DOUBLE_EQ(LeastSquaresAngularSpeedDegPerSec({}), -1.0);
}

TEST(FusionRelayPredictionTest, RelayRemainingCoverageSec) {
  // 20° 视场已扫 5°、2°/s → 剩余 7.5s。
  EXPECT_NEAR(RelayRemainingCoverageSec(20.0, 5.0, 2.0), 7.5, 1.0e-9);
  // 已扫满视场 → 夹取 0（不下穿）。
  EXPECT_DOUBLE_EQ(RelayRemainingCoverageSec(20.0, 20.0, 2.0), 0.0);
  EXPECT_DOUBLE_EQ(RelayRemainingCoverageSec(20.0, 25.0, 2.0), 0.0);
  // 角速率不可用（-1/0）或视场非正 → -1（调用方写无）。
  EXPECT_DOUBLE_EQ(RelayRemainingCoverageSec(20.0, 5.0, -1.0), -1.0);
  EXPECT_DOUBLE_EQ(RelayRemainingCoverageSec(20.0, 5.0, 0.0), -1.0);
  EXPECT_DOUBLE_EQ(RelayRemainingCoverageSec(0.0, 5.0, 2.0), -1.0);
}

}  // namespace
}  // namespace fusion
