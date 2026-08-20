// Copyright 2026. All Rights Reserved.
//
// @file common_radar_equations_test.cpp
// @brief 验证 common 雷达方程纯函数基本行为。

#include <gtest/gtest.h>

#include <cmath>

#include "common/radar/RadarEquations.h"

namespace oneq {
namespace common {
namespace radar {
namespace {

TEST(CommonRadarEquationsTest, EchoPowerWithGainReturnsExpectedSignatures) {
  // 大距离/非正 RCS 返回 -300 dBW 保护值。
  EXPECT_FLOAT_EQ(RadarEquations::ComputeEchoPowerWithGain_dBW(
                      1.0e6f, 0.0f, 13.0e-6f, 9.5e9f, 38.0f, -1.0f, 1000.0f, 0.0f),
                  -300.0f);
  EXPECT_LT(RadarEquations::ComputeEchoPowerWithGain_dBW(
                1.0e6f, 0.0f, 13.0e-6f, 9.5e9f, 38.0f, 1.0f, 1000.0f, 0.0f),
            0.0f);
}

TEST(CommonRadarEquationsTest, ThermalNoiseAndIntegrationGain) {
  const float noise_w = RadarEquations::ComputeThermalNoisePower_W(10.0e6f, 3.0f);
  EXPECT_GT(noise_w, 0.0f);
  EXPECT_FLOAT_EQ(RadarEquations::ComputeIntegrationGain(10), 10.0f);
  EXPECT_FLOAT_EQ(RadarEquations::ComputeIntegrationGain(0), 1.0f);
}

TEST(CommonRadarEquationsTest, DetectionProbabilityInUnitInterval) {
  const float pd = RadarEquations::ComputeDetectionProbability(
      10.0f, 1e-6f, SwerlingModel::kSwerling0, 1);
  EXPECT_GE(pd, 0.0f);
  EXPECT_LE(pd, 1.0f);
}

}  // namespace
}  // namespace radar
}  // namespace common
}  // namespace oneq
