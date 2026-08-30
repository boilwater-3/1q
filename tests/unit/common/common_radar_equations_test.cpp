// Copyright 2026. All Rights Reserved.
//
// @file common_radar_equations_test.cpp
// @brief 验证 common 雷达方程纯函数基本行为。

#include <gtest/gtest.h>

#include <cmath>

#include "common/numerics/Constants.h"
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

TEST(CommonRadarEquationsTest, RangeStdDevIsPureRandomTerm) {
  // SNR=20dB, BW=1MHz：δ_R = c/(2B) ≈ 149.896 m，σ = 0.5·δ_R/√100 = 7.495 m。
  // 固定 20 m 偏置已于 2026-08-30 拆出（kRangeMeasurementBiasM），不再并入 std。
  const float std_dev = RadarEquations::ComputeRangeErrorStdDev(20.0f, 1e6f);
  const float range_resolution =
      0.5f * static_cast<float>(oneq::common::numerics::kLightSpeed) / 1.0e6f;
  EXPECT_NEAR(std_dev, 0.5f * range_resolution / 10.0f, 1.0e-4f);
}

TEST(CommonRadarEquationsTest, RangeStdDevLowSnrFloorExcludesBias) {
  // snr<-10dB 下限分支保持只含随机项的既有形态：1.5777·δ_R，不额外并入偏置。
  const float std_floor = RadarEquations::ComputeRangeErrorStdDev(-20.0f, 1e6f);
  const float range_resolution =
      0.5f * static_cast<float>(oneq::common::numerics::kLightSpeed) / 1.0e6f;
  EXPECT_NEAR(std_floor, 1.5777f * range_resolution, 1.0e-3f);
}

TEST(CommonRadarEquationsTest, AngleStdDevIsPureRandomTerm) {
  // SNR=20dB：σ = 0.317·θ_bw/√100；θ_bw/30 偏置已拆出，不再并入 std。
  const float beamwidth_rad = 0.05f;
  const float std_dev = RadarEquations::ComputeAngleErrorStdDev(20.0f, beamwidth_rad);
  EXPECT_NEAR(std_dev, 0.317f * beamwidth_rad / 10.0f, 1.0e-6f);
}

TEST(CommonRadarEquationsTest, AngleStdDevLowSnrFloorReturnsBeamwidth) {
  // snr<-10dB 下限分支保持只含随机项的既有形态：返回全波束宽度，不额外并入偏置。
  EXPECT_FLOAT_EQ(RadarEquations::ComputeAngleErrorStdDev(-20.0f, 0.05f), 0.05f);
}

TEST(CommonRadarEquationsTest, MeasurementBiasExportsMatchLegacyTerms) {
  // 拆出的偏置量与旧实现并入 std 的口径一致：距离 20 m 常数、角度波束宽度/30。
  EXPECT_FLOAT_EQ(kRangeMeasurementBiasM, 20.0f);
  EXPECT_FLOAT_EQ(ComputeAngleMeasurementBiasRad(0.03f), 0.03f / 30.0f);
  EXPECT_FLOAT_EQ(ComputeAngleMeasurementBiasRad(0.09f), 0.09f / 30.0f);
}

}  // namespace
}  // namespace radar
}  // namespace common
}  // namespace oneq
