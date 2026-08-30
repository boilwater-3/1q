// Copyright 2026. All Rights Reserved.
//
// @file rir_surface_clutter_model_test.cpp
// @brief 逐目标主瓣地杂波最小物理模型特征化测试。
//
// 锁定口径：σ₀ 档位序、擦地角几何响应（主瓣离地归零）、脉冲限制区距离
// 衰减、雷达方程 λ² 频率响应、退化输入免疫与确定性。数值不追手工恒等式，
// 以序关系与量级带锁定物理行为。

#include <gtest/gtest.h>

#include <cmath>

#include "remote_identification_radar/internal/RirSurfaceClutterModel.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using config::RirVegetationCoverProfile;
using internal::RirSurfaceClutterInput;
using internal::RirSurfaceClutterModel;

constexpr float kThermalNoiseW = 4.5e-14f;  ///< kTBF 量级参考（NF≈4 dB @ 4.5 MHz）。

RirSurfaceClutterInput MakeInput(RirVegetationCoverProfile profile, float range_m,
                                 float look_el_deg) {
  RirSurfaceClutterInput input;
  input.vegetation_cover_profile = profile;
  input.transmitter.frequency_hz = 3.0e9f;
  input.transmitter.bandwidth_hz = 4.5e6f;
  input.transmitter.peak_power_w = 1.0e6f;
  input.transmitter.transmit_loss_db = 3.5f;
  input.antenna.main_beam_gain_db = 35.0f;
  input.antenna.nominal_az_beamwidth_deg = 4.0f;
  input.antenna.nominal_el_beamwidth_deg = 4.0f;
  input.propagation_loss_db = 6.5f;
  input.range_m = range_m;
  input.look_el_deg = look_el_deg;
  input.thermal_noise_w = kThermalNoiseW;
  return input;
}

float CnrDb(const RirSurfaceClutterModel& model, const RirSurfaceClutterInput& input) {
  const float clutter_w = model.EvaluateClutterNoiseW(input);
  return 10.0f * std::log10(clutter_w / input.thermal_noise_w);
}

/// @brief 植被关闭恒零（行为不变式：默认环境零杂波）。
TEST(RirSurfaceClutterModelTest, DisabledVegetationIsZero) {
  const RirSurfaceClutterModel model;
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(
                      MakeInput(RirVegetationCoverProfile::kDisabled, 5000.0f, 0.0f)),
                  0.0f);
}

/// @brief 主瓣离地归零：目标仰角达到半俯仰波束宽（4°/2 = 2°）后无主瓣地杂波。
TEST(RirSurfaceClutterModelTest, BeamFullyAboveHorizonIsZero) {
  const RirSurfaceClutterModel model;
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(
                      MakeInput(RirVegetationCoverProfile::kDeciduousForest, 5000.0f, 2.0f)),
                  0.0f);
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(
                      MakeInput(RirVegetationCoverProfile::kDeciduousForest, 5000.0f, 8.0f)),
                  0.0f);
}

/// @brief 地平线附近杂波存在且随俯角压低单调增强（擦地角增大 → σ₀ 因子增大）。
TEST(RirSurfaceClutterModelTest, NearHorizonClutterGrowsWithDepression) {
  const RirSurfaceClutterModel model;
  const float at_horizon =
      model.EvaluateClutterNoiseW(MakeInput(RirVegetationCoverProfile::kOpenGrassland,
                                            5000.0f, 0.0f));
  const float depressed =
      model.EvaluateClutterNoiseW(MakeInput(RirVegetationCoverProfile::kOpenGrassland,
                                            5000.0f, -5.0f));
  EXPECT_GT(at_horizon, 0.0f);
  EXPECT_GT(depressed, at_horizon);
}

/// @brief 量级带：参考几何（草地 5 km 地平线）CNR 落在主瓣地杂波物理量级内。
TEST(RirSurfaceClutterModelTest, ReferenceGeometryCnrInPhysicalBand) {
  const RirSurfaceClutterModel model;
  const float cnr_db = CnrDb(model, MakeInput(RirVegetationCoverProfile::kOpenGrassland,
                                              5000.0f, 0.0f));
  EXPECT_GT(cnr_db, 20.0f);
  EXPECT_LT(cnr_db, 100.0f);
}

/// @brief 档位序：σ₀ 表序 → CNR 严格同序（修复旧模型档位差 <0.1 dB 的失效）。
TEST(RirSurfaceClutterModelTest, ProfileOrderingFollowsSigma0Table) {
  const RirSurfaceClutterModel model;
  const RirVegetationCoverProfile profiles[] = {
      RirVegetationCoverProfile::kOpenGrassland, RirVegetationCoverProfile::kSparseWoodland,
      RirVegetationCoverProfile::kDeciduousForest,
      RirVegetationCoverProfile::kConiferousForest, RirVegetationCoverProfile::kTropicalDense};
  float previous_cnr_db = -1.0e9f;
  for (auto profile : profiles) {
    const float cnr_db =
        CnrDb(model, MakeInput(profile, 5000.0f, -10.0f));
    EXPECT_GT(cnr_db, previous_cnr_db) << "profile=" << static_cast<int>(profile);
    previous_cnr_db = cnr_db;
  }
}

/// @brief 距离响应：脉冲限制区杂波功率随斜距三次方衰减（σ_c∝R，雷达方程 R⁻⁴）。
TEST(RirSurfaceClutterModelTest, PulseLimitedClutterDecaysWithRange) {
  const RirSurfaceClutterModel model;
  const float near = model.EvaluateClutterNoiseW(
      MakeInput(RirVegetationCoverProfile::kSparseWoodland, 2000.0f, -5.0f));
  const float mid = model.EvaluateClutterNoiseW(
      MakeInput(RirVegetationCoverProfile::kSparseWoodland, 5000.0f, -5.0f));
  const float far = model.EvaluateClutterNoiseW(
      MakeInput(RirVegetationCoverProfile::kSparseWoodland, 10000.0f, -5.0f));
  EXPECT_GT(near, mid);
  EXPECT_GT(mid, far);
}

/// @brief 频率响应：同几何下载频升高 → λ² 项使杂波下降（σ₀ 频段常数是声明简化）。
TEST(RirSurfaceClutterModelTest, HigherCarrierLowersClutter) {
  const RirSurfaceClutterModel model;
  RirSurfaceClutterInput s_band = MakeInput(RirVegetationCoverProfile::kDeciduousForest,
                                            5000.0f, 0.0f);
  RirSurfaceClutterInput x_band = s_band;
  x_band.transmitter.frequency_hz = 9.0e9f;
  EXPECT_LT(model.EvaluateClutterNoiseW(x_band), model.EvaluateClutterNoiseW(s_band));
}

/// @brief 退化输入免疫：斜距/热噪/载频/带宽非正一律返回 0。
TEST(RirSurfaceClutterModelTest, DegenerateInputsAreZero) {
  const RirSurfaceClutterModel model;
  RirSurfaceClutterInput base = MakeInput(RirVegetationCoverProfile::kTropicalDense,
                                          5000.0f, 0.0f);
  RirSurfaceClutterInput zero_range = base;
  zero_range.range_m = 0.0f;
  RirSurfaceClutterInput zero_thermal = base;
  zero_thermal.thermal_noise_w = 0.0f;
  RirSurfaceClutterInput zero_carrier = base;
  zero_carrier.transmitter.frequency_hz = 0.0f;
  RirSurfaceClutterInput zero_bandwidth = base;
  zero_bandwidth.transmitter.bandwidth_hz = 0.0f;
  RirSurfaceClutterInput zero_beamwidth = base;
  zero_beamwidth.antenna.nominal_az_beamwidth_deg = 0.0f;
  zero_beamwidth.antenna.nominal_el_beamwidth_deg = 0.0f;
  zero_beamwidth.antenna.antenna_length_m = 0.0f;
  zero_beamwidth.antenna.antenna_width_m = 0.0f;
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(zero_range), 0.0f);
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(zero_thermal), 0.0f);
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(zero_carrier), 0.0f);
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(zero_bandwidth), 0.0f);
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(zero_beamwidth), 0.0f);
}

/// @brief 同输入确定性输出。
TEST(RirSurfaceClutterModelTest, DeterministicAcrossCalls) {
  const RirSurfaceClutterModel model;
  const auto input = MakeInput(RirVegetationCoverProfile::kTropicalDense, 5000.0f, 0.0f);
  EXPECT_FLOAT_EQ(model.EvaluateClutterNoiseW(input), model.EvaluateClutterNoiseW(input));
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
