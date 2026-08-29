// Copyright 2026. All Rights Reserved.
//
// @file rir_antenna_pattern_test.cpp
// @brief 验证 RIR 方向图评估与波束控制子集（副本改写自 ar_signal_detection_test.cpp
//        的 BeamControlResolver/方向图段；阶段 2-M M2）。

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "common/radar/AntennaPatternRuntime.h"
#include "support/oneq_test_temp_dir.h"
#include "remote_identification_radar/dwell/RirAntennaPatternRuntime.h"
#include "remote_identification_radar/dwell/RirBeamControl.h"
#include "remote_identification_radar/runtime/RirAcceptanceRecords.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using config::hardware::RirAntennaConfig;
using config::hardware::RirAntennaPatternModelType;
using dwell::RirAntennaLookOffsetDeg;
using dwell::RirAntennaPatternBeamwidthDeg;
using dwell::RirEvaluateAntennaPattern;
using dwell::RirResolveBeamStateForPointing;
using dwell::RirResolveEffectiveBeamwidth;

constexpr float kDeg2Rad = 3.14159265358979f / 180.0f;

RirAntennaConfig MakeAntenna(RirAntennaPatternModelType model) {
  RirAntennaConfig ant;
  ant.main_beam_gain_db = 35.0f;
  ant.nominal_az_beamwidth_deg = 3.0f;
  ant.nominal_el_beamwidth_deg = 3.0f;
  ant.pattern.model_type = model;
  ant.pattern.max_sidelobe_level_db = -20.0f;
  ant.pattern.backlobe_level_db = -35.0f;
  return ant;
}

// ===========================================================================
// 方向图评估（4 模型 × 边界区域）
// ===========================================================================

/// @brief 轴向目标（离轴 0）增益 = 峰值 - 扫描损失（所有模型）。
TEST(RirAntennaPatternTest, OnAxis_GainEqualsPeakMinusScanLoss) {
  const RirAntennaPatternModelType models[] = {
      RirAntennaPatternModelType::kGaussianMainLobe,
      RirAntennaPatternModelType::kParabolicMainLobe, RirAntennaPatternModelType::kCosinePower,
      RirAntennaPatternModelType::kSincPattern};
  for (auto model : models) {
    const RirAntennaConfig ant = MakeAntenna(model);
    const auto sample = RirEvaluateAntennaPattern(
        ant.main_beam_gain_db, ant.pattern, RirAntennaPatternBeamwidthDeg{3.0f, 3.0f},
        RirAntennaLookOffsetDeg{0.0f, 0.0f}, config::RirAzimuthElevationDeg{0.0f, 0.0f});
    EXPECT_NEAR(sample.gain_dbi, ant.main_beam_gain_db, 1e-3f)
        << "model=" << static_cast<int>(model);
    EXPECT_TRUE(sample.inside_main_lobe);
    EXPECT_FALSE(sample.inside_back_lobe);
  }
}

/// @brief 半功率波束边缘（离轴 = 半宽）衰减约 3 dB（高斯/抛物线模型）。
TEST(RirAntennaPatternTest, HalfBeamwidthEdge_AtThreeDb) {
  const RirAntennaPatternModelType quadratic[] = {
      RirAntennaPatternModelType::kGaussianMainLobe,
      RirAntennaPatternModelType::kParabolicMainLobe};
  for (auto model : quadratic) {
    const RirAntennaConfig ant = MakeAntenna(model);
    const auto sample = RirEvaluateAntennaPattern(
        ant.main_beam_gain_db, ant.pattern, RirAntennaPatternBeamwidthDeg{3.0f, 3.0f},
        RirAntennaLookOffsetDeg{1.5f, 0.0f}, config::RirAzimuthElevationDeg{0.0f, 0.0f});
    EXPECT_NEAR(sample.main_lobe_attenuation_db, 3.0f, 1e-3f)
        << "model=" << static_cast<int>(model);
    EXPECT_NEAR(sample.gain_dbi, ant.main_beam_gain_db - 3.0f, 1e-3f);
  }
}

/// @brief 主瓣外副瓣区（评审 2026-08-26 条14）：sinc² 包络连续延拓并以最大旁瓣
/// 电平钳制——近副瓣区（包络高于电平）增益 = 峰值 + 最大旁瓣电平；零陷区增益
/// 严格低于该电平（主副瓣结构可辨，不再恒定平台）。
TEST(RirAntennaPatternTest, SideLobe_UsesMaxSidelobeLevel) {
  const RirAntennaConfig ant = MakeAntenna(RirAntennaPatternModelType::kGaussianMainLobe);
  // 近副瓣区：u_az = 2.4/1.5 = 1.6，sinc² 衰减 ≈8.9dB < 20dB → 钳制在电平上。
  const auto shelf = RirEvaluateAntennaPattern(
      ant.main_beam_gain_db, ant.pattern, RirAntennaPatternBeamwidthDeg{3.0f, 3.0f},
      RirAntennaLookOffsetDeg{2.4f, 0.0f}, config::RirAzimuthElevationDeg{0.0f, 0.0f});
  EXPECT_FALSE(shelf.inside_main_lobe);
  EXPECT_NEAR(shelf.gain_dbi, ant.main_beam_gain_db + ant.pattern.max_sidelobe_level_db, 1e-3f);
  // 零陷区：u_az = 10/1.5 ≈ 6.67，sinc² 衰减 >20dB → 低于钳制电平。
  const auto null_region = RirEvaluateAntennaPattern(
      ant.main_beam_gain_db, ant.pattern, RirAntennaPatternBeamwidthDeg{3.0f, 3.0f},
      RirAntennaLookOffsetDeg{10.0f, 0.0f}, config::RirAzimuthElevationDeg{0.0f, 0.0f});
  EXPECT_FALSE(null_region.inside_main_lobe);
  EXPECT_LT(null_region.gain_dbi,
            ant.main_beam_gain_db + ant.pattern.max_sidelobe_level_db - 1.0f);
}

/// @brief 后瓣（离轴 > 90°）增益 = 峰值 + 后瓣电平 - 扫描损失。
TEST(RirAntennaPatternTest, BackLobe_UsesBacklobeLevel) {
  const RirAntennaConfig ant = MakeAntenna(RirAntennaPatternModelType::kGaussianMainLobe);
  const auto sample = RirEvaluateAntennaPattern(
      ant.main_beam_gain_db, ant.pattern, RirAntennaPatternBeamwidthDeg{3.0f, 3.0f},
      RirAntennaLookOffsetDeg{120.0f, 0.0f}, config::RirAzimuthElevationDeg{0.0f, 0.0f});
  EXPECT_TRUE(sample.inside_back_lobe);
  EXPECT_NEAR(sample.gain_dbi, ant.main_beam_gain_db + ant.pattern.backlobe_level_db, 1e-3f);
}

/// @brief 扫描损失随偏置平方增长并钳位到 max_scan_loss_db。
TEST(RirAntennaPatternTest, ScanLoss_ClampedToMax) {
  RirAntennaConfig ant = MakeAntenna(RirAntennaPatternModelType::kGaussianMainLobe);
  ant.pattern.scan_loss_coeff_db_per_deg2 = 0.01f;
  ant.pattern.max_scan_loss_db = 6.0f;
  const auto small = RirEvaluateAntennaPattern(
      ant.main_beam_gain_db, ant.pattern, RirAntennaPatternBeamwidthDeg{3.0f, 3.0f},
      RirAntennaLookOffsetDeg{0.0f, 0.0f}, config::RirAzimuthElevationDeg{10.0f, 0.0f});
  EXPECT_NEAR(small.scan_loss_db, 0.01f * 100.0f, 1e-3f);
  const auto large = RirEvaluateAntennaPattern(
      ant.main_beam_gain_db, ant.pattern, RirAntennaPatternBeamwidthDeg{3.0f, 3.0f},
      RirAntennaLookOffsetDeg{0.0f, 0.0f}, config::RirAzimuthElevationDeg{60.0f, 0.0f});
  EXPECT_NEAR(large.scan_loss_db, 6.0f, 1e-3f);
}

/// @brief sinc² 模式：孔径与波长给定，第一零点附近增益显著低于轴向。
TEST(RirAntennaPatternTest, SincPattern_ApertureDrivenAttenuation) {
  RirAntennaConfig ant = MakeAntenna(RirAntennaPatternModelType::kSincPattern);
  ant.antenna_length_m = 4.0f;
  const float wavelength_m = 0.1f;
  // sinc 第一零点：sin(θ) = λ/L → θ ≈ asin(0.025) rad ≈ 1.432°
  const float first_null_deg = std::asin(wavelength_m / ant.antenna_length_m) / kDeg2Rad;
  const auto axial = RirEvaluateAntennaPattern(
      ant.main_beam_gain_db, ant.pattern, RirAntennaPatternBeamwidthDeg{3.0f, 3.0f},
      RirAntennaLookOffsetDeg{0.0f, 0.0f}, config::RirAzimuthElevationDeg{0.0f, 0.0f},
      ant.antenna_length_m, ant.antenna_width_m, wavelength_m);
  const auto off_zero = RirEvaluateAntennaPattern(
      ant.main_beam_gain_db, ant.pattern, RirAntennaPatternBeamwidthDeg{3.0f, 3.0f},
      RirAntennaLookOffsetDeg{first_null_deg * 0.95f, 0.0f},
      config::RirAzimuthElevationDeg{0.0f, 0.0f}, ant.antenna_length_m, ant.antenna_width_m,
      wavelength_m);
  EXPECT_GT(axial.gain_dbi, off_zero.gain_dbi + 10.0f);
}

// ===========================================================================
// 波束控制子集（有效波束宽度 + 给定指向增益）
// ===========================================================================

/// @brief 有效波束宽度：nominal 为正直接生效；为 0 时从 λ/L 物理推导。
TEST(RirBeamControlTest, EffectiveBeamwidth_NominalThenAperture) {
  RirAntennaConfig ant = MakeAntenna(RirAntennaPatternModelType::kGaussianMainLobe);
  auto bw = RirResolveEffectiveBeamwidth(ant, 0.1f);
  EXPECT_NEAR(bw.az_beamwidth_deg, 3.0f, 1e-4f);
  EXPECT_NEAR(bw.el_beamwidth_deg, 3.0f, 1e-4f);

  ant.nominal_az_beamwidth_deg = 0.0f;
  ant.antenna_length_m = 4.0f;
  bw = RirResolveEffectiveBeamwidth(ant, 0.1f);
  // θ_bw = λ/L = 0.025 rad ≈ 1.432°
  EXPECT_NEAR(bw.az_beamwidth_deg, 0.025f / kDeg2Rad, 1e-3f);
  EXPECT_NEAR(bw.el_beamwidth_deg, 3.0f, 1e-4f);
}

/// @brief 无有效视线角时同样回退主瓣峰值。
TEST(RirBeamControlTest, NoLookAngles_FallsBackToPeakGain) {
  RirAntennaConfig ant = MakeAntenna(RirAntennaPatternModelType::kGaussianMainLobe);
  const auto state =
      RirResolveBeamStateForPointing(ant, config::RirAzimuthElevationDeg{0.0f, 0.0f}, 45.0f, 0.0f,
                                     false, 0.1f);
  EXPECT_FLOAT_EQ(state.one_way_antenna_gain_db, ant.main_beam_gain_db);
}

/// @brief 波束指向目标 → 轴向增益；指向偏离 → 增益按方向图下降。
TEST(RirBeamControlTest, PointingOnTarget_PeakGainElseAttenuated) {
  RirAntennaConfig ant = MakeAntenna(RirAntennaPatternModelType::kGaussianMainLobe);
  const auto on_target =
      RirResolveBeamStateForPointing(ant, config::RirAzimuthElevationDeg{30.0f, 5.0f}, 30.0f, 5.0f,
                                     true, 0.1f);
  EXPECT_NEAR(on_target.one_way_antenna_gain_db, ant.main_beam_gain_db, 1e-3f);

  const auto off_target =
      RirResolveBeamStateForPointing(ant, config::RirAzimuthElevationDeg{30.0f, 5.0f}, 31.5f, 5.0f,
                                     true, 0.1f);
  // 离轴 1.5° = 半功率边缘 → 约 -3 dB
  EXPECT_NEAR(off_target.one_way_antenna_gain_db, ant.main_beam_gain_db - 3.0f, 0.05f);
}

TEST(RirAcceptanceScanPatternTest, WritesIndexAzElCsv) {
  std::vector<oneq::common::radar::AzimuthElevationDeg> pattern;
  pattern.push_back(oneq::common::radar::AzimuthElevationDeg(-10.0f, 5.0f));
  pattern.push_back(oneq::common::radar::AzimuthElevationDeg(20.0f, 15.0f));
  const std::string path = oneq_test::TempDir() + "rir_scan_pattern_test.csv";
  ASSERT_TRUE(runtime::TryExportRirScanPatternCsv(pattern, path.c_str()));
  std::ifstream in(path.c_str());
  ASSERT_TRUE(in.is_open());
  std::string header;
  std::getline(in, header);
  EXPECT_EQ(header, "index,az_deg,el_deg");
  std::string row0;
  std::getline(in, row0);
  EXPECT_NE(row0.find("0,"), std::string::npos);
  EXPECT_NE(row0.find("-10.000"), std::string::npos);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
