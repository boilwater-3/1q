// Copyright 2026. All Rights Reserved.
//
// @file rir_measurement_error_test.cpp
// @brief 验证 RIR 测量误差模型（副本改写自 ar_signal_detection_test.cpp 的
//        MeasurementErrorModel 段；阶段 2-M M6）。

#include <gtest/gtest.h>

#include <cmath>

#include "remote_identification_radar/dwell/RirMeasurementErrorModel.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using dwell::RirEffectiveBeamwidthDeg;
using dwell::RirMeasurementErrorModel;

/// @brief 高 SNR 下距离误差接近偏置项、角度误差小。
TEST(RirMeasurementErrorModelTest, HighSnrSmallErrors) {
  const auto state = RirMeasurementErrorModel::Compute(20.0f, RirEffectiveBeamwidthDeg{3.0f, 3.0f},
                                                       1.0e6f);
  EXPECT_NEAR(state.range_error_std_m, 27.5f, 0.5f);  // 0.5·150/10 + 20
  EXPECT_LT(state.angle_error_std_rad, 0.01f);
}

/// @brief 俯仰波束宽度加宽 → 等效角度标准差增大。
TEST(RirMeasurementErrorModelTest, ElevationBeamwidthAffectsEquivalentAngleStdDev) {
  const auto narrow =
      RirMeasurementErrorModel::Compute(13.0f, RirEffectiveBeamwidthDeg{3.0f, 3.0f}, 1.0e6f);
  const auto wide =
      RirMeasurementErrorModel::Compute(13.0f, RirEffectiveBeamwidthDeg{3.0f, 6.0f}, 1.0e6f);
  EXPECT_GT(wide.angle_error_std_rad, narrow.angle_error_std_rad);
  // 波束内距离误差不受角度维影响。
  EXPECT_FLOAT_EQ(wide.range_error_std_m, narrow.range_error_std_m);
}

/// @brief SNR 降低 → 距离与角度误差均增大。
TEST(RirMeasurementErrorModelTest, LowerSnrInflatesErrors) {
  const auto strong = RirMeasurementErrorModel::Compute(20.0f, RirEffectiveBeamwidthDeg{3.0f, 3.0f},
                                                        1.0e6f);
  const auto weak = RirMeasurementErrorModel::Compute(0.0f, RirEffectiveBeamwidthDeg{3.0f, 3.0f},
                                                     1.0e6f);
  EXPECT_GT(weak.range_error_std_m, strong.range_error_std_m);
  EXPECT_GT(weak.angle_error_std_rad, strong.angle_error_std_rad);
}

/// @brief 等效角度标准差 = az/el 两轴的 RMS 合成（各向同性时等于单轴）。
TEST(RirMeasurementErrorModelTest, EquivalentAngleIsRmsOfAxes) {
  const auto iso =
      RirMeasurementErrorModel::Compute(10.0f, RirEffectiveBeamwidthDeg{3.0f, 3.0f}, 1.0e6f);
  const auto az_only =
      RirMeasurementErrorModel::Compute(10.0f, RirEffectiveBeamwidthDeg{3.0f, 0.001f}, 1.0e6f);
  // el 波束极窄 → el 误差近似仅偏置项，等效值低于各向同性。
  EXPECT_LT(az_only.angle_error_std_rad, iso.angle_error_std_rad);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
