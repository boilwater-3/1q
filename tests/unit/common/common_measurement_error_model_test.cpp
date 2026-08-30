/**
 * @file common_measurement_error_model_test.cpp
 * @brief 验证 common 测量误差模型的高 SNR、低 SNR、波束宽度、RMS 合成与 bias/std 拆分语义。
 */

#include <gtest/gtest.h>

#include <cmath>

#include "common/numerics/Constants.h"
#include "common/radar/MeasurementErrorModel.h"

namespace oneq {
namespace common {
namespace radar {
namespace {

TEST(CommonMeasurementErrorModelTest, HighSnrProducesSmallErrors) {
  const MeasurementErrorState state = ComputeMeasurementError(20.0f, 1.0e6f, 0.05f, 0.05f);
  // 纯随机项（2026-08-30 拆分后）：δ_R = c/(2B) ≈ 149.896 m，
  // σ = 0.5·δ_R/√100 = 7.495 m；固定 20 m 偏置已拆出，不再加进 std。
  EXPECT_NEAR(state.range_error_std_m, 7.4948f, 1.0e-3f);
  EXPECT_GT(state.angle_error_std_rad, 0.0f);
  EXPECT_LT(state.angle_error_std_rad, 0.01f);
}

TEST(CommonMeasurementErrorModelTest, LowerSnrInflatesErrors) {
  const MeasurementErrorState strong = ComputeMeasurementError(20.0f, 1.0e6f, 0.05f, 0.05f);
  const MeasurementErrorState weak = ComputeMeasurementError(0.0f, 1.0e6f, 0.05f, 0.05f);
  EXPECT_GT(weak.range_error_std_m, strong.range_error_std_m);
  EXPECT_GT(weak.angle_error_std_rad, strong.angle_error_std_rad);
}

TEST(CommonMeasurementErrorModelTest, BiasFieldsSplitFromRandomStd) {
  const MeasurementErrorState state = ComputeMeasurementError(20.0f, 1.0e6f, 0.05f, 0.10f);
  // 距离偏置为固定常数（未标定残差），不随 SNR/带宽/波束宽度变化。
  EXPECT_FLOAT_EQ(state.range_bias_m, kRangeMeasurementBiasM);
  EXPECT_FLOAT_EQ(state.range_bias_m, 20.0f);
  // 角度偏置按两轴 bw/30 的 RMS 合成，与 angle_error_std_rad 的两轴合成口径一致。
  const float expected_angle_bias =
      std::sqrt(0.5f * ((0.05f / 30.0f) * (0.05f / 30.0f) + (0.10f / 30.0f) * (0.10f / 30.0f)));
  EXPECT_FLOAT_EQ(state.angle_bias_rad, expected_angle_bias);
  // bias 是传感器属性而非噪声属性：SNR 变化不影响 bias 字段。
  const MeasurementErrorState weak = ComputeMeasurementError(0.0f, 1.0e6f, 0.05f, 0.10f);
  EXPECT_FLOAT_EQ(weak.range_bias_m, state.range_bias_m);
  EXPECT_FLOAT_EQ(weak.angle_bias_rad, state.angle_bias_rad);
}

TEST(CommonMeasurementErrorModelTest, LowSnrStdSaturatedRandomOnlyBiasStillReported) {
  // snr<-10dB 下限分支：std 只含随机项的既有饱和形态，不额外并入偏置；
  // bias 字段仍按固定口径填充。
  const MeasurementErrorState state = ComputeMeasurementError(-15.0f, 1.0e6f, 0.05f, 0.10f);
  const float range_resolution =
      0.5f * static_cast<float>(oneq::common::numerics::kLightSpeed) / 1.0e6f;
  EXPECT_NEAR(state.range_error_std_m, 1.5777f * range_resolution, 1.0e-3f);
  EXPECT_FLOAT_EQ(state.angle_error_std_rad, std::sqrt(0.5f * (0.05f * 0.05f + 0.10f * 0.10f)));
  EXPECT_FLOAT_EQ(state.range_bias_m, 20.0f);
}

TEST(CommonMeasurementErrorModelTest, ElevationBeamwidthAffectsEquivalentAngleStdDev) {
  const MeasurementErrorState narrow = ComputeMeasurementError(13.0f, 1.0e6f, 0.05f, 0.05f);
  const MeasurementErrorState wide = ComputeMeasurementError(13.0f, 1.0e6f, 0.05f, 0.10f);
  EXPECT_GT(wide.angle_error_std_rad, narrow.angle_error_std_rad);
}

TEST(CommonMeasurementErrorModelTest, EquivalentAngleIsRmsOfAxes) {
  const float az_std = RadarEquations::ComputeAngleErrorStdDev(10.0f, 0.05f);
  const float el_std = RadarEquations::ComputeAngleErrorStdDev(10.0f, 1.0e-5f);
  const float expected = std::sqrt(0.5f * (az_std * az_std + el_std * el_std));
  const float actual = ComputeEquivalentAngleErrorStdDev(10.0f, 0.05f, 1.0e-5f);
  EXPECT_FLOAT_EQ(actual, expected);
}

}  // namespace
}  // namespace radar
}  // namespace common
}  // namespace oneq
