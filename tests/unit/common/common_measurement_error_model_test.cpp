/**
 * @file common_measurement_error_model_test.cpp
 * @brief 验证 common 测量误差模型的高 SNR、低 SNR、波束宽度与 RMS 合成语义。
 */

#include <gtest/gtest.h>

#include <cmath>

#include "common/radar/MeasurementErrorModel.h"

namespace oneq {
namespace common {
namespace radar {
namespace {

TEST(CommonMeasurementErrorModelTest, HighSnrProducesSmallErrors) {
  const MeasurementErrorState state = ComputeMeasurementError(20.0f, 1.0e6f, 0.05f, 0.05f);
  EXPECT_NEAR(state.range_error_std_m, 27.5f, 0.5f);
  EXPECT_GT(state.angle_error_std_rad, 0.0f);
  EXPECT_LT(state.angle_error_std_rad, 0.01f);
}

TEST(CommonMeasurementErrorModelTest, LowerSnrInflatesErrors) {
  const MeasurementErrorState strong = ComputeMeasurementError(20.0f, 1.0e6f, 0.05f, 0.05f);
  const MeasurementErrorState weak = ComputeMeasurementError(0.0f, 1.0e6f, 0.05f, 0.05f);
  EXPECT_GT(weak.range_error_std_m, strong.range_error_std_m);
  EXPECT_GT(weak.angle_error_std_rad, strong.angle_error_std_rad);
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
