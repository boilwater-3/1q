/**
 * @file eos_noise_model_unit_test.cpp
 * @brief 验证 EOS 背景噪声统计模型数值行为。
 */

#include <gtest/gtest.h>

#include "electro_optical_sensor/foundation/EosNoiseModel.h"

namespace electro_optical_sensor {
namespace foundation {
namespace noise {
namespace {

TEST(EosNoiseModelTest, EquivalentNoiseIncreasesWithBackgroundFlux) {
  BackgroundNoiseModelInputs low_flux_inputs;
  low_flux_inputs.background_flux_w = 1.0e-11f;
  low_flux_inputs.electrical_bandwidth_hz = 1500.0f;
  low_flux_inputs.integration_time_sec = 0.02f;
  low_flux_inputs.cloud_coverage_ratio = 0.2f;

  BackgroundNoiseModelInputs high_flux_inputs = low_flux_inputs;
  high_flux_inputs.background_flux_w = 5.0e-11f;

  const BackgroundNoiseStatistics low_flux_stats =
      ComputeBackgroundNoiseStatistics(low_flux_inputs);
  const BackgroundNoiseStatistics high_flux_stats =
      ComputeBackgroundNoiseStatistics(high_flux_inputs);

  EXPECT_GT(high_flux_stats.equivalent_noise_power_w,
            low_flux_stats.equivalent_noise_power_w);
}

TEST(EosNoiseModelTest, SuppressionWeightIsClampedToConfiguredRange) {
  BackgroundNoiseModelInputs inputs;
  inputs.background_flux_w = 2.0e-9f;
  inputs.electrical_bandwidth_hz = 2.0e4f;
  inputs.integration_time_sec = 0.005f;
  inputs.cloud_coverage_ratio = 1.0f;
  inputs.scene_complexity_factor = 3.0f;
  inputs.photon_noise_enhancement_factor = 2.0f;

  const BackgroundNoiseStatistics stats = ComputeBackgroundNoiseStatistics(inputs);
  EXPECT_GE(stats.suppression_weight, 0.05f);
  EXPECT_LE(stats.suppression_weight, 0.60f);
}

TEST(EosNoiseModelTest, EffectiveSignalPowerRespectsBackgroundSuppression) {
  BackgroundNoiseStatistics stats;
  stats.suppression_weight = 0.30f;

  const float effective_signal_w = ComputeEffectiveSignalPowerW(2.0e-10f, 3.0e-10f, stats);
  const float zero_floor_signal_w = ComputeEffectiveSignalPowerW(1.0e-11f, 5.0e-10f, stats);

  EXPECT_FLOAT_EQ(effective_signal_w, 1.1e-10f);
  EXPECT_GT(zero_floor_signal_w, 0.0f);
  EXPECT_LT(zero_floor_signal_w, 1.0e-11f);
}

}  // namespace
}  // namespace noise
}  // namespace foundation
}  // namespace electro_optical_sensor
