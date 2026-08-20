/**
 * @file eos_spatial_spectrum_unit_test.cpp
 * @brief 验证 EOS 空间频率谱可分辨性评估行为。
 */

#include <gtest/gtest.h>

#include "electro_optical_sensor/foundation/EosSpatialSpectrum.h"

namespace electro_optical_sensor {
namespace foundation {
namespace spatial_spectrum {
namespace {

TEST(EosSpatialSpectrumTest, ResolvabilityImprovesForLargerTargetAtSameGsd) {
  SpatialSpectrumInputs small_target_inputs;
  small_target_inputs.target_characteristic_size_m = 0.6f;
  small_target_inputs.ground_sample_distance_m = 0.25f;
  small_target_inputs.optical_mtf_reference = 0.7f;
  small_target_inputs.sampling_efficiency = 0.9f;
  small_target_inputs.scene_contrast_ratio = 0.4f;

  SpatialSpectrumInputs large_target_inputs = small_target_inputs;
  large_target_inputs.target_characteristic_size_m = 2.0f;

  const SpatialSpectrumResult small_result =
      EvaluateSpatialResolvability(small_target_inputs);
  const SpatialSpectrumResult large_result =
      EvaluateSpatialResolvability(large_target_inputs);

  EXPECT_GT(large_result.resolvability_score, small_result.resolvability_score);
  EXPECT_GT(large_result.spectrum_quality_gain, small_result.spectrum_quality_gain);
}

TEST(EosSpatialSpectrumTest, ResolvabilityDegradesWhenGsdGetsCoarser) {
  SpatialSpectrumInputs fine_inputs;
  fine_inputs.target_characteristic_size_m = 1.5f;
  fine_inputs.ground_sample_distance_m = 0.15f;
  fine_inputs.optical_mtf_reference = 0.7f;
  fine_inputs.sampling_efficiency = 0.9f;
  fine_inputs.scene_contrast_ratio = 0.35f;

  SpatialSpectrumInputs coarse_inputs = fine_inputs;
  coarse_inputs.ground_sample_distance_m = 0.6f;

  const SpatialSpectrumResult fine_result = EvaluateSpatialResolvability(fine_inputs);
  const SpatialSpectrumResult coarse_result = EvaluateSpatialResolvability(coarse_inputs);

  EXPECT_GT(fine_result.nyquist_frequency_cpm, coarse_result.nyquist_frequency_cpm);
  EXPECT_GT(fine_result.resolvability_score, coarse_result.resolvability_score);
}

}  // namespace
}  // namespace spatial_spectrum
}  // namespace foundation
}  // namespace electro_optical_sensor
