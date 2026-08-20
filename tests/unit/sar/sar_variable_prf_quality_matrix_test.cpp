#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

#include "sar/imaging/SarImageQuality.h"
#include "sar/imaging/SarRda.h"
#include "sar/imaging/SarSlowTimeResampling.h"
#include "support/sar_reference_scene.h"

namespace sar {
namespace imaging {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

struct QualityCase {
  double jitter_ratio{0.0};
  double raw_nrms{0.0};
  ImageComparisonMetrics image_comparison{};
  SlowTimeResamplingDiagnostics timing{};
};

double UnitEnergyNrms(const signal::ComplexMatrix& reference,
                      const signal::ComplexMatrix& candidate) {
  double reference_energy = 0.0;
  double candidate_energy = 0.0;
  for (std::size_t index = 0U; index < reference.values.size(); ++index) {
    reference_energy += std::norm(reference.values[index]);
    candidate_energy += std::norm(candidate.values[index]);
  }
  const double reference_scale = 1.0 / std::sqrt(reference_energy);
  const double candidate_scale = 1.0 / std::sqrt(candidate_energy);
  double error_energy = 0.0;
  for (std::size_t index = 0U; index < reference.values.size(); ++index) {
    error_energy +=
        std::norm(reference.values[index] * reference_scale -
                  candidate.values[index] * candidate_scale);
  }
  return std::sqrt(error_energy);
}

bool EvaluateCase(double jitter_ratio, QualityCase* result) {
  test_support::ReferencePointScene uniform_scene;
  if (result == nullptr || !test_support::BuildReferencePointScene(&uniform_scene)) {
    return false;
  }
  const std::size_t target_delay = 28U;
  const std::vector<echo::PointTarget> targets{
      test_support::MakeReferenceTargetAtDelay(target_delay, uniform_scene.sample_rate_hz, 1.0)};
  signal::ComplexMatrix uniform_raw;
  if (!test_support::BuildReferenceRawHistory(uniform_scene, targets, &uniform_raw)) {
    return false;
  }

  test_support::ReferencePointScene jittered_scene = uniform_scene;
  std::vector<double> jittered_times;
  jittered_times.reserve(uniform_scene.pulses.size());
  const double interval_s = 1.0 / uniform_scene.prf_hz;
  const double start_x_m = uniform_scene.pulses.front().position_m.x_m;
  for (std::size_t index = 0U; index < uniform_scene.pulses.size(); ++index) {
    const double phase =
        2.0 * kPi * static_cast<double>(index) /
        static_cast<double>(uniform_scene.pulses.size() - 1U);
    const double time_s =
        uniform_scene.pulses[index].time_s + jitter_ratio * interval_s * std::sin(phase);
    if (index > 0U && time_s <= jittered_times.back()) {
      return false;
    }
    jittered_times.push_back(time_s);
    jittered_scene.pulses[index].time_s = time_s;
    jittered_scene.pulses[index].position_m.x_m =
        start_x_m + uniform_scene.platform_velocity_mps * time_s;
  }

  signal::ComplexMatrix jittered_raw;
  signal::ComplexMatrix resampled_raw;
  if (!test_support::BuildReferenceRawHistory(jittered_scene, targets, &jittered_raw) ||
      !ResampleRawHistorySlowTimeLinear(jittered_times, jittered_raw, &resampled_raw,
                                        &result->timing)) {
    return false;
  }

  FocusedSarImage uniform_focus;
  FocusedSarImage resampled_focus;
  const RdaConfig rda_config = test_support::MakeReferenceRdaConfig(uniform_scene, target_delay);
  if (!FocusStripmapRda(rda_config, uniform_raw, uniform_scene.matched_filter, &uniform_focus) ||
      !FocusStripmapRda(rda_config, resampled_raw, uniform_scene.matched_filter,
                        &resampled_focus)) {
    return false;
  }

  result->jitter_ratio = jitter_ratio;
  result->raw_nrms = UnitEnergyNrms(uniform_raw, resampled_raw);
  result->image_comparison =
      CompareImagesWithGlobalPhaseReference(uniform_focus.image, resampled_focus.image);
  return result->image_comparison.valid;
}

TEST(SarVariablePrfQualityMatrixTest, RecordsDeterministicJitterQualityTrend) {
  const std::vector<double> jitter_ratios{0.0, 0.05, 0.15, 0.35};
  std::vector<QualityCase> cases(jitter_ratios.size());
  for (std::size_t index = 0U; index < jitter_ratios.size(); ++index) {
    ASSERT_TRUE(EvaluateCase(jitter_ratios[index], &cases[index]));
    RecordProperty("jitter_ratio_" + std::to_string(index),
                   std::to_string(cases[index].jitter_ratio));
    RecordProperty("raw_nrms_" + std::to_string(index), std::to_string(cases[index].raw_nrms));
    RecordProperty("image_nrms_" + std::to_string(index),
                   std::to_string(cases[index].image_comparison.normalized_rms_error));
    RecordProperty("image_correlation_" + std::to_string(index),
                   std::to_string(cases[index].image_comparison.coherent_correlation));
  }

  EXPECT_NEAR(cases[0].raw_nrms, 0.0, 1.0e-12);
  EXPECT_NEAR(cases[0].image_comparison.normalized_rms_error, 0.0, 1.0e-12);
  EXPECT_NEAR(cases[0].image_comparison.coherent_correlation, 1.0, 1.0e-12);
  for (std::size_t index = 1U; index < cases.size(); ++index) {
    EXPECT_LT(cases[index - 1U].timing.maximum_abs_time_axis_deviation_s,
              cases[index].timing.maximum_abs_time_axis_deviation_s);
  }
  EXPECT_LT(cases[1].raw_nrms, cases[3].raw_nrms);
  EXPECT_LT(cases[1].image_comparison.normalized_rms_error,
            cases[3].image_comparison.normalized_rms_error);
  EXPECT_GT(cases[1].image_comparison.coherent_correlation,
            cases[3].image_comparison.coherent_correlation);
}

}  // namespace
}  // namespace imaging
}  // namespace sar
