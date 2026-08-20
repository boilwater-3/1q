#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "sar/imaging/SarImageQuality.h"
#include "sar/imaging/SarRda.h"
#include "sar/imaging/SarSlowTimeResamplingExecutor.h"
#include "support/sar_reference_scene.h"

namespace sar {
namespace imaging {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr std::size_t kCenterDelay = 28U;

enum class JitterKind {
  kUniform,
  kPeriodic,
  kRandom,
};

struct JitterProfile {
  const char* name;
  JitterKind kind;
  double ratio;
  std::uint64_t seed;
};

struct ExtendedQualityResult {
  double raw_nrms{0.0};
  ImageComparisonMetrics image{};
  SlowTimeResamplingExecutionResult execution{};
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

std::vector<double> BuildExplicitTimes(const test_support::ReferencePointScene& scene,
                                       const JitterProfile& profile) {
  std::vector<double> times;
  times.reserve(scene.pulses.size());
  const double interval_s = 1.0 / scene.prf_hz;
  std::uint64_t state = profile.seed;
  for (std::size_t index = 0U; index < scene.pulses.size(); ++index) {
    double shape = 0.0;
    if (index != 0U && index + 1U != scene.pulses.size()) {
      if (profile.kind == JitterKind::kPeriodic) {
        shape = std::sin(2.0 * kPi * static_cast<double>(index) /
                         static_cast<double>(scene.pulses.size() - 1U));
      } else if (profile.kind == JitterKind::kRandom) {
        shape = 2.0 * test_support::NextReferenceUniformOpen(&state) - 1.0;
      }
    }
    times.push_back(scene.pulses[index].time_s + profile.ratio * interval_s * shape);
  }
  return times;
}

bool Evaluate(const std::vector<echo::PointTarget>& targets, const JitterProfile& profile,
              std::uint64_t request_id, ExtendedQualityResult* result) {
  test_support::ReferencePointScene uniform_scene;
  if (result == nullptr || !test_support::BuildReferencePointScene(&uniform_scene)) {
    return false;
  }
  signal::ComplexMatrix uniform_raw;
  if (!test_support::BuildReferenceRawHistory(uniform_scene, targets, &uniform_raw)) {
    return false;
  }

  const std::vector<double> explicit_times = BuildExplicitTimes(uniform_scene, profile);
  test_support::ReferencePointScene jittered_scene = uniform_scene;
  const double start_x_m = uniform_scene.pulses.front().position_m.x_m;
  for (std::size_t index = 0U; index < explicit_times.size(); ++index) {
    jittered_scene.pulses[index].time_s = explicit_times[index];
    jittered_scene.pulses[index].position_m.x_m =
        start_x_m + uniform_scene.platform_velocity_mps * explicit_times[index];
  }
  signal::ComplexMatrix jittered_raw;
  if (!test_support::BuildReferenceRawHistory(jittered_scene, targets, &jittered_raw)) {
    return false;
  }

  SlowTimeResamplingRequest request;
  request.request_id = request_id;
  request.explicit_times_s = explicit_times;
  request.expected_interval_s = 1.0 / uniform_scene.prf_hz;
  request.raw_history = jittered_raw;
  result->execution = ExecuteSlowTimeResamplingRequest(request);
  const SlowTimeResamplingExecutionResult repeated = ExecuteSlowTimeResamplingRequest(request);
  if (result->execution.status != SlowTimeResamplingExecutionStatus::kSucceeded ||
      repeated.resampled_raw_history.values != result->execution.resampled_raw_history.values) {
    return false;
  }

  FocusedSarImage uniform_focus;
  FocusedSarImage resampled_focus;
  const RdaConfig rda_config = test_support::MakeReferenceRdaConfig(uniform_scene, kCenterDelay);
  if (!FocusStripmapRda(rda_config, uniform_raw, uniform_scene.matched_filter, &uniform_focus) ||
      !FocusStripmapRda(rda_config, result->execution.resampled_raw_history,
                        uniform_scene.matched_filter, &resampled_focus)) {
    return false;
  }
  result->raw_nrms = UnitEnergyNrms(uniform_raw, result->execution.resampled_raw_history);
  result->image =
      CompareImagesWithGlobalPhaseReference(uniform_focus.image, resampled_focus.image);
  return result->image.valid;
}

std::vector<echo::PointTarget> MakeTargets(const std::string& scene_name) {
  test_support::ReferencePointScene scene;
  EXPECT_TRUE(test_support::BuildReferencePointScene(&scene));
  if (scene_name == "e2_m4") {
    return {
        test_support::MakeReferenceTargetAtPosition(-0.2, 18U, scene.sample_rate_hz, 3.0),
        test_support::MakeReferenceTargetAtPosition(0.0, 20U, scene.sample_rate_hz, 2.0),
        test_support::MakeReferenceTargetAtPosition(0.2, 22U, scene.sample_rate_hz, 1.0),
    };
  }
  if (scene_name == "e3_high_doppler") {
    return {test_support::MakeReferenceTargetAtPosition(0.4, kCenterDelay,
                                                        scene.sample_rate_hz, 1.0)};
  }
  return {test_support::MakeReferenceTargetAtDelay(kCenterDelay, scene.sample_rate_hz, 1.0)};
}

TEST(SarExtendedVariablePrfQualityMatrixTest,
     RecordsMultiTargetHighDopplerAndRandomJitterQuality) {
  const std::vector<std::string> scenes{"e1_m1", "e2_m4", "e3_high_doppler"};
  const std::vector<JitterProfile> profiles{
      {"j0_uniform", JitterKind::kUniform, 0.0, 0U},
      {"j1_periodic_015", JitterKind::kPeriodic, 0.15, 0U},
      {"j2_random_005_seed17", JitterKind::kRandom, 0.05, 17U},
      {"j3_random_015_seed17", JitterKind::kRandom, 0.15, 17U},
      {"j4_random_015_seed29", JitterKind::kRandom, 0.15, 29U},
  };

  std::uint64_t request_id = 1U;
  for (const std::string& scene_name : scenes) {
    std::vector<ExtendedQualityResult> results(profiles.size());
    const std::vector<echo::PointTarget> targets = MakeTargets(scene_name);
    for (std::size_t profile_index = 0U; profile_index < profiles.size(); ++profile_index) {
      ASSERT_TRUE(Evaluate(targets, profiles[profile_index], request_id++, &results[profile_index]));
      const std::string prefix = scene_name + "_" + profiles[profile_index].name;
      RecordProperty(prefix + "_raw_nrms", std::to_string(results[profile_index].raw_nrms));
      RecordProperty(prefix + "_image_nrms",
                     std::to_string(results[profile_index].image.normalized_rms_error));
      RecordProperty(prefix + "_image_correlation",
                     std::to_string(results[profile_index].image.coherent_correlation));
      RecordProperty(prefix + "_maximum_gap_ratio",
                     std::to_string(results[profile_index].execution.gap_diagnostics.maximum_gap_ratio));
      EXPECT_EQ(results[profile_index].execution.reason, SlowTimeResamplingRejectionReason::kNone);
      EXPECT_TRUE(results[profile_index].execution.gap_diagnostics.resampling_allowed);
      EXPECT_GE(results[profile_index].raw_nrms, 0.0);
      EXPECT_GE(results[profile_index].image.normalized_rms_error, 0.0);
      EXPECT_LE(results[profile_index].image.normalized_rms_error, 2.0);
      EXPECT_GE(results[profile_index].image.coherent_correlation, -1.0);
      // 相关系数 = |cross|/sqrt(P·P) 理论 ≤1，浮点累加可产生 ~1e-15 越界。
      EXPECT_LE(results[profile_index].image.coherent_correlation, 1.0 + 1.0e-12);
    }
    EXPECT_NEAR(results[0].raw_nrms, 0.0, 1.0e-12);
    EXPECT_NEAR(results[0].image.normalized_rms_error, 0.0, 1.0e-12);
    EXPECT_NEAR(results[0].image.coherent_correlation, 1.0, 1.0e-12);
    EXPECT_LT(results[2].execution.resampling_diagnostics.maximum_abs_time_axis_deviation_s,
              results[3].execution.resampling_diagnostics.maximum_abs_time_axis_deviation_s);
  }
}

}  // namespace
}  // namespace imaging
}  // namespace sar
