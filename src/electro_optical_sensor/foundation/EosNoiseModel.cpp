#include "electro_optical_sensor/foundation/EosNoiseModel.h"

#include <algorithm>
#include <cmath>

#include "common/numerics/ClampUtils.h"

namespace electro_optical_sensor {
namespace foundation {
namespace noise {

namespace {

constexpr float kEquivalentNoiseGuardW = 1.0e-12f;
constexpr float kCloudScatteringGain = 0.8f;
constexpr float kShotNoiseScale = 1.0e-7f;
constexpr float kThermalNoiseScale = 0.02f;
constexpr float kMeanNoiseFluxRatio = 0.08f;
constexpr float kSuppressionBase = 0.05f;
constexpr float kSuppressionAdaptiveGain = 0.4f;
constexpr float kSuppressionLowerBound = 0.05f;
constexpr float kSuppressionUpperBound = 0.60f;

}  // namespace

BackgroundNoiseStatistics ComputeBackgroundNoiseStatistics(
    const BackgroundNoiseModelInputs& inputs) {
  const float background_flux_w = std::max(0.0f, inputs.background_flux_w);
  const float cloud_ratio = oneq::common::numerics::Clamp01(inputs.cloud_coverage_ratio);
  const float scene_complexity = std::max(1.0f, inputs.scene_complexity_factor);
  const float photon_noise_factor = std::max(1.0f, inputs.photon_noise_enhancement_factor);
  const float detector_area_cm2 = oneq::common::numerics::SafePositive(inputs.detector_area_cm2, 0.25f);
  const float integration_time_sec = oneq::common::numerics::SafePositive(inputs.integration_time_sec, 1.0f / 30.0f);
  const float electrical_bandwidth_hz = oneq::common::numerics::SafePositive(inputs.electrical_bandwidth_hz, 1000.0f);
  // Equivalent noise bandwidth of a first-order RC low-pass (single pole). This approximation
  // assumes stationary noise in the frame integration window and is intended for frame-level
  // EOS detection modeling, not high-fidelity transient front-end simulation.
  const float effective_bandwidth_hz =
      electrical_bandwidth_hz / (1.0f + integration_time_sec * electrical_bandwidth_hz);
  const float cloud_gain = 1.0f + kCloudScatteringGain * cloud_ratio;

  BackgroundNoiseStatistics stats;
  stats.mean_noise_power_w = background_flux_w * kMeanNoiseFluxRatio * cloud_gain * scene_complexity;

  const float shot_sigma =
      std::sqrt(background_flux_w * effective_bandwidth_hz) * kShotNoiseScale *
      std::sqrt(detector_area_cm2) * photon_noise_factor;
  const float thermal_sigma = background_flux_w * kThermalNoiseScale / std::sqrt(integration_time_sec);
  stats.sigma_noise_power_w = (shot_sigma + thermal_sigma) * cloud_gain * scene_complexity;
  stats.equivalent_noise_power_w = stats.mean_noise_power_w + 3.0f * stats.sigma_noise_power_w;

  const float suppression_reference = background_flux_w + kEquivalentNoiseGuardW;
  const float adaptive_term = stats.equivalent_noise_power_w / suppression_reference;
  stats.suppression_weight = oneq::common::numerics::Clamp(
      kSuppressionBase + kSuppressionAdaptiveGain * adaptive_term, kSuppressionLowerBound,
      kSuppressionUpperBound);
  return stats;
}

float ComputeEffectiveSignalPowerW(float received_power_w, float background_flux_w,
                                   const BackgroundNoiseStatistics& stats) {
  const float safe_received_power_w = std::max(0.0f, received_power_w);
  const float safe_background_flux_w = std::max(0.0f, background_flux_w);
  const float suppression_weight =
      oneq::common::numerics::Clamp(stats.suppression_weight, kSuppressionLowerBound,
                                      kSuppressionUpperBound);

  const float suppressed_background_budget_w = suppression_weight * safe_background_flux_w;
  const float suppressed_background_w =
      std::min(suppressed_background_budget_w, 0.95f * safe_received_power_w);
  const float effective_signal_w = safe_received_power_w - suppressed_background_w;
  return std::max(0.0f, effective_signal_w);
}

}  // namespace noise
}  // namespace foundation
}  // namespace electro_optical_sensor
