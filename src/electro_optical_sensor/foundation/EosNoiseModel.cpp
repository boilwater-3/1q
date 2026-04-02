#include "1q/electro_optical_sensor/foundation/EosNoiseModel.h"

#include <algorithm>
#include <cmath>

namespace electro_optical_sensor {
namespace foundation {
namespace noise {

namespace {

float Clamp(float value, float lower, float upper) {
  return std::max(lower, std::min(value, upper));
}

float Clamp01(float value) { return Clamp(value, 0.0f, 1.0f); }

float SafePositive(float value, float fallback) {
  if (std::isfinite(value) == 0 || value <= 0.0f) {
    return fallback;
  }
  return value;
}

}  // namespace

BackgroundNoiseStatistics ComputeBackgroundNoiseStatistics(
    const BackgroundNoiseModelInputs& inputs) {
  const float background_flux_w = std::max(0.0f, inputs.background_flux_w);
  const float cloud_ratio = Clamp01(inputs.cloud_coverage_ratio);
  const float scene_complexity = std::max(1.0f, inputs.scene_complexity_factor);
  const float photon_noise_factor = std::max(1.0f, inputs.photon_noise_enhancement_factor);
  const float detector_area_cm2 = SafePositive(inputs.detector_area_cm2, 0.25f);
  const float integration_time_sec = SafePositive(inputs.integration_time_sec, 1.0f / 30.0f);
  const float electrical_bandwidth_hz = SafePositive(inputs.electrical_bandwidth_hz, 1000.0f);
  const float effective_bandwidth_hz =
      electrical_bandwidth_hz / (1.0f + integration_time_sec * electrical_bandwidth_hz);
  const float cloud_gain = 1.0f + 0.8f * cloud_ratio;

  BackgroundNoiseStatistics stats;
  stats.mean_noise_power_w = background_flux_w * 0.08f * cloud_gain * scene_complexity;

  const float shot_sigma =
      std::sqrt(background_flux_w * effective_bandwidth_hz) * 1.0e-7f *
      std::sqrt(detector_area_cm2) * photon_noise_factor;
  const float thermal_sigma = background_flux_w * 0.02f / std::sqrt(integration_time_sec);
  stats.sigma_noise_power_w = (shot_sigma + thermal_sigma) * cloud_gain * scene_complexity;
  stats.equivalent_noise_power_w = stats.mean_noise_power_w + 3.0f * stats.sigma_noise_power_w;

  const float suppression_reference = background_flux_w + 1.0e-12f;
  const float adaptive_term = stats.equivalent_noise_power_w / suppression_reference;
  stats.suppression_weight = Clamp(0.05f + 0.4f * adaptive_term, 0.05f, 0.60f);
  return stats;
}

float ComputeEffectiveSignalPowerW(float received_power_w, float background_flux_w,
                                   const BackgroundNoiseStatistics& stats) {
  const float safe_received_power_w = std::max(0.0f, received_power_w);
  const float safe_background_flux_w = std::max(0.0f, background_flux_w);
  const float suppression_weight = Clamp(stats.suppression_weight, 0.05f, 0.60f);

  const float suppressed_background_budget_w = suppression_weight * safe_background_flux_w;
  const float suppressed_background_w =
      std::min(suppressed_background_budget_w, 0.95f * safe_received_power_w);
  const float effective_signal_w = safe_received_power_w - suppressed_background_w;
  return std::max(0.0f, effective_signal_w);
}

}  // namespace noise
}  // namespace foundation
}  // namespace electro_optical_sensor
