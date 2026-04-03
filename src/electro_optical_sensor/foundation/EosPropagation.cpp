#include "electro_optical_sensor/foundation/EosPropagation.h"

#include <algorithm>
#include <cmath>

namespace electro_optical_sensor {
namespace foundation {
namespace propagation {

namespace {

constexpr float kFourPi = 12.56637061435917295f;

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

float ComputeBeerLambertTransmittance(float attenuation_coeff_per_m, float path_length_m) {
  const float safe_coeff_per_m = std::max(0.0f, attenuation_coeff_per_m);
  const float safe_path_length_m = std::max(0.0f, path_length_m);
  return std::exp(-safe_coeff_per_m * safe_path_length_m);
}

float ComputeAtmosphericTransmittance(float base_extinction_coeff_per_m,
                                      float humidity_absorption_coeff_per_m,
                                      float path_length_m) {
  const float total_coeff_per_m =
      std::max(0.0f, base_extinction_coeff_per_m) + std::max(0.0f, humidity_absorption_coeff_per_m);
  return ComputeBeerLambertTransmittance(total_coeff_per_m, path_length_m);
}

float ComputeBackgroundFluxW(float background_radiance_w_sr_m2, float aperture_area_m2,
                             float fov_solid_angle_sr, float optical_transmittance) {
  const float safe_radiance = std::max(0.0f, background_radiance_w_sr_m2);
  const float safe_aperture_area_m2 = SafePositive(aperture_area_m2, 1.0e-4f);
  const float safe_fov_solid_angle_sr = std::max(0.0f, fov_solid_angle_sr);
  return safe_radiance * safe_aperture_area_m2 * safe_fov_solid_angle_sr *
         Clamp01(optical_transmittance);
}

float ComputeReceivedPowerW(float source_radiance_w_sr_m2, float projected_area_m2, float range_m,
                            float aperture_area_m2, float atmospheric_transmittance,
                            float optical_transmittance) {
  const float safe_source_radiance = std::max(0.0f, source_radiance_w_sr_m2);
  const float safe_projected_area_m2 = SafePositive(projected_area_m2, 1.0f);
  const float safe_range_m = SafePositive(range_m, 1.0f);
  const float safe_aperture_area_m2 = SafePositive(aperture_area_m2, 1.0e-4f);
  const float geometric_loss = kFourPi * safe_range_m * safe_range_m;
  return safe_source_radiance * safe_projected_area_m2 * safe_aperture_area_m2 *
         Clamp01(atmospheric_transmittance) * Clamp01(optical_transmittance) / geometric_loss;
}

float ComputeSnrLinear(float received_power_w, float nep_w) {
  const float safe_received_power_w = std::max(0.0f, received_power_w);
  const float safe_nep_w = SafePositive(nep_w, 1.0e-12f);
  return safe_received_power_w / safe_nep_w;
}

float ComputeSnrDb(float snr_linear) {
  const float safe_snr_linear = std::max(snr_linear, 1.0e-12f);
  return 10.0f * std::log10(safe_snr_linear);
}

float ComputeNepW(const NepNoiseModelInputs& inputs) {
  const float safe_detectivity = SafePositive(inputs.detector_detectivity_cm_sqrt_hz_per_w, 1.0e10f);
  const float safe_detector_area_cm2 = SafePositive(inputs.detector_area_cm2, 0.25f);
  const float safe_bandwidth_hz = SafePositive(inputs.electrical_bandwidth_hz, 1000.0f);
  const float safe_integration_time_sec = SafePositive(inputs.integration_time_sec, 1.0f / 30.0f);
  const float effective_bandwidth_hz = safe_bandwidth_hz / (1.0f + safe_integration_time_sec * safe_bandwidth_hz);
  const float optical_factor = std::max(Clamp01(inputs.optical_transmittance), 1.0e-3f);
  const float system_noise_factor = std::max(inputs.system_noise_factor, 1.0f);

  return system_noise_factor * std::sqrt(safe_detector_area_cm2 * effective_bandwidth_hz) /
         (safe_detectivity * optical_factor);
}

SnrEvaluationResult EvaluateSnrWithNep(float received_power_w, const NepNoiseModelInputs& inputs) {
  SnrEvaluationResult result;
  result.nep_w = ComputeNepW(inputs);
  result.snr_linear = ComputeSnrLinear(received_power_w, result.nep_w);
  result.snr_db = ComputeSnrDb(result.snr_linear);
  return result;
}

}  // namespace propagation
}  // namespace foundation
}  // namespace electro_optical_sensor
