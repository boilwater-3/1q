#include "1q/electro_optical_sensor/foundation/EosRadiometry.h"

#include <algorithm>
#include <cmath>

namespace electro_optical_sensor {
namespace foundation {
namespace radiometry {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kPlanckConstant = 6.62607015e-34f;
constexpr float kLightSpeed = 2.99792458e8f;
constexpr float kBoltzmannConstant = 1.380649e-23f;

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

float ComputeIlluminationScale(IlluminationCondition illumination) {
  if (illumination == IlluminationCondition::kNight) {
    return 0.05f;
  }
  if (illumination == IlluminationCondition::kTwilight) {
    return 0.25f;
  }
  return 1.0f;
}

}  // namespace

float ComputePlanckRadiance(float wavelength_um, float temperature_k) {
  const float safe_wavelength_m = SafePositive(wavelength_um * 1.0e-6f, 4.0e-6f);
  const float safe_temperature_k = SafePositive(temperature_k, 290.0f);
  const float lambda5 = std::pow(safe_wavelength_m, 5.0f);
  if (lambda5 <= 0.0f) {
    return 0.0f;
  }

  const float c1 = 2.0f * kPlanckConstant * kLightSpeed * kLightSpeed;
  const float exponent =
      (kPlanckConstant * kLightSpeed) / (safe_wavelength_m * kBoltzmannConstant * safe_temperature_k);
  const float exp_value = std::exp(std::min(exponent, 80.0f));
  const float denominator = lambda5 * std::max(exp_value - 1.0f, 1.0e-12f);
  return c1 / denominator;
}

float ComputeInfraredRadianceDelta(const InfraredRadianceInputs& inputs) {
  const float emissivity = Clamp01(inputs.emissivity);
  const float target_radiance =
      emissivity * ComputePlanckRadiance(inputs.wavelength_um, inputs.target_temperature_k);
  const float background_radiance =
      ComputePlanckRadiance(inputs.wavelength_um, inputs.background_temperature_k);
  return target_radiance - background_radiance;
}

float ComputeVisibleLambertianRadiance(const VisibleRadianceInputs& inputs) {
  const float range_m = SafePositive(inputs.range_m, 1000.0f);
  const float reflectance = Clamp01(inputs.reflectance);
  const float transmittance = Clamp01(inputs.atmospheric_transmittance);
  const float cloud_coverage = Clamp01(inputs.cloud_coverage_ratio);
  const float projected_area_m2 = SafePositive(inputs.projected_area_m2, 1.0f);
  const float effective_irradiance =
      std::max(0.0f, inputs.solar_irradiance_w_m2) * ComputeIlluminationScale(inputs.illumination);
  const float solar_altitude_rad = inputs.solar_altitude_deg * kPi / 180.0f;
  const float solar_geometry_gain = std::max(0.0f, std::sin(solar_altitude_rad));
  const float cloud_gain = 1.0f - 0.7f * cloud_coverage;
  const float range_gain = 1.0f / (1.0f + range_m / 5000.0f);

  return reflectance * projected_area_m2 * effective_irradiance * transmittance * cloud_gain *
         solar_geometry_gain * range_gain / kPi;
}

float ComputeRelativeContrast(float target_radiance, float background_radiance) {
  const float safe_background = std::max(std::fabs(background_radiance), 1.0e-12f);
  return (target_radiance - background_radiance) / safe_background;
}

}  // namespace radiometry
}  // namespace foundation
}  // namespace electro_optical_sensor
