#include "electro_optical_sensor/foundation/EosRadiometry.h"

#include <algorithm>
#include <cmath>

#include "common/numerics/ClampUtils.h"
#include "common/numerics/Constants.h"
#include "electro_optical_sensor/foundation/EosPhysicalConstants.h"

namespace electro_optical_sensor {
namespace foundation {
namespace radiometry {

namespace {

constexpr float kPlanckConstant = 6.62607015e-34f;

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
  const float safe_wavelength_m = oneq::common::numerics::SafePositive(wavelength_um * 1.0e-6f, 4.0e-6f);
  const float safe_temperature_k = oneq::common::numerics::SafePositive(temperature_k, 290.0f);
  const float lambda5 = std::pow(safe_wavelength_m, 5.0f);
  if (!std::isfinite(lambda5) || lambda5 <= 0.0f) {
    return 0.0f;
  }

  const float c1 = 2.0f * kPlanckConstant * oneq::common::numerics::kLightSpeed * oneq::common::numerics::kLightSpeed;
  const float exponent =
      (kPlanckConstant * oneq::common::numerics::kLightSpeed) / (safe_wavelength_m * oneq::common::numerics::kBoltzmann * safe_temperature_k);
  const float exp_value = std::exp(std::min(exponent, 80.0f));
  const float denominator = lambda5 * std::max(exp_value - 1.0f, 1.0e-12f);
  if (!std::isfinite(denominator) || denominator <= 0.0f) {
    return 0.0f;
  }
  return c1 / denominator;
}

float IntegrateSpectralRadianceOverBand(float spectral_radiance_w_sr_m3,
                                        float wavelength_bandwidth_um) {
  const float safe_spectral_radiance = std::max(0.0f, spectral_radiance_w_sr_m3);
  const float safe_bandwidth_m = oneq::common::numerics::SafePositive(wavelength_bandwidth_um * 1.0e-6f, 1.0e-6f);
  return safe_spectral_radiance * safe_bandwidth_m;
}

float ComputeInfraredRadianceDelta(const InfraredRadianceInputs& inputs) {
  const float emissivity = oneq::common::numerics::Clamp01(inputs.emissivity);
  const float target_radiance =
      emissivity * ComputePlanckRadiance(inputs.wavelength_um, inputs.target_temperature_k);
  const float background_radiance =
      ComputePlanckRadiance(inputs.wavelength_um, inputs.background_temperature_k);
  return target_radiance - background_radiance;
}

float ComputeVisibleLambertianRadiance(const VisibleRadianceInputs& inputs) {
  const float reflectance = oneq::common::numerics::Clamp01(inputs.reflectance);
  const float cloud_coverage = oneq::common::numerics::Clamp01(inputs.cloud_coverage_ratio);
  const float effective_irradiance =
      std::max(0.0f, inputs.solar_irradiance_w_m2) * ComputeIlluminationScale(inputs.illumination);
  const float solar_altitude_rad =
      inputs.solar_altitude_deg * constants::kPi / 180.0f;
  const float solar_geometry_gain = std::max(0.0f, std::sin(solar_altitude_rad));
  const float cloud_gain = 1.0f - 0.7f * cloud_coverage;

  return reflectance * effective_irradiance * cloud_gain * solar_geometry_gain /
         constants::kPi;
}

VisibleChannelResult ComputeVisibleChannelResult(const VisibleChannelInputs& inputs) {
  VisibleChannelResult result;
  result.target_radiance = ComputeVisibleLambertianRadiance(inputs.target);

  VisibleRadianceInputs background_inputs = inputs.target;
  background_inputs.reflectance =
      oneq::common::numerics::Clamp01(inputs.background_reflectance);
  result.background_radiance = ComputeVisibleLambertianRadiance(background_inputs);
  result.normalized_contrast =
      ComputeRelativeContrast(result.target_radiance, result.background_radiance);
  return result;
}

float ComputeRelativeContrast(float target_radiance, float background_radiance) {
  const float safe_background = std::max(std::fabs(background_radiance), 1.0e-12f);
  return (target_radiance - background_radiance) / safe_background;
}

}  // namespace radiometry
}  // namespace foundation
}  // namespace electro_optical_sensor
