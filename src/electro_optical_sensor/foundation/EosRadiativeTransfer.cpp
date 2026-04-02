#include "1q/electro_optical_sensor/foundation/EosRadiativeTransfer.h"

#include <algorithm>
#include <cmath>

#include "1q/electro_optical_sensor/foundation/EosPropagation.h"

namespace electro_optical_sensor {
namespace foundation {
namespace radiative_transfer {

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

RadiativeTransferResult EvaluateRadiativeTransfer(const RadiativeTransferInputs& inputs) {
  const float base_transmittance = Clamp01(inputs.base_transmittance);
  const float cloud_ratio = Clamp01(inputs.cloud_coverage_ratio);
  const float path_length_m = std::max(0.0f, inputs.path_length_m);
  const float aerosol_density_factor = std::max(1.0f, inputs.aerosol_density_factor);
  const float turbulence_factor = std::max(1.0f, inputs.turbulence_factor);

  const float safe_base_transmittance = std::max(base_transmittance, 1.0e-4f);
  const float base_extinction_coeff_per_m = -std::log(safe_base_transmittance) / 5000.0f;
  const float humidity_absorption_coeff_per_m = 2.5e-5f * cloud_ratio;
  const float aerosol_coeff_per_m = 1.0e-5f * (aerosol_density_factor - 1.0f);
  const float turbulence_coeff_per_m = 8.0e-6f * (turbulence_factor - 1.0f);

  float total_extinction_coeff_per_m = base_extinction_coeff_per_m + humidity_absorption_coeff_per_m;
  float path_radiance_penalty_scale = 1.0f;

  if (inputs.model == RadiativeTransferModel::kHumidityWeighted) {
    total_extinction_coeff_per_m = base_extinction_coeff_per_m +
                                   1.5f * humidity_absorption_coeff_per_m +
                                   aerosol_coeff_per_m;
    path_radiance_penalty_scale = 1.0f + 0.5f * cloud_ratio;
  } else if (inputs.model == RadiativeTransferModel::kAdaptivePathRadiance) {
    total_extinction_coeff_per_m = base_extinction_coeff_per_m +
                                   1.2f * humidity_absorption_coeff_per_m +
                                   aerosol_coeff_per_m + turbulence_coeff_per_m;
    const float path_km = path_length_m / 1000.0f;
    path_radiance_penalty_scale =
        1.0f + 0.3f * cloud_ratio + 0.02f * path_km * turbulence_factor;
  }

  RadiativeTransferResult result;
  result.total_extinction_coeff_per_m = std::max(0.0f, total_extinction_coeff_per_m);
  result.transmittance = propagation::ComputeBeerLambertTransmittance(
      result.total_extinction_coeff_per_m, path_length_m);
  result.path_radiance_penalty_scale =
      SafePositive(path_radiance_penalty_scale, 1.0f);
  return result;
}

}  // namespace radiative_transfer
}  // namespace foundation
}  // namespace electro_optical_sensor
